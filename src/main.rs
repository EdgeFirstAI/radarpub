use async_std::{net::UdpSocket, task::block_on};
use cdr::{CdrLe, Infinite};
use clap::Parser;
use drvegrd::{can::read_message, eth::RadarCubeReader};
use kanal::{bounded_async as channel, AsyncSender as Sender};
use log::{debug, error, trace, warn};
use socketcan::async_std::CanSocket;
use std::{
    error::Error,
    f32::consts::PI,
    str::FromStr as _,
    sync::Arc,
    thread,
    time::{Instant, SystemTime},
};
use unix_ts::Timestamp;
use zenoh::{config::Config, prelude::r#async::*};
use zenoh_ros_type::{
    common_interfaces::sensor_msgs::{PointCloud2, PointField},
    edgefirst_msgs,
    rcl_interfaces::builtin_interfaces::Time as ROSTime,
};

#[derive(Debug)]
#[allow(dead_code)]
pub enum PointFieldType {
    INT8 = 1,
    UINT8 = 2,
    INT16 = 3,
    UINT16 = 4,
    INT32 = 5,
    UINT32 = 6,
    FLOAT32 = 7,
    FLOAT64 = 8,
}

#[derive(Parser, Debug, Clone)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// zenoh connection mode
    #[arg(long, default_value = "client")]
    mode: String,

    /// connect to endpoint
    #[arg(short, long, default_value = "tcp/127.0.0.1:7447")]
    endpoint: Vec<String>,

    /// ros topic
    #[arg(long, default_value = "rt/radar/targets")]
    targets_topic: String,

    /// ros topic
    #[arg(long, default_value = "rt/radar/cube")]
    cube_topic: String,

    /// can device connected to radar
    #[arg(long, default_value = "can0")]
    can: String,

    /// mirror the radar data
    #[arg(long)]
    mirror: bool,

    /// Center the doppler axis in the cube.
    #[arg(short = 'd', long)]
    center_doppler: bool,
}

/// The port5 implementation on Linux uses the recvmmsg system call to enable
/// bulk reads of UDP packets.  This is not available on other platforms.
#[cfg(target_os = "linux")]
async fn port5(tx: Sender<Vec<u8>>) -> Result<(), Box<dyn std::error::Error>> {
    use std::os::fd::AsRawFd;

    const VLEN: usize = 50;
    const BUFLEN: usize = 1500;
    const RETRY_TIME: std::time::Duration = std::time::Duration::from_micros(250);

    let mut vlen_history = [0; 1000];
    let mut vlen_index = 0;

    let sock = UdpSocket::bind("0.0.0.0:50005").await.unwrap();
    let mut mmsgs: [libc::mmsghdr; VLEN] = unsafe { std::mem::zeroed() };
    let mut iovecs: [libc::iovec; VLEN] = unsafe { std::mem::zeroed() };
    let mut bufs: [[u8; BUFLEN]; VLEN] = [[0; BUFLEN]; VLEN];

    loop {
        for i in 0..VLEN {
            iovecs[i].iov_base = bufs[i].as_mut_ptr() as *mut libc::c_void;
            iovecs[i].iov_len = BUFLEN;
            mmsgs[i].msg_hdr.msg_iov = &mut iovecs[i];
            mmsgs[i].msg_hdr.msg_iovlen = 1;
            mmsgs[i].msg_hdr.msg_name = std::ptr::null_mut();
            mmsgs[i].msg_hdr.msg_namelen = 0;
            mmsgs[i].msg_hdr.msg_control = std::ptr::null_mut();
            mmsgs[i].msg_hdr.msg_controllen = 0;
            mmsgs[i].msg_hdr.msg_flags = 0;
            mmsgs[i].msg_len = 0;
        }

        match unsafe {
            libc::recvmmsg(
                sock.as_raw_fd(),
                mmsgs.as_mut_ptr(),
                VLEN as u32,
                0,
                std::ptr::null_mut(),
            )
        } {
            -1 => {
                let err = std::io::Error::last_os_error();
                match err.kind() {
                    std::io::ErrorKind::Interrupted => (),
                    std::io::ErrorKind::WouldBlock => thread::sleep(RETRY_TIME),
                    _ => error!("port5 error: {:?}", err),
                }
            }
            n => {
                vlen_history[vlen_index] = n;
                vlen_index += 1;
                if vlen_index == vlen_history.len() {
                    vlen_index = 0;
                    let avg = vlen_history.iter().sum::<i32>() / vlen_history.len() as i32;
                    let min = vlen_history.iter().min().unwrap();
                    let max = vlen_history.iter().max().unwrap();
                    trace!("recvmmsg avg={} min={} max={}", avg, min, max);
                }

                for buf in bufs.iter().take(n as usize) {
                    match tx.send(buf.to_vec()).await {
                        Ok(_) => (),
                        Err(e) => error!("port5 error: {:?}", e),
                    }
                }
            }
        }
    }
}

#[cfg(not(target_os = "linux"))]
async fn port5(tx: Sender<Vec<u8>>) -> Result<(), Box<dyn std::error::Error>> {
    let sock = UdpSocket::bind("0.0.0.0:50005").await.unwrap();
    let mut buf = [0; 1500];

    loop {
        match sock.recv_from(&mut buf).await {
            Ok((n, _)) => match tx.send(buf[..n].to_vec()).await {
                Ok(_) => (),
                Err(e) => error!("port5 error: {:?}", e),
            },
            Err(e) => error!("port5 error: {:?}", e),
        }
    }
}

async fn port63(tx: Sender<Vec<u8>>) -> Result<(), Box<dyn std::error::Error>> {
    let sock = UdpSocket::bind("0.0.0.0:50063").await.unwrap();
    let mut buf = [0; 1500];

    loop {
        match sock.recv_from(&mut buf).await {
            Ok((n, _)) => match tx.send(buf[..n].to_vec()).await {
                Ok(_) => (),
                Err(e) => error!("port63 error: {:?}", e),
            },
            Err(e) => error!("port63 error: {:?}", e),
        }
    }
}

async fn udp_loop(
    session: Arc<Session>,
    topic: &String,
    center_doppler: bool,
) -> Result<(), Box<dyn std::error::Error>> {
    let (tx5, rx) = channel(10000);
    let tx63 = tx5.clone();

    thread::Builder::new()
        .name("port5".to_string())
        .spawn(move || block_on(port5(tx5)).unwrap())?;

    thread::Builder::new()
        .name("port63".to_string())
        .spawn(move || block_on(port63(tx63)).unwrap())?;

    let mut reader = RadarCubeReader::default();

    let mut cube_times = [0; 1080];
    let mut cube_index = 0;
    let mut dropped = 0;
    let mut prev_time = Instant::now();

    loop {
        let msg = match rx.recv().await {
            Ok(msg) => msg,
            Err(e) => {
                error!("recv error: {:?}", e);
                continue;
            }
        };

        match reader.read(&msg, center_doppler) {
            Ok(Some(cubemsg)) => {
                cube_times[cube_index] = prev_time.elapsed().as_millis() as i32;
                prev_time = Instant::now();
                cube_index += 1;

                if cube_index == cube_times.len() {
                    cube_index = 0;
                    let avg = cube_times.iter().sum::<i32>() / cube_times.len() as i32;
                    let min = cube_times.iter().min().unwrap();
                    let max = cube_times.iter().max().unwrap();
                    let fps = 1000.0 / (avg as f32);
                    let droprate = (dropped as f32) / cube_times.len() as f32;
                    dropped = 0;

                    if droprate > 0.05 {
                        error!(
                            "cube fps={} avg={} min={} max={} droprate={:.2}%",
                            fps,
                            avg,
                            min,
                            max,
                            droprate * 100.0
                        );
                    } else if droprate > 0.025 {
                        warn!(
                            "cube fps={} avg={} min={} max={} droprate={:.2}%",
                            fps,
                            avg,
                            min,
                            max,
                            droprate * 100.0
                        );
                    } else {
                        debug!(
                            "cube fps={} avg={} min={} max={} droprate={:.2}%",
                            fps,
                            avg,
                            min,
                            max,
                            droprate * 100.0
                        );
                    }
                }

                let ts = SystemTime::now().duration_since(SystemTime::UNIX_EPOCH)?;
                let ts = Timestamp::from_nanos(ts.as_nanos() as i128);

                let layout = vec![
                    edgefirst_msgs::radar_cube_dimension::SEQUENCE,
                    edgefirst_msgs::radar_cube_dimension::RANGE,
                    edgefirst_msgs::radar_cube_dimension::RXCHANNEL,
                    edgefirst_msgs::radar_cube_dimension::DOPPLER,
                ];

                // Double the final dimension to account for complex data.
                let shape = cubemsg.data.shape();
                let shape = vec![
                    shape[0] as u16,
                    shape[1] as u16,
                    shape[2] as u16,
                    shape[3] as u16 * 2,
                ];

                // Cast the Complex<i16> vector to a i16 vector.
                let data = cubemsg.data.into_raw_vec();
                let data2 = unsafe {
                    Vec::from_raw_parts(data.as_ptr() as *mut i16, data.len() * 2, data.len() * 2)
                };
                std::mem::forget(data);

                let msg = edgefirst_msgs::RadarCube {
                    header: zenoh_ros_type::std_msgs::Header {
                        stamp: ROSTime {
                            sec: ts.seconds() as i32,
                            nanosec: ts.subsec(9),
                        },
                        frame_id: "".to_string(),
                    },
                    timestamp: cubemsg.timestamp,
                    frame_id: cubemsg.frame_counter,
                    layout,
                    shape,
                    scales: vec![
                        cubemsg.bin_properties.speed_per_bin,
                        cubemsg.bin_properties.range_per_bin,
                        cubemsg.bin_properties.bin_per_speed,
                    ],
                    cube: data2,
                    is_complex: true,
                };

                let encoded = cdr::serialize::<_, _, CdrLe>(&msg, Infinite)?;

                match session
                    .put(topic, encoded)
                    .encoding(Encoding::WithSuffix(
                        KnownEncoding::AppOctetStream,
                        "sensor_msgs/msg/RadCube".into(),
                    ))
                    .res()
                    .await
                {
                    Ok(_) => trace!("RadarCube Message Sent"),
                    Err(e) => error!("RadarCube Message Error: {:?}", e),
                }
            }
            Ok(None) => (),
            Err(err) => {
                // Ignore errors related to dropped packets.  We do measure the
                // rate of dropped packets for logging purposes.
                dropped += 1;
                match err {
                    drvegrd::eth::SMSError::MissingCubeData(_, _) => (),
                    drvegrd::eth::SMSError::CubeHeaderMissing => (),
                    _ => error!("Cube Error: {:?}", err),
                }
            }
        }
    }
}

#[async_std::main]
async fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();

    env_logger::init();

    let mut config = Config::default();
    let mode = WhatAmI::from_str(&args.mode).unwrap();
    config.set_mode(Some(mode)).unwrap();
    config.connect.endpoints = args.endpoint.iter().map(|v| v.parse().unwrap()).collect();
    let _ = config.scouting.multicast.set_enabled(Some(false));
    let session = zenoh::open(config.clone())
        .res_async()
        .await
        .unwrap()
        .into_arc();
    log::info!("Opened Zenoh session");

    let sock = CanSocket::open(&args.can)?;
    let cube_session = session.clone();

    thread::Builder::new()
        .name("radarcube".to_string())
        .spawn(move || {
            block_on(udp_loop(
                cube_session,
                &args.cube_topic,
                args.center_doppler,
            ))
            .unwrap();
        })?;

    loop {
        match read_message(&sock).await {
            Err(err) => println!("Error: {:?}", err),
            Ok(frame) => {
                trace!(
                    "Processing radar frame with {} targets",
                    frame.header.n_targets
                );
                let data: Vec<_> = (0..frame.header.n_targets)
                    .flat_map(|idx| {
                        let tgt = &frame.targets[idx];
                        let xyz = transform_xyz(
                            tgt.range as f32,
                            tgt.azimuth as f32,
                            tgt.elevation as f32,
                            args.mirror,
                        );
                        [
                            xyz[0],
                            xyz[1],
                            xyz[2],
                            tgt.speed as f32,
                            tgt.power as f32,
                            tgt.rcs as f32,
                        ]
                    })
                    .flat_map(|elem| elem.to_ne_bytes())
                    .collect();
                let fields = vec![
                    PointField {
                        name: String::from("x"),
                        offset: 0,
                        datatype: PointFieldType::FLOAT32 as u8,
                        count: 1,
                    },
                    PointField {
                        name: String::from("y"),
                        offset: 4,
                        datatype: PointFieldType::FLOAT32 as u8,
                        count: 1,
                    },
                    PointField {
                        name: String::from("z"),
                        offset: 8,
                        datatype: PointFieldType::FLOAT32 as u8,
                        count: 1,
                    },
                    PointField {
                        name: String::from("speed"),
                        offset: 12,
                        datatype: PointFieldType::FLOAT32 as u8,
                        count: 1,
                    },
                    PointField {
                        name: String::from("power"),
                        offset: 16,
                        datatype: PointFieldType::FLOAT32 as u8,
                        count: 1,
                    },
                    PointField {
                        name: String::from("rcs"),
                        offset: 20,
                        datatype: PointFieldType::FLOAT32 as u8,
                        count: 1,
                    },
                ];

                let ts = SystemTime::now().duration_since(SystemTime::UNIX_EPOCH)?;
                let ts = Timestamp::from_nanos(ts.as_nanos() as i128);

                let msg = PointCloud2 {
                    header: zenoh_ros_type::std_msgs::Header {
                        stamp: ROSTime {
                            sec: ts.seconds() as i32,
                            nanosec: ts.subsec(9),
                        },
                        frame_id: "".to_string(),
                    },
                    height: 1,
                    width: frame.header.n_targets as u32,
                    fields,
                    is_bigendian: false,
                    point_step: 24,
                    row_step: 24 * frame.header.n_targets as u32,
                    data,
                    is_dense: true,
                };
                let encoded = cdr::serialize::<_, _, CdrLe>(&msg, Infinite)?;

                match session
                    .put(&args.targets_topic, encoded)
                    .encoding(Encoding::WithSuffix(
                        KnownEncoding::AppOctetStream,
                        "sensor_msgs/msg/PointCloud2".into(),
                    ))
                    .res()
                    .await
                {
                    Ok(_) => trace!("PointCloud2 Message Sent"),
                    Err(e) => error!("PointCloud2 Message Error: {:?}", e),
                }
            }
        }
    }
}

fn transform_xyz(range: f32, azimuth: f32, elevation: f32, mirror: bool) -> [f32; 3] {
    let azi = azimuth / 180.0 * PI;
    let ele = elevation / 180.0 * PI;
    let x = range * ele.cos() * azi.cos();
    let y = range * ele.cos() * azi.sin();
    let z = range * ele.sin();
    if mirror { [x, -y, z] } else { [x, y, z] }
}
