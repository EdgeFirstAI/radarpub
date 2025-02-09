mod can;
mod eth;

use async_std::{net::UdpSocket, task::block_on};
use clap::Parser;
use eth::{RadarCube, RadarCubeReader, SMSError, TransportHeaderSlice, SMS_PACKET_SIZE};
use kanal::{bounded_async as channel, AsyncSender as Sender};
use log::{debug, error, trace, warn};
use ndarray::{s, Array2};
use ndarray_npy::write_npy;
use num::complex::Complex32;
use rerun::{external::re_sdk_comms::DEFAULT_SERVER_PORT, RecordingStream};
use std::{
    fs::File,
    net::{Ipv4Addr, SocketAddr},
    thread,
    time::Instant,
};

mod common;

#[derive(Parser, Debug, Clone)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// connect to remote rerun viewer at this address
    #[arg(short, long)]
    connect: Option<Ipv4Addr>,

    /// record rerun data to file instead of live viewer
    #[arg(short, long)]
    record: Option<String>,

    /// launch local rerun viewer
    #[arg(short, long)]
    viewer: bool,

    /// use this port for the rerun viewer (remote or web server)
    #[arg(short, long)]
    port: Option<u16>,

    /// Save Numpy files to this directory, one per frame.
    #[arg(short, long)]
    numpy: Option<String>,

    /// Read from a pcapng file instead of a live interface.
    #[arg()]
    pcap: Option<String>,

    /// Enable radar data cube streaming.
    #[arg(long)]
    cube: bool,

    /// Open can interface for target data.
    #[cfg(feature = "can")]
    #[arg(long)]
    device: Option<String>,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    let args = Args::parse();

    let rr = if let Some(addr) = args.connect {
        let port = args.port.unwrap_or(DEFAULT_SERVER_PORT);
        let remote = SocketAddr::new(addr.into(), port);
        Some(
            rerun::RecordingStreamBuilder::new("radarview")
                .connect_opts(remote, rerun::default_flush_timeout())?,
        )
    } else if let Some(record) = args.record {
        Some(rerun::RecordingStreamBuilder::new("radarview").save(record)?)
    } else if args.viewer {
        Some(rerun::RecordingStreamBuilder::new("radarview").spawn()?)
    } else {
        None
    };

    if let Some(pcap) = args.pcap {
        pcap_loop(&rr, &pcap, &args.numpy)?;
    } else {
        #[cfg(feature = "can")]
        if let Some(device) = args.device {
            let rr2 = rr.clone();

            if args.cube {
                let cube_thread = thread::Builder::new()
                    .name("radarcube".to_string())
                    .spawn(move || block_on(udp_loop(&rr, &args.numpy)).unwrap())?;
                block_on(can_loop(&rr2, Some(device)));
                cube_thread.join().unwrap();
            } else {
                block_on(can_loop(&rr2, Some(device)));
            }

            return Ok(());
        }

        if args.cube {
            let cube_thread = thread::Builder::new()
                .name("radarcube".to_string())
                .spawn(move || block_on(udp_loop(&rr, &args.numpy)).unwrap())?;
            cube_thread.join().unwrap();
        } else {
            println!("Neither cube nor can interface selected, exiting...");
        }
    }

    Ok(())
}

fn format_cube(
    cube: &RadarCube,
    numpy: &Option<String>,
) -> Result<Array2<i16>, Box<dyn std::error::Error>> {
    if let Some(numpy) = numpy {
        // Numpy requires complex arrays to be either f32 or f64.
        let npdata = cube.data.mapv(|x| Complex32::new(x.re as f32, x.im as f32));
        write_npy(
            format!("{}/cube_{}.npy", numpy, cube.frame_counter),
            &npdata,
        )?;
    }

    // The radar cube shape is (sequence, range, rx antenna, doppler, complex).
    // When saving the cube this shape should be maintained (possibly shuffled)
    // but for display purposes we take the first sequence, first rx antenna,
    // and the real portion of the signal (note drvegrd does imaginary first).
    let data = cube.data.slice(s![1, .., 0, ..]);

    // Convert the cube to real absolute values for display as rerun cannot
    // handle complex numbers.  The absolute value is to ensure a constant
    // background.
    let data = data.mapv(|x| x.re.abs());

    trace!(
        "format_cube shape {:?} -> {:?}",
        cube.data.shape(),
        data.shape()
    );

    Ok(data)
}

/// The port5 implementation on Linux uses the recvmmsg system call to enable
/// bulk reads of UDP packets.  This is not available on other platforms.
#[cfg(target_os = "linux")]
async fn port5(tx: Sender<Vec<u8>>) -> Result<(), Box<dyn std::error::Error>> {
    use std::{os::fd::AsRawFd, time::Duration};
    use eth::SMS_PACKET_SIZE;

    const VLEN: usize = 32;
    const RETRY_TIME: Duration = Duration::from_micros(100);

    let mut mmsgs = vec![
        libc::mmsghdr {
            msg_hdr: libc::msghdr {
                msg_name: std::ptr::null_mut(),
                msg_namelen: 0,
                msg_iov: std::ptr::null_mut(),
                msg_iovlen: 0,
                msg_control: std::ptr::null_mut(),
                msg_controllen: 0,
                msg_flags: 0,
            },
            msg_len: 0,
        };
        VLEN
    ];
    let mut iovecs = vec![
        libc::iovec {
            iov_base: std::ptr::null_mut(),
            iov_len: 0,
        };
        VLEN
    ];
    let mut buf = vec![0; SMS_PACKET_SIZE * VLEN];

    common::set_process_priority();

    let sock = UdpSocket::bind("0.0.0.0:50005").await.unwrap();
    let sock = common::set_socket_bufsize(sock, 2 * 1024 * 1024);

    loop {
        for i in 0..VLEN {
            iovecs[i].iov_base = buf[i * SMS_PACKET_SIZE..].as_mut_ptr() as *mut libc::c_void;
            iovecs[i].iov_len = SMS_PACKET_SIZE;
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
                    _ => error!("port5 read error: {:?}", err),
                }
            }
            n => match tx.send(buf[..n as usize * SMS_PACKET_SIZE].to_vec()).await {
                Ok(_) => (),
                Err(e) => error!("port5 send error: {:?}", e),
            },
        }
    }
}

#[cfg(not(target_os = "linux"))]
async fn port5(tx: Sender<Vec<u8>>) -> Result<(), Box<dyn std::error::Error>> {
    let sock = UdpSocket::bind("0.0.0.0:50005").await.unwrap();
    let mut buf = [0; SMS_PACKET_SIZE];

    loop {
        match sock.recv_from(&mut buf).await {
            Ok((n, _)) => match tx.send(buf.to_vec()).await {
                Ok(_) => (),
                Err(e) => error!("port5 error: {:?}", e),
            },
            Err(e) => error!("port5 error: {:?}", e),
        }
    }
}

async fn port63(tx: Sender<Vec<u8>>) -> Result<(), Box<dyn std::error::Error>> {
    let sock = UdpSocket::bind("0.0.0.0:50063").await.unwrap();
    let mut buf = [0; SMS_PACKET_SIZE];

    loop {
        match sock.recv_from(&mut buf).await {
            Ok((_, _)) => match tx.send(buf.to_vec()).await {
                Ok(_) => (),
                Err(e) => error!("port63 error: {:?}", e),
            },
            Err(e) => error!("port63 error: {:?}", e),
        }
    }
}

async fn udp_loop(
    rr: &Option<RecordingStream>,
    numpy: &Option<String>,
) -> Result<(), Box<dyn std::error::Error>> {
    if let Some(numpy) = numpy {
        std::fs::create_dir_all(numpy)?;
    }

    let (tx5, rx) = channel(10000);
    let tx63 = tx5.clone();

    thread::Builder::new()
        .name("port5".to_owned())
        .spawn(move || block_on(port5(tx5)).unwrap())?;

    thread::Builder::new()
        .name("port63".to_owned())
        .spawn(move || block_on(port63(tx63)).unwrap())?;

    let mut reader = RadarCubeReader::default();

    let mut cube_times = [0; 180];
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

        let n_msg = msg.len() / SMS_PACKET_SIZE;

        for i in 0..n_msg {
            let start = i * SMS_PACKET_SIZE;
            let end = start + SMS_PACKET_SIZE;

            match reader.read(&msg[start..end]) {
                Ok(Some(cubemsg)) => {
                    cube_times[cube_index] = prev_time.elapsed().as_millis() as i32;
                    cube_index += 1;

                    let badcount = cubemsg.data.iter().filter(|x| x.re == 32767).count();
                    if badcount != 0 {
                        error!(
                            "encountered {} invalid elements in the radar cube",
                            badcount
                        );
                    }

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
                                droprate * 100.0,
                            );
                        } else if droprate > 0.025 {
                            warn!(
                                "cube fps={} avg={} min={} max={} droprate={:.2}%",
                                fps,
                                avg,
                                min,
                                max,
                                droprate * 100.0,
                            );
                        } else {
                            debug!(
                                "cube fps={} avg={} min={} max={} droprate={:.2}%",
                                fps,
                                avg,
                                min,
                                max,
                                droprate * 100.0,
                            );
                        }
                    }

                    prev_time = Instant::now();

                    let cube = format_cube(&cubemsg, numpy)?;

                    if let Some(rr) = rr {
                        let tensor = rerun::Tensor::try_from(cube)?;
                        rr.log("cube", &tensor)?;

                        rr.log(
                            "cube/speed_per_bin",
                            &rerun::Scalar::new(cubemsg.bin_properties.speed_per_bin as f64),
                        )?;
                        rr.log(
                            "cube/range_per_bin",
                            &rerun::Scalar::new(cubemsg.bin_properties.range_per_bin as f64),
                        )?;
                        rr.log(
                            "cube/bin_per_speed",
                            &rerun::Scalar::new(cubemsg.bin_properties.bin_per_speed as f64),
                        )?;
                    }
                }
                Ok(None) => (),
                Err(err) => {
                    dropped += 1;
                    error!("Cube Error: {:?}", err);
                }
            }
        }
    }
}

fn pcap_loop(
    rr: &Option<RecordingStream>,
    path: &String,
    numpy: &Option<String>,
) -> Result<(), Box<dyn std::error::Error>> {
    if let Some(numpy) = numpy {
        std::fs::create_dir_all(numpy)?;
    }

    let file = File::open(path)?;
    let mut reader = RadarCubeReader::default();
    let mut frame_num = 0;

    if let Some(rr) = rr {
        rr.set_time_seconds("stable_time", 0f64)
    }

    for cap in pcarp::Capture::new(file) {
        match etherparse::SlicedPacket::from_ethernet(&cap.unwrap().data) {
            Err(err) => error!("Err {:?}", err),
            Ok(pkt) => {
                if let Some(etherparse::TransportSlice::Udp(udp)) = pkt.transport {
                    if TransportHeaderSlice::from_slice(udp.payload()).is_ok() {
                        match reader.read(udp.payload()) {
                            Ok(Some(cubemsg)) => {
                                frame_num += 1;
                                let time = frame_num as f32 * 0.055;
                                let cube = format_cube(&cubemsg, numpy)?;

                                if let Some(rr) = rr {
                                    rr.set_time_seconds("stable_time", time as f64);
                                    let tensor = rerun::Tensor::try_from(cube)?;
                                    rr.log("cube", &tensor)?;
                                }
                            }
                            Ok(None) => (),
                            // Ignore StartPattern errors when reading from pcap which includes
                            // non-SMS data.
                            Err(SMSError::StartPattern(_)) => (),
                            Err(err) => error!("Cube Error: {:?}", err),
                        }
                    }
                }
            }
        }
    }

    Ok(())
}

#[cfg(feature = "can")]
async fn can_loop(rr: &Option<RecordingStream>, device: Option<String>) {
    use async_std::task::yield_now;
    use can;
    use rerun::Points3D;

    let iface = match device {
        Some(device) => device,
        None => loop {
            yield_now().await
        },
    };

    debug!("opening can interface {}", iface);
    let sock = socketcan::async_io::CanSocket::open(&iface).unwrap();

    loop {
        match can::read_message(&sock).await {
            Err(err) => println!("Error: {:?}", err),
            Ok(msg) => {
                trace!("radar can header {:?}", msg.header);

                if let Some(rr) = rr {
                    rr.log(
                        "radar",
                        &Points3D::new((0..msg.header.n_targets).map(|idx| {
                            let tgt = &msg.targets[idx];
                            transform_xyz(
                                tgt.range as f32,
                                tgt.azimuth as f32,
                                tgt.elevation as f32,
                                false,
                            )
                        }))
                        .with_radii([0.5])
                        .with_colors(
                            msg.targets
                                .map(|tgt| colormap_viridis_srgb(tgt.power as f32)),
                        ),
                    )
                    .unwrap()
                }
            }
        }
    }
}

#[cfg(feature = "can")]
fn transform_xyz(range: f32, azimuth: f32, elevation: f32, mirror: bool) -> [f32; 3] {
    use core::f32::consts::PI;

    let azi = azimuth / 180.0 * PI;
    let ele = elevation / 180.0 * PI;
    let x = range * ele.cos() * azi.cos();
    let y = range * ele.cos() * azi.sin();
    let z = range * ele.sin();
    if mirror {
        [x, -y, z]
    } else {
        [x, y, z]
    }
}

#[cfg(feature = "can")]
fn colormap_viridis_srgb(t: f32) -> [u8; 4] {
    use rerun::external::glam::Vec3A;

    const C0: Vec3A = Vec3A::new(0.277_727_34, 0.005_407_344_5, 0.334_099_8);
    const C1: Vec3A = Vec3A::new(0.105_093_04, 1.404_613_5, 1.384_590_1);
    const C2: Vec3A = Vec3A::new(-0.330_861_84, 0.214_847_56, 0.095_095_165);
    const C3: Vec3A = Vec3A::new(-4.634_230_6, -5.799_101, -19.332_441);
    const C4: Vec3A = Vec3A::new(6.228_27, 14.179_934, 56.690_55);
    const C5: Vec3A = Vec3A::new(4.776_385, -13.745_146, -65.353_035);
    const C6: Vec3A = Vec3A::new(-5.435_456, 4.645_852_6, 26.312_435);

    debug_assert!((0.0..=1.0).contains(&t));

    let c = C0 + t * (C1 + t * (C2 + t * (C3 + t * (C4 + t * (C5 + t * C6)))));

    let c = c * 255.0;
    [c.x as u8, c.y as u8, c.z as u8, 255]
}
