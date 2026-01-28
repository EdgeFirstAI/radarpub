// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.

mod args;
mod can;
mod clustering;
mod common;
mod eth;
mod net;

use args::{Args, CenterFrequency, DetectionSensitivity, FrequencySweep, RangeToggle};
use can::{read_message, read_status, write_parameter, Parameter, Status, Target};
use clap::Parser;
use clustering::Clustering;
use core::f64;
use edgefirst_schemas::{
    builtin_interfaces::{self, Time},
    edgefirst_msgs::{self, RadarInfo},
    geometry_msgs::{Quaternion, Transform, TransformStamped, Vector3},
    sensor_msgs, serde_cdr,
    std_msgs::{self, Header},
};
use eth::{RadarCube, RadarCubeReader, SMS_PACKET_SIZE};
use kanal::{AsyncReceiver, AsyncSender};
use socketcan::tokio::CanSocket;
use std::{
    collections::VecDeque,
    f32::consts::PI,
    thread::{self},
    time::Duration,
};
use tracing::{error, event, info, info_span, instrument, warn, Instrument, Level};
use tracing_subscriber::{layer::SubscriberExt as _, Layer as _, Registry};
use tracy_client::{frame_mark, plot, secondary_frame_mark};
use zenoh::{
    bytes::{Encoding, ZBytes},
    qos::{CongestionControl, Priority},
    Session,
};

#[cfg(feature = "profiling")]
#[global_allocator]
static GLOBAL: tracy_client::ProfiledAllocator<std::alloc::System> =
    tracy_client::ProfiledAllocator::new(std::alloc::System, 100);

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

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();

    args.tracy.then(tracy_client::Client::start);

    let stdout_log = tracing_subscriber::fmt::layer()
        .pretty()
        .with_filter(args.rust_log);

    let journald = match tracing_journald::layer() {
        Ok(journald) => Some(journald.with_filter(args.rust_log)),
        Err(_) => None,
    };

    let tracy = match args.tracy {
        true => Some(tracing_tracy::TracyLayer::default().with_filter(args.rust_log)),
        false => None,
    };

    let subscriber = Registry::default()
        .with(stdout_log)
        .with(journald)
        .with(tracy);
    tracing::subscriber::set_global_default(subscriber).expect("setting default subscriber failed");
    tracing_log::LogTracer::init()?;

    let session = zenoh::open(args.clone()).await.unwrap();
    let can = CanSocket::open(&args.can)?;

    let software_generation = read_status(&can, Status::SoftwareGeneration).await.unwrap();
    let major_version = read_status(&can, Status::MajorVersion).await.unwrap();
    let minor_version = read_status(&can, Status::MinorVersion).await.unwrap();
    let patch_version = read_status(&can, Status::PatchVersion).await.unwrap();
    let serial_number = read_status(&can, Status::SerialNumber).await.unwrap();
    info!("Software Generation: {}", software_generation);
    info!(
        "Version: {}.{}.{}",
        major_version, minor_version, patch_version
    );
    info!("Serial Number: {}", serial_number);

    let center_frequency = write_parameter(
        &can,
        Parameter::CenterFrequency,
        args.center_frequency as u32,
    )
    .await?;

    let frequency_sweep =
        write_parameter(&can, Parameter::FrequencySweep, args.frequency_sweep as u32).await?;

    let range_toggle =
        write_parameter(&can, Parameter::RangeToggle, args.range_toggle as u32).await?;

    let detection_sensitivity = write_parameter(
        &can,
        Parameter::DetectionSensitivity,
        args.detection_sensitivity as u32,
    )
    .await?;

    info!(
        "radar parameters: center_frequency={:?} frequency_sweep={:?} range_toggle={:?} detection_sensitivity={:?}",
        CenterFrequency::try_from(center_frequency).unwrap(),
        FrequencySweep::try_from(frequency_sweep).unwrap(),
        RangeToggle::try_from(range_toggle).unwrap(),
        DetectionSensitivity::try_from(detection_sensitivity).unwrap()
    );

    let tf_session = session.clone();
    let tf_msg = TransformStamped {
        header: Header {
            frame_id: args.base_frame_id.clone(),
            stamp: timestamp().unwrap_or(Time { sec: 0, nanosec: 0 }),
        },
        child_frame_id: args.radar_frame_id.clone(),
        transform: Transform {
            translation: Vector3 {
                x: args.radar_tf_vec[0],
                y: args.radar_tf_vec[1],
                z: args.radar_tf_vec[2],
            },
            rotation: Quaternion {
                x: args.radar_tf_quat[0],
                y: args.radar_tf_quat[1],
                z: args.radar_tf_quat[2],
                w: args.radar_tf_quat[3],
            },
        },
    };
    let tf_msg = ZBytes::from(serde_cdr::serialize(&tf_msg).unwrap());
    let tf_enc = Encoding::APPLICATION_CDR.with_schema("geometry_msgs/msg/TransformStamped");
    let tf_task = tokio::spawn(async move { tf_static(tf_session, tf_msg, tf_enc).await.unwrap() });
    std::mem::drop(tf_task);

    let info_msg = RadarInfo {
        header: Header {
            frame_id: args.base_frame_id.clone(),
            stamp: timestamp().unwrap_or(Time { sec: 0, nanosec: 0 }),
        },
        center_frequency: args.center_frequency.to_string(),
        frequency_sweep: args.frequency_sweep.to_string(),
        range_toggle: args.range_toggle.to_string(),
        detection_sensitivity: args.detection_sensitivity.to_string(),
        cube: args.cube,
    };

    let info_session = session.clone();
    let info_msg = ZBytes::from(serde_cdr::serialize(&info_msg).unwrap());
    let info_enc = Encoding::APPLICATION_CDR.with_schema("edgefirst_msgs/msg/RadarInfo");
    let tf_task =
        tokio::spawn(async move { radar_info(info_session, info_msg, info_enc).await.unwrap() });
    std::mem::drop(tf_task);

    let clustering = if args.clustering {
        let session = session.clone();
        let args = args.clone();
        let (tx, rx) = kanal::bounded_async(16);

        thread::Builder::new()
            .name("cluster".to_string())
            .spawn(move || {
                tokio::runtime::Builder::new_current_thread()
                    .enable_all()
                    .build()
                    .unwrap()
                    .block_on(clustering_task(session, args, rx))
                    .unwrap();
            })?;

        Some(tx)
    } else {
        None
    };

    if args.cube {
        let session = session.clone();
        let topic = args.cube_topic.clone();
        let frame_id = args.radar_frame_id.clone();

        thread::Builder::new()
            .name("cube".to_string())
            .spawn(move || {
                tokio::runtime::Builder::new_current_thread()
                    .enable_all()
                    .build()
                    .unwrap()
                    .block_on(cube_loop(session, topic, frame_id, args.tracy))
                    .unwrap();
            })?;
    }

    let stream_task = stream(can, session, args, clustering);
    stream_task.await.unwrap();

    Ok(())
}

async fn stream(
    can: CanSocket,
    session: Session,
    args: Args,
    clustering: Option<AsyncSender<Vec<Target>>>,
) -> Result<(), Box<dyn std::error::Error>> {
    let targets_publisher = session
        .declare_publisher(args.targets_topic.clone())
        .priority(Priority::DataHigh)
        .congestion_control(CongestionControl::Drop)
        .await
        .unwrap();

    loop {
        match read_message(&can).await {
            Err(err) => error!("canbus error: {:?}", err),
            Ok(frame) => {
                let targets = &frame.targets[..frame.header.n_targets];
                args.tracy.then(|| plot!("targets", targets.len() as f64));

                if let Some(tx) = &clustering {
                    tx.send(targets.to_vec()).await.unwrap();
                }

                let (msg, enc) = format_targets(targets, args.mirror, &args.radar_frame_id)?;

                let span = info_span!("targets_publish");
                async {
                    match targets_publisher.put(msg).encoding(enc).await {
                        Ok(_) => {}
                        Err(e) => error!("{} publish error: {:?}", args.targets_topic, e),
                    }
                }
                .instrument(span)
                .await;

                args.tracy.then(frame_mark);
            }
        }
    }
}

#[instrument(skip_all)]
fn format_targets(
    targets: &[Target],
    mirror: bool,
    frame_id: &str,
) -> Result<(ZBytes, Encoding), Box<dyn std::error::Error>> {
    let n_targets = targets.len() as u32;
    let data: Vec<_> = targets
        .iter()
        .flat_map(|target| {
            let xyz = transform_xyz(
                target.range as f32,
                target.azimuth as f32,
                target.elevation as f32,
                mirror,
            );
            [
                xyz[0],
                xyz[1],
                xyz[2],
                target.speed as f32,
                target.power as f32,
                target.rcs as f32,
            ]
        })
        .flat_map(|elem| elem.to_ne_bytes())
        .collect();

    let fields = vec![
        sensor_msgs::PointField {
            name: String::from("x"),
            offset: 0,
            datatype: PointFieldType::FLOAT32 as u8,
            count: 1,
        },
        sensor_msgs::PointField {
            name: String::from("y"),
            offset: 4,
            datatype: PointFieldType::FLOAT32 as u8,
            count: 1,
        },
        sensor_msgs::PointField {
            name: String::from("z"),
            offset: 8,
            datatype: PointFieldType::FLOAT32 as u8,
            count: 1,
        },
        sensor_msgs::PointField {
            name: String::from("speed"),
            offset: 12,
            datatype: PointFieldType::FLOAT32 as u8,
            count: 1,
        },
        sensor_msgs::PointField {
            name: String::from("power"),
            offset: 16,
            datatype: PointFieldType::FLOAT32 as u8,
            count: 1,
        },
        sensor_msgs::PointField {
            name: String::from("rcs"),
            offset: 20,
            datatype: PointFieldType::FLOAT32 as u8,
            count: 1,
        },
    ];

    let msg = sensor_msgs::PointCloud2 {
        header: std_msgs::Header {
            stamp: timestamp()?,
            frame_id: frame_id.to_string(),
        },
        height: 1,
        width: n_targets,
        fields,
        is_bigendian: false,
        point_step: 24,
        row_step: 24 * n_targets,
        data,
        is_dense: true,
    };

    let msg = ZBytes::from(serde_cdr::serialize(&msg)?);
    let enc = Encoding::APPLICATION_CDR.with_schema("sensor_msgs/msg/PointCloud2");

    Ok((msg, enc))
}

async fn clustering_task(
    session: Session,
    args: Args,
    rx: AsyncReceiver<Vec<Target>>,
) -> Result<(), Box<dyn std::error::Error>> {
    let publisher = session
        .declare_publisher(&args.clusters_topic)
        .priority(Priority::DataHigh)
        .congestion_control(CongestionControl::Drop)
        .await
        .unwrap();

    let mut window = VecDeque::<Vec<Target>>::with_capacity(args.window_size);
    let mut clustering = Clustering::new(
        args.clustering_eps,
        &args.clustering_param_scale,
        args.clustering_point_limit,
    );

    loop {
        let targets: Vec<Target> = rx.recv().await.unwrap();
        let time = timestamp()?;

        let (targets, clusters) = info_span!("clustering").in_scope(|| {
            if window.len() == args.window_size {
                window.pop_front();
            }
            window.push_back(targets);

            let targets = window.iter().flat_map(|v| v.iter()).collect::<Vec<_>>();
            let dbscantargets: Vec<_> = targets
                .iter()
                .map(|t| {
                    let [x, y, z] = transform_xyz(
                        t.range as f32,
                        t.azimuth as f32,
                        t.elevation as f32,
                        args.mirror,
                    );

                    let mut v = [x, y, z, t.speed as f32];
                    for (i, val) in v.iter_mut().enumerate() {
                        *val *= args.clustering_param_scale[i];
                    }
                    v
                })
                .collect();
            let clusters = clustering
                .cluster(dbscantargets, time.to_nanos())
                .into_iter()
                .map(|v| v[4]);

            (targets, clusters)
        });

        let (msg, enc) = format_clusters(
            time,
            &targets,
            clusters,
            args.mirror,
            args.radar_frame_id.clone(),
        )?;

        let span = info_span!("clusters_publish");
        async {
            match publisher.put(msg).encoding(enc).await {
                Ok(_) => {}
                Err(e) => error!("{} message error: {:?}", args.clusters_topic, e),
            }
        }
        .instrument(span)
        .await;

        args.tracy.then(|| secondary_frame_mark!("clustering"));
    }
}

#[instrument(skip_all)]
fn format_clusters<T: Iterator<Item = f32>>(
    time: Time,
    targets: &[&Target],
    clusters: T,
    mirror: bool,
    frame_id: String,
) -> Result<(ZBytes, Encoding), Box<dyn std::error::Error>> {
    let data: Vec<_> = targets
        .iter()
        .zip(clusters)
        .flat_map(|(target, cluster)| {
            let xyz = transform_xyz(
                target.range as f32,
                target.azimuth as f32,
                target.elevation as f32,
                mirror,
            );
            [
                xyz[0],
                xyz[1],
                xyz[2],
                target.speed as f32,
                target.power as f32,
                target.rcs as f32,
                cluster,
            ]
        })
        .flat_map(|elem| elem.to_ne_bytes())
        .collect();
    let fields = vec![
        sensor_msgs::PointField {
            name: String::from("x"),
            offset: 0,
            datatype: PointFieldType::FLOAT32 as u8,
            count: 1,
        },
        sensor_msgs::PointField {
            name: String::from("y"),
            offset: 4,
            datatype: PointFieldType::FLOAT32 as u8,
            count: 1,
        },
        sensor_msgs::PointField {
            name: String::from("z"),
            offset: 8,
            datatype: PointFieldType::FLOAT32 as u8,
            count: 1,
        },
        sensor_msgs::PointField {
            name: String::from("speed"),
            offset: 12,
            datatype: PointFieldType::FLOAT32 as u8,
            count: 1,
        },
        sensor_msgs::PointField {
            name: String::from("power"),
            offset: 16,
            datatype: PointFieldType::FLOAT32 as u8,
            count: 1,
        },
        sensor_msgs::PointField {
            name: String::from("rcs"),
            offset: 20,
            datatype: PointFieldType::FLOAT32 as u8,
            count: 1,
        },
        sensor_msgs::PointField {
            name: String::from("cluster_id"),
            offset: 24,
            datatype: PointFieldType::FLOAT32 as u8,
            count: 1,
        },
    ];

    let msg = sensor_msgs::PointCloud2 {
        header: std_msgs::Header {
            stamp: time,
            frame_id,
        },
        height: 1,
        width: targets.len() as u32,
        fields,
        is_bigendian: false,
        point_step: 28,
        row_step: 28 * targets.len() as u32,
        data,
        is_dense: true,
    };

    let msg = ZBytes::from(serde_cdr::serialize(&msg)?);
    let enc = Encoding::APPLICATION_CDR.with_schema("sensor_msgs/msg/PointCloud2");

    Ok((msg, enc))
}

async fn cube_loop(
    session: Session,
    topic: String,
    frame_id: String,
    tracy: bool,
) -> Result<(), Box<dyn std::error::Error>> {
    let cube_publisher = match session
        .declare_publisher(&topic)
        .priority(Priority::DataHigh)
        .congestion_control(CongestionControl::Drop)
        .await
    {
        Ok(v) => v,
        Err(e) => {
            error!("Failed to create publisher {}: {:?}", topic, e);
            return Err(e);
        }
    };

    let (tx5, rx) = kanal::bounded_async(128);
    let tx63 = tx5.clone();

    thread::Builder::new()
        .name("port5".to_string())
        .spawn(move || {
            tokio::runtime::Builder::new_current_thread()
                .enable_all()
                .build()
                .unwrap()
                .block_on(net::port5(tx5));
        })?;

    thread::Builder::new()
        .name("port63".to_string())
        .spawn(move || {
            tokio::runtime::Builder::new_current_thread()
                .enable_all()
                .build()
                .unwrap()
                .block_on(net::port63(tx63));
        })?;

    let mut reader = RadarCubeReader::default();

    loop {
        let msg = match rx.recv().await {
            Ok(msg) => msg,
            Err(e) => {
                error!("recv error: {:?}", e);
                continue;
            }
        };

        let n_msg = msg.len() / SMS_PACKET_SIZE;

        event!(Level::TRACE, event = "port5", n_msg = n_msg);

        for i in 0..n_msg {
            let begin = i * SMS_PACKET_SIZE;
            let end = begin + SMS_PACKET_SIZE;
            let cubemsg = reader.read(&msg[begin..end]);

            match cubemsg {
                Ok(Some(cubemsg)) => {
                    tracy.then(|| {
                        plot!("cube captured data", cubemsg.data.len() as f64);
                        plot!("cube missing data", cubemsg.missing_data as f64);
                    });

                    if cubemsg.missing_data == 0 {
                        let (msg, enc) = format_cube(cubemsg, &frame_id).unwrap();
                        let span = info_span!("cube_publish");
                        async {
                            match cube_publisher.put(msg).encoding(enc).await {
                                Ok(_) => {}
                                Err(e) => error!("publish cube error: {:?}", e),
                            }
                        }
                        .instrument(span)
                        .await;

                        tracy.then(|| secondary_frame_mark!("cube"));
                    } else {
                        warn!("dropping cube with {} missing data", cubemsg.missing_data);
                    }
                }
                Ok(None) => (),
                Err(err) => {
                    error!("capture cube error: {}", err);
                }
            }
        }
    }
}

#[instrument(skip_all, fields(shape = cubemsg.data.shape().iter().map(|s| s.to_string()).collect::<Vec<_>>().join(" ")))]
fn format_cube(
    cubemsg: RadarCube,
    frame_id: &str,
) -> Result<(ZBytes, Encoding), Box<dyn std::error::Error>> {
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
    let data = cubemsg.data.into_raw_vec_and_offset().0;
    let data2 =
        unsafe { Vec::from_raw_parts(data.as_ptr() as *mut i16, data.len() * 2, data.len() * 2) };
    std::mem::forget(data);

    let msg = edgefirst_msgs::RadarCube {
        header: std_msgs::Header {
            stamp: timestamp()?,
            frame_id: frame_id.to_string(),
        },
        timestamp: cubemsg.timestamp,
        layout,
        shape,
        scales: vec![
            1.0,
            cubemsg.bin_properties.range_per_bin,
            1.0,
            cubemsg.bin_properties.speed_per_bin,
        ],
        cube: data2,
        is_complex: true,
    };

    let msg = ZBytes::from(serde_cdr::serialize(&msg)?);
    let enc = Encoding::APPLICATION_CDR.with_schema("edgefirst_msgs/msg/RadarCube");

    Ok((msg, enc))
}

fn transform_xyz(range: f32, azimuth: f32, elevation: f32, mirror: bool) -> [f32; 3] {
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

async fn tf_static(
    session: Session,
    msg: ZBytes,
    enc: Encoding,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let topic = "rt/tf_static".to_string();
    let mut interval = tokio::time::interval(Duration::from_secs(1));

    loop {
        interval.tick().await;
        let span = info_span!("tf_static_publish");
        async { session.put(&topic, msg.clone()).encoding(enc.clone()).await }
            .instrument(span)
            .await?;
    }
}

async fn radar_info(
    session: Session,
    msg: ZBytes,
    enc: Encoding,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let topic = "rt/radar/info".to_string();
    let mut interval = tokio::time::interval(Duration::from_secs(1));

    loop {
        interval.tick().await;
        let span = info_span!("radar_info_publish");
        async { session.put(&topic, msg.clone()).encoding(enc.clone()).await }
            .instrument(span)
            .await?;
    }
}

fn timestamp() -> Result<builtin_interfaces::Time, std::io::Error> {
    let mut tp = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    let err = unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC_RAW, &mut tp) };
    if err != 0 {
        return Err(std::io::Error::last_os_error());
    }

    Ok(builtin_interfaces::Time {
        sec: tp.tv_sec as i32,
        nanosec: tp.tv_nsec as u32,
    })
}
