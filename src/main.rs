use async_std::{net::UdpSocket, task::block_on};
use cdr::{CdrLe, Infinite};
use clap::{Parser, ValueEnum};
use drvegrd::{
    can::{read_message, write_parameter, Parameter, Target},
    eth::RadarCubeReader,
};
use edgefirst_schemas::{
    builtin_interfaces::{self, Time},
    edgefirst_msgs::{self, RadarInfo},
    geometry_msgs::{Quaternion, Transform, TransformStamped, Vector3},
    sensor_msgs,
    std_msgs::{self, Header},
};
use kanal::{bounded_async as channel, AsyncReceiver, AsyncSender as Sender};
use log::{debug, error, info, trace, warn};
#[cfg(feature = "rerun")]
use rerun::{external::re_sdk_comms::DEFAULT_SERVER_PORT, Points3D};
use socketcan::async_std::CanSocket;
use std::{
    collections::VecDeque,
    f32::consts::PI,
    fmt, io,
    str::FromStr as _,
    sync::Arc,
    thread::{self, sleep},
    time::{Duration, Instant},
};
use zenoh::{
    config::Config,
    prelude::{r#async::*, sync::SyncResolve},
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

#[derive(Debug)]
enum Error {
    Io(io::Error),
    InvalidCenterFrequency(u32),
    InvalidFrequencySweep(u32),
    InvalidRangeToggle(u32),
    InvalidDetectionSensitivity(u32),
}

impl std::error::Error for Error {}

impl From<io::Error> for Error {
    fn from(err: io::Error) -> Error {
        Error::Io(err)
    }
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter) -> std::fmt::Result {
        match self {
            Error::Io(err) => write!(f, "io error: {}", err),
            Error::InvalidCenterFrequency(value) => {
                write!(f, "invalid center frequency: {}", value)
            }
            Error::InvalidFrequencySweep(value) => write!(f, "invalid frequency sweep: {}", value),
            Error::InvalidRangeToggle(value) => write!(f, "invalid range toggle: {}", value),
            Error::InvalidDetectionSensitivity(value) => {
                write!(f, "invalid detection sensitivity: {}", value)
            }
        }
    }
}

/// The center frequency for the radar.
/// Note: ultra-short range is only supported with the low center frequency.
#[derive(Copy, Clone, Debug, ValueEnum)]
enum CenterFrequency {
    Low = 0,
    Medium = 1,
    High = 2,
}

impl TryFrom<u32> for CenterFrequency {
    type Error = Error;

    fn try_from(value: u32) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(CenterFrequency::Low),
            1 => Ok(CenterFrequency::Medium),
            2 => Ok(CenterFrequency::High),
            _ => Err(Error::InvalidCenterFrequency(value)),
        }
    }
}

impl fmt::Display for CenterFrequency {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            CenterFrequency::Low => write!(f, "low"),
            CenterFrequency::Medium => write!(f, "medium"),
            CenterFrequency::High => write!(f, "high"),
        }
    }
}

/// The frequency sweep which controls the range of the radar.
/// Note: ultra-short range is only supported with the low center frequency.
#[derive(Copy, Clone, Debug, ValueEnum)]
enum FrequencySweep {
    Long = 0,
    Medium = 1,
    Short = 2,
    UltraShort = 3,
}

impl TryFrom<u32> for FrequencySweep {
    type Error = Error;

    fn try_from(value: u32) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(FrequencySweep::Long),
            1 => Ok(FrequencySweep::Medium),
            2 => Ok(FrequencySweep::Short),
            3 => Ok(FrequencySweep::UltraShort),
            _ => Err(Error::InvalidFrequencySweep(value)),
        }
    }
}

impl fmt::Display for FrequencySweep {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            FrequencySweep::Long => write!(f, "long"),
            FrequencySweep::Medium => write!(f, "medium"),
            FrequencySweep::Short => write!(f, "short"),
            FrequencySweep::UltraShort => write!(f, "ultra-short"),
        }
    }
}

/// The range toggle mode allows the radar to alternate between various
/// frequency sweeps at runtime.
#[derive(Copy, Clone, Debug, ValueEnum)]
enum RangeToggle {
    Off = 0,
    ShortMedium = 1,
    ShortLong = 2,
    MediumLong = 3,
    LongUltraShort = 4,
    MediumUltraShort = 5,
    ShortUltraShort = 6,
}

impl TryFrom<u32> for RangeToggle {
    type Error = Error;

    fn try_from(value: u32) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(RangeToggle::Off),
            1 => Ok(RangeToggle::ShortMedium),
            2 => Ok(RangeToggle::ShortLong),
            3 => Ok(RangeToggle::MediumLong),
            4 => Ok(RangeToggle::LongUltraShort),
            5 => Ok(RangeToggle::MediumUltraShort),
            6 => Ok(RangeToggle::ShortUltraShort),
            _ => Err(Error::InvalidRangeToggle(value)),
        }
    }
}

impl fmt::Display for RangeToggle {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            RangeToggle::Off => write!(f, "off"),
            RangeToggle::ShortMedium => write!(f, "short-medium"),
            RangeToggle::ShortLong => write!(f, "short-long"),
            RangeToggle::MediumLong => write!(f, "medium-long"),
            RangeToggle::LongUltraShort => write!(f, "long-ultra-short"),
            RangeToggle::MediumUltraShort => write!(f, "medium-ultra-short"),
            RangeToggle::ShortUltraShort => write!(f, "short-ultra-short"),
        }
    }
}

/// The detection sensitivity controls the radar's ability to detect targets.
#[derive(Copy, Clone, Debug, ValueEnum)]
enum DetectionSensitivity {
    Low = 0,
    Medium = 1,
    High = 2,
}

impl TryFrom<u32> for DetectionSensitivity {
    type Error = Error;

    fn try_from(value: u32) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(DetectionSensitivity::Low),
            1 => Ok(DetectionSensitivity::Medium),
            2 => Ok(DetectionSensitivity::High),
            _ => Err(Error::InvalidDetectionSensitivity(value)),
        }
    }
}

impl fmt::Display for DetectionSensitivity {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            DetectionSensitivity::Low => write!(f, "low"),
            DetectionSensitivity::Medium => write!(f, "medium"),
            DetectionSensitivity::High => write!(f, "high"),
        }
    }
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

    /// radar targets topic name
    #[arg(long, default_value = "rt/radar/targets")]
    targets_topic: String,

    /// radar clusters topic name
    #[arg(long, default_value = "rt/radar/clusters")]
    clusters_topic: String,

    /// radar data cube topic name
    #[arg(long, default_value = "rt/radar/cube")]
    cube_topic: String,

    /// can device connected to radar
    #[arg(long, default_value = "can0")]
    can: String,

    /// mirror the radar data
    #[arg(long, env)]
    mirror: bool,

    /// radar frame transform vector from base_link
    #[arg(
        long,
        env,
        default_value = "0 0 0",
        value_delimiter = ' ',
        num_args = 3
    )]
    radar_tf_vec: Vec<f64>,

    /// radar frame transform quaternion from base_link
    #[arg(
        long,
        env,
        default_value = "0 0 0 1",
        value_delimiter = ' ',
        num_args = 4
    )]
    radar_tf_quat: Vec<f64>,

    /// The name of the base frame
    #[arg(long, env, default_value = "base_link")]
    base_frame_id: String,

    /// The name of the radar frame
    #[arg(long, env, default_value = "radar")]
    radar_frame_id: String,

    /// The center frequency for the radar.
    #[arg(long, env, default_value = "medium")]
    center_frequency: CenterFrequency,

    /// The frequency sweep which controls the range of the radar.
    #[arg(long, env, default_value = "short")]
    frequency_sweep: FrequencySweep,

    /// The range toggle mode allows the radar to alternate between various
    /// frequencies.
    #[arg(long, env, default_value = "off")]
    range_toggle: RangeToggle,

    /// The detection sensitivity controls the radar's ability to detect
    /// targets.
    #[arg(long, env, default_value = "medium")]
    detection_sensitivity: DetectionSensitivity,

    /// Enable streaming the low-level radar data cube on the cube_topic.
    #[arg(long, env, default_value = "false")]
    cube: bool,

    /// Enable radar target clustering task.
    #[arg(long, env, default_value = "false")]
    clustering: bool,

    /// Clustering window size in frames (one frame is 55ms).
    #[arg(long, env, default_value = "1")]
    window_size: usize,

    // Clustering DBSCAN distance limit (euclidean distance)
    #[arg(long, env, default_value = "1")]
    clustering_eps: f64,

    // Clustering DBSCAN parameter scaling. Parameter order is x, y, z, speed. Set the appropriate
    // axis to 0 to ignore that axis
    #[arg(
        long,
        env,
        default_value = "1 1 0 0",
        value_delimiter = ' ',
        num_args = 4
    )]
    clustering_param_scale: Vec<f32>,

    // Clustering DBSCAN point limit. Minimum 3
    #[arg(long, env, default_value = "5")]
    clustering_point_limit: usize,

    /// connect to remote rerun viewer at this address
    #[cfg(feature = "rerun")]
    #[arg(short, long)]
    connect: Option<std::net::Ipv4Addr>,

    /// record rerun data to file instead of live viewer
    #[cfg(feature = "rerun")]
    #[arg(short, long)]
    record: Option<String>,

    /// use this port for the rerun viewer (remote or web server)
    #[cfg(feature = "rerun")]
    #[arg(short, long)]
    port: Option<u16>,
}

struct Context {
    session: Arc<Session>,
    topic: String,
    radar_frame_id: String,

    #[cfg(feature = "rerun")]
    rr: Option<rerun::RecordingStream>,
}

#[async_std::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();
    env_logger::init();

    #[cfg(feature = "rerun")]
    let rr = if let Some(addr) = args.connect {
        let port = args.port.unwrap_or(DEFAULT_SERVER_PORT);
        let remote = std::net::SocketAddr::new(addr.into(), port);
        Some(
            rerun::RecordingStreamBuilder::new("radarpub")
                .connect_opts(remote, rerun::default_flush_timeout())?,
        )
    } else if let Some(record) = args.record {
        Some(rerun::RecordingStreamBuilder::new("radarpub").save(record)?)
    } else {
        None
    };

    let mut config = Config::default();
    let mode = WhatAmI::from_str(&args.mode).unwrap();
    config.set_mode(Some(mode)).unwrap();
    config.connect.endpoints = args.endpoint.iter().map(|v| v.parse().unwrap()).collect();
    let _ = config.scouting.multicast.set_enabled(Some(false));
    let session = zenoh::open(config).res_async().await.unwrap().into_arc();
    debug!("Opened Zenoh session");

    let sock = CanSocket::open(&args.can)?;

    let center_frequency = write_parameter(
        &sock,
        Parameter::CenterFrequency,
        args.center_frequency as u32,
    )
    .await?;

    let frequency_sweep = write_parameter(
        &sock,
        Parameter::FrequencySweep,
        args.frequency_sweep as u32,
    )
    .await?;

    let range_toggle =
        write_parameter(&sock, Parameter::RangeToggle, args.range_toggle as u32).await?;

    let detection_sensitivity = write_parameter(
        &sock,
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

    let ctx = Context {
        session: session.clone(),
        topic: args.cube_topic.clone(),
        radar_frame_id: args.radar_frame_id.clone(),
        #[cfg(feature = "rerun")]
        rr: rr.clone(),
    };

    spawn_tf_static(session.clone(), &args).await.unwrap();
    spawn_radar_info(session.clone(), &args).await.unwrap();

    let clustering = if args.clustering {
        let session = session.clone();
        let args = args.clone();
        let (tx, rx) = channel(10000);

        thread::Builder::new()
            .name("clustering".to_string())
            .spawn(move || block_on(clustering_task(session, &args, rx)).unwrap())?;

        Some(tx)
    } else {
        None
    };

    if args.cube {
        thread::Builder::new()
            .name("radarcube".to_string())
            .spawn(move || {
                block_on(udp_loop(ctx)).unwrap();
            })?;
    }

    loop {
        match read_message(&sock).await {
            Err(err) => println!("Error: {:?}", err),
            Ok(frame) => {
                trace!("Received radar frame: {:?}", frame);

                if frame.header.n_targets == 0 {
                    continue;
                }

                let min = frame.targets[..frame.header.n_targets]
                    .iter()
                    .map(|tgt| tgt.power)
                    .reduce(f64::min)
                    .unwrap();
                let max = frame.targets[..frame.header.n_targets]
                    .iter()
                    .map(|tgt| tgt.power)
                    .reduce(f64::max)
                    .unwrap();
                let avg = frame.targets[..frame.header.n_targets]
                    .iter()
                    .map(|tgt| tgt.power)
                    .sum::<f64>()
                    / frame.header.n_targets as f64;

                debug!(
                    "Processing radar frame with {} targets - power: min={} max={} avg={}",
                    frame.header.n_targets, min, max, avg
                );

                #[cfg(feature = "rerun")]
                if let Some(rr) = &rr {
                    rr.log(
                        "radar",
                        &Points3D::new((0..frame.header.n_targets).map(|idx| {
                            let tgt = &frame.targets[idx];
                            transform_xyz(
                                tgt.range as f32,
                                tgt.azimuth as f32,
                                tgt.elevation as f32,
                                false,
                            )
                        }))
                        .with_radii([0.5])
                        .with_colors(
                            frame.targets[..frame.header.n_targets].iter().map(|tgt| {
                                let c = tgt.power - min;
                                let c = c / (max - min + 0.0001);
                                colormap_viridis_srgb(c as f32)
                            }),
                        ),
                    )
                    .unwrap();
                }

                let targets = frame.targets[..frame.header.n_targets].to_vec();

                let data: Vec<_> = targets
                    .iter()
                    .flat_map(|target| {
                        let xyz = transform_xyz(
                            target.range as f32,
                            target.azimuth as f32,
                            target.elevation as f32,
                            args.mirror,
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

                if let Some(tx) = &clustering {
                    tx.send(targets).await.unwrap();
                }

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
                        frame_id: args.radar_frame_id.clone(),
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
                    .res_async()
                    .await
                {
                    Ok(_) => trace!("{} message sent", args.targets_topic),
                    Err(e) => error!("{} message error: {:?}", args.targets_topic, e),
                }
            }
        }
    }
}

async fn spawn_tf_static(
    session: Arc<zenoh::Session>,
    args: &Args,
) -> Result<(), Box<dyn std::error::Error>> {
    let publisher = match session
        .declare_publisher("rt/tf_static".to_string())
        .priority(Priority::Background)
        .congestion_control(CongestionControl::Drop)
        .res_async()
        .await
    {
        Ok(v) => v,
        Err(e) => {
            error!("Failed to create publisher rt/tf_static: {:?}", e);
            return Err(e);
        }
    };

    let msg = TransformStamped {
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

    let msg =
        Value::from(cdr::serialize::<_, _, CdrLe>(&msg, Infinite)?).encoding(Encoding::WithSuffix(
            KnownEncoding::AppOctetStream,
            "geometry_msgs/msg/TransformStamped".into(),
        ));

    thread::spawn(move || {
        let interval = Duration::from_secs(1);
        let mut target_time = Instant::now() + interval;

        loop {
            publisher.put(msg.clone()).res_sync().unwrap();
            trace!("radarpub publishing rt/tf_static");
            sleep(target_time.duration_since(Instant::now()));
            target_time += interval
        }
    });

    Ok(())
}

async fn spawn_radar_info(
    session: Arc<zenoh::Session>,
    args: &Args,
) -> Result<(), Box<dyn std::error::Error>> {
    let publisher = match session
        .declare_publisher("rt/radar/info".to_string())
        .priority(Priority::Background)
        .congestion_control(CongestionControl::Drop)
        .res_async()
        .await
    {
        Ok(v) => v,
        Err(e) => {
            error!("Failed to create publisher rt/radar/info: {:?}", e);
            return Err(e);
        }
    };

    let msg = RadarInfo {
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

    let msg =
        Value::from(cdr::serialize::<_, _, CdrLe>(&msg, Infinite)?).encoding(Encoding::WithSuffix(
            KnownEncoding::AppOctetStream,
            "edgefirst_msgs/msg/RadarInfo".into(),
        ));

    thread::spawn(move || {
        let interval = Duration::from_secs(1);
        let mut target_time = Instant::now() + interval;

        loop {
            publisher.put(msg.clone()).res_sync().unwrap();
            trace!("radarpub publishing rt/radar/info");
            sleep(target_time.duration_since(Instant::now()));
            target_time += interval
        }
    });

    Ok(())
}

use dbscan::{Classification, Model};
async fn clustering_task(
    session: Arc<zenoh::Session>,
    args: &Args,
    rx: AsyncReceiver<Vec<Target>>,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut window = VecDeque::<Vec<Target>>::with_capacity(args.window_size);
    loop {
        if window.len() == args.window_size {
            window.pop_front();
        }

        let targets: Vec<Target> = rx.recv().await.unwrap();
        window.push_back(targets);
        let targets: Vec<&Target> = window.iter().flat_map(|v| v.iter()).collect::<Vec<_>>();
        let dbscantargets: Vec<Vec<f32>> = targets
            .iter()
            .map(|t| {
                let mut xyz = transform_xyz(
                    t.range as f32,
                    t.azimuth as f32,
                    t.elevation as f32,
                    args.mirror,
                );
                for i in 0..3 {
                    xyz[i] *= args.clustering_param_scale[i];
                }
                let mut v = Vec::from(xyz);
                v.push(t.speed as f32 * args.clustering_param_scale[3]);
                v
            })
            .collect();
        let dbscan_clusters =
            Model::new(args.clustering_eps, args.clustering_point_limit).run(&dbscantargets);

        let data: Vec<_> = targets
            .iter()
            .zip(dbscan_clusters.iter())
            .flat_map(|(target, cluster)| {
                let xyz = transform_xyz(
                    target.range as f32,
                    target.azimuth as f32,
                    target.elevation as f32,
                    args.mirror,
                );
                let cluster_id = match cluster {
                    Classification::Core(i) => i + 1,
                    Classification::Edge(i) => i + 1,
                    Classification::Noise => 0,
                };
                [
                    xyz[0],
                    xyz[1],
                    xyz[2],
                    target.speed as f32,
                    target.power as f32,
                    target.rcs as f32,
                    cluster_id as f32,
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
                stamp: timestamp()?,
                frame_id: args.radar_frame_id.clone(),
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
        let encoded = cdr::serialize::<_, _, CdrLe>(&msg, Infinite)?;

        match session
            .put(&args.clusters_topic, encoded)
            .encoding(Encoding::WithSuffix(
                KnownEncoding::AppOctetStream,
                "sensor_msgs/msg/PointCloud2".into(),
            ))
            .res_async()
            .await
        {
            Ok(_) => trace!("{} message sent", args.clusters_topic),
            Err(e) => error!("{} message error: {:?}", args.clusters_topic, e),
        }
    }
}

async fn udp_loop(ctx: Context) -> Result<(), Box<dyn std::error::Error>> {
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

        match reader.read(&msg) {
            Ok(Some(cubemsg)) => {
                trace!("radar data cube with shape {:?}", cubemsg.data.shape());

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

                #[cfg(feature = "rerun")]
                match &ctx.rr {
                    Some(rr) => {
                        // The radar cube shape is (sequence, range, rx antenna, doppler, complex).
                        // When saving the cube this shape should be maintained (possibly shuffled)
                        // but for display purposes we take the first sequence, first rx antenna,
                        // and the real portion of the signal (note drvegrd does imaginary first).
                        let data = cubemsg.data.slice(ndarray::s![0, .., 0, ..]);

                        // Convert the cube to real absolute values for display as rerun cannot
                        // handle complex numbers.  The absolute value is to ensure a constant
                        // background.
                        let data = data.mapv(|x| x.re.abs());
                        let tensor = rerun::Tensor::try_from(data)?;

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
                    None => (),
                }

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
                    header: std_msgs::Header {
                        stamp: timestamp()?,
                        frame_id: ctx.radar_frame_id.clone(),
                    },
                    timestamp: cubemsg.timestamp,
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

                match ctx
                    .session
                    .put(&ctx.topic, encoded)
                    .encoding(Encoding::WithSuffix(
                        KnownEncoding::AppOctetStream,
                        "edgefirst_msgs/msg/RadarCube".into(),
                    ))
                    .res_async()
                    .await
                {
                    Ok(_) => trace!("RadarCube Message Sent"),
                    Err(e) => error!("RadarCube Message Error: {:?}", e),
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

/// The port5 implementation on Linux uses the recvmmsg system call to enable
/// bulk reads of UDP packets.  This is not available on other platforms.
#[cfg(target_os = "linux")]
async fn port5(tx: Sender<Vec<u8>>) -> Result<(), Box<dyn std::error::Error>> {
    use libc::{sched_param, SCHED_FIFO};
    use std::{mem, os::fd::AsRawFd, time::Duration};

    const VLEN: usize = 50;
    const BUFLEN: usize = 1500;
    const RETRY_TIME: Duration = Duration::from_micros(250);

    let mut vlen_history = [0; 1000];
    let mut vlen_index = 0;

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
    let mut bufs = vec![[0; BUFLEN]; VLEN];

    let mut param = sched_param { sched_priority: 10 };
    let pid = unsafe { libc::pthread_self() };
    let err =
        unsafe { libc::pthread_setschedparam(pid, SCHED_FIFO, &mut param as *mut sched_param) };
    if err != 0 {
        let err = std::io::Error::last_os_error();
        warn!("unable to set port5 real-time fifo scheduler: {}", err);
    }

    let sock = UdpSocket::bind("0.0.0.0:50005").await.unwrap();

    let maxbuf: libc::c_int = 2 * 1024 * 1024;
    let err = unsafe {
        libc::setsockopt(
            sock.as_raw_fd(),
            libc::SOL_SOCKET,
            libc::SO_RCVBUFFORCE,
            &maxbuf as *const _ as *const libc::c_void,
            mem::size_of_val(&maxbuf) as libc::socklen_t,
        )
    };
    if err != 0 {
        let err = std::io::Error::last_os_error();
        warn!(
            "unable to set port5 socket buffer size to {}: {}",
            maxbuf, err
        );
    }

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

                for i in 0..n as usize {
                    let len = mmsgs[i].msg_len as usize;
                    match tx.send(bufs[i][..len].to_vec()).await {
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

#[cfg(feature = "rerun")]
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
