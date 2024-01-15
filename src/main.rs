use async_std::task::block_on;
use cdr::{CdrLe, Infinite};
use clap::Parser;
use deepviewrt::context::Context;
use drvegrd::Frame;
use drvegrd::{load_data, read_frame, Packet};
use futures::join;
use socketcan::{async_std::CanSocket, CanFrame, EmbeddedFrame, Id as CanId};
use std::error::Error;
use std::f32::consts::PI;
use std::str::FromStr as _;
use std::time::Instant;
use std::time::{Duration, SystemTime};
use unix_ts::Timestamp;
use zenoh::{config::Config, prelude::r#async::*};
use zenoh_ros_type::common_interfaces::sensor_msgs::PointCloud2;
use zenoh_ros_type::common_interfaces::sensor_msgs::PointField;
use zenoh_ros_type::rcl_interfaces::builtin_interfaces::Time as ROSTime;
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

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// zenoh connection mode
    #[arg(long, default_value = "peer")]
    mode: String,

    /// connect to endpoint
    #[arg(short, long)]
    endpoint: Vec<String>,

    /// ros topic
    #[arg(short, long, default_value = "rt/radar/targets")]
    topic: String,

    /// clustering by defaut its off
    #[arg(long)]
    cluster: bool,

    /// Height of the clustered points
    #[arg(long, default_value = "0")]
    proj: f32,

    /// size of the window for clustering
    #[arg(long, default_value = "6")]
    window: usize,

    /// distance threshold for clustring
    #[arg(long, default_value = "0.5")]
    distance_threshold: f32,

    /// min number of point for a cluster
    #[arg(long, default_value = "3")]
    count_threshold: usize,

    /// model threshold for clustring
    #[arg(long, default_value = "0.3")]
    model_threshold: f32,

    /// classify targets using model
    #[arg(short, long)]
    model: Option<String>,

    /// can device connected to radar
    #[arg(short, long, default_value = "can0")]
    can: String,

    /// mirror the radar data
    #[arg(long)]
    mirror: bool,
}

fn distance(point1: &[f32], point2: &[f32]) -> f32 {
    let squared_diff: f32 = point1
        .iter()
        .zip(point2)
        .map(|(x, y)| (x - y).powi(2))
        .sum();
    squared_diff.sqrt()
}

fn cluster_points(points: &Vec<Vec<f32>>, distance_threshold: f32) -> Vec<Vec<Vec<f32>>> {
    let mut clusters: Vec<Vec<Vec<f32>>> = Vec::new();

    for (_i, point) in points.iter().enumerate() {
        let mut assigned = false;

        for cluster in clusters.iter_mut() {
            for existing_point in cluster.iter() {
                let dist = distance(point, existing_point);
                if dist < distance_threshold {
                    cluster.push(point.clone());
                    assigned = true;
                    break;
                }
            }

            if assigned {
                break;
            }
        }

        if !assigned {
            clusters.push(vec![point.clone()]);
        }
    }

    clusters
}

#[async_std::main]
async fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    let mut config = Config::default();

    let mode = WhatAmI::from_str(&args.mode).unwrap();
    config.set_mode(Some(mode)).unwrap();
    config.connect.endpoints = args.endpoint.iter().map(|v| v.parse().unwrap()).collect();

    let session = zenoh::open(config).res().await.unwrap();

    let mut classifier = match &args.model {
        Some(path) => {
            let mut context = Context::new(None, 0, 0)?;
            let model = std::fs::read(path)?;
            context.load_model(model)?;
            Some(context)
        }
        None => None,
    };

    let sock = CanSocket::open(&args.can)?;
    let mut windowed_objs = Vec::new();
    let mut head = 0;

    loop {
        match read_frame(read_packet, &sock) {
            Err(err) => println!("Error: {:?}", err),
            Ok(frame) => {
                let now = Instant::now();
                let labels = match &mut classifier {
                    Some(classifier) => classify(classifier, &frame)?,
                    None => (0..frame.header.n_targets).map(|_| 0.0f32).collect(),
                };
                let classify_time = now.elapsed();

                let now = Instant::now();
                let data: Vec<_> = (0..frame.header.n_targets)
                    .flat_map(|idx| {
                        let tgt = &frame.targets[idx];
                        let xyz = transform_xyz(
                            tgt.range as f32,
                            tgt.azimuth as f32,
                            tgt.elevation as f32,
                            args.mirror,
                        );
                        [xyz[0], xyz[1], xyz[2], labels[idx]]
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
                        name: String::from("label"),
                        offset: 12,
                        datatype: PointFieldType::FLOAT32 as u8,
                        count: 1,
                    },
                ];
                let field_clone = fields.clone();

                let ts: Timestamp = SystemTime::now()
                    .duration_since(SystemTime::UNIX_EPOCH)?
                    .into();
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
                    point_step: 16,
                    row_step: 16 * frame.header.n_targets as u32,
                    data,
                    is_dense: true,
                };
                if windowed_objs.len() < args.window {
                    windowed_objs.push(
                        (0..frame.header.n_targets)
                            .map(|idx| {
                                let tgt = &frame.targets[idx];
                                let xyz = transform_xyz(
                                    tgt.range as f32,
                                    tgt.azimuth as f32,
                                    tgt.elevation as f32,
                                    args.mirror,
                                );
                                [xyz[0], xyz[1], xyz[2], labels[idx]]
                            })
                            .collect::<Vec<[f32; 4]>>(),
                    );
                } else {
                    windowed_objs[head] = (0..frame.header.n_targets)
                        .map(|idx| {
                            let tgt = &frame.targets[idx];
                            let xyz = transform_xyz(
                                tgt.range as f32,
                                tgt.azimuth as f32,
                                tgt.elevation as f32,
                                args.mirror,
                            );
                            [xyz[0], xyz[1], xyz[2], labels[idx]]
                        })
                        .collect::<Vec<[f32; 4]>>();
                    head = (head + 1) % args.window;
                }
                let mut kmeans_arr = Vec::new();
                for v in windowed_objs.iter() {
                    for p in v {
                        if p[3] > args.model_threshold {
                            kmeans_arr.push(p[0]);
                            kmeans_arr.push(p[1]);
                            kmeans_arr.push(p[2]);
                        }
                    }
                }

                if args.cluster {
                    let mut points: Vec<Vec<f32>> = Vec::new();
                    for i in (0..kmeans_arr.len()).step_by(3) {
                        let point = kmeans_arr[i..(i + 3)].to_vec();
                        points.push(point);
                    }
                    let clusters: Vec<_> = cluster_points(&points, args.distance_threshold)
                        .into_iter()
                        .filter(|e| e.len() > args.count_threshold)
                        .collect();

                    let avg_distances: Vec<_> = clusters
                        .iter()
                        .flat_map(|cluster| {
                            let mut x = 0.0;
                            let mut y = 0.0;
                            for p in cluster {
                                x += p[0];
                                y += p[1];
                            }
                            let count: f32 = cluster.len() as f32;
                            [x / count, y / count, args.proj, 1.0]
                        })
                        .collect();
                    let data: Vec<_> = avg_distances
                        .into_iter()
                        .flat_map(|elem| elem.to_ne_bytes())
                        .collect();

                    let ts: Timestamp = SystemTime::now()
                        .duration_since(SystemTime::UNIX_EPOCH)?
                        .into();
                    let fields = field_clone;
                    let cluster_msg = PointCloud2 {
                        header: zenoh_ros_type::std_msgs::Header {
                            stamp: ROSTime {
                                sec: ts.seconds() as i32,
                                nanosec: ts.subsec(9),
                            },
                            frame_id: "".to_string(),
                        },
                        height: 1,
                        width: clusters.len() as u32,
                        fields,
                        is_bigendian: false,
                        point_step: 16,
                        row_step: 16 * clusters.len() as u32,
                        data,
                        is_dense: true,
                    };
                    let cluster_encoded = cdr::serialize::<_, _, CdrLe>(&cluster_msg, Infinite)?;
                    let cluster_serialize_time = now.elapsed();
                    session
                        .put(&args.topic, cluster_encoded)
                        .res()
                        .await
                        .unwrap();
                    send_radar_timing(&session, &args, &classify_time, &cluster_serialize_time)
                        .await
                        .unwrap();
                } else {
                    let encoded = cdr::serialize::<_, _, CdrLe>(&msg, Infinite)?;
                    let serialize_time = now.elapsed();
                    session.put(&args.topic, encoded).res().await.unwrap();
                    send_radar_timing(&session, &args, &classify_time, &serialize_time)
                        .await
                        .unwrap();
                }
            }
        }
    }
}

fn classify(
    classifier: &mut deepviewrt::context::Context,
    frame: &Frame,
) -> Result<Vec<f32>, Box<dyn Error>> {
    match classifier.input_mut(0)?.maprw_f32() {
        Err(err) => panic!("failed to map input: {:?}", err),
        Ok(mut map) => {
            for idx in 0..frame.header.n_targets {
                let tgt = &frame.targets[idx];
                map[idx * 4] = tgt.speed as f32;
                map[idx * 4 + 1] = tgt.power as f32;
                map[idx * 4 + 2] = tgt.rcs as f32;
                map[idx * 4 + 3] = tgt.noise as f32;
            }
        }
    }

    classifier.run()?;

    let mut labels = vec![0.0f32; frame.header.n_targets];
    match classifier.output(0)?.mapro_f32() {
        Err(err) => panic!("failed to map output: {:?}", err),
        Ok(map) => {
            for idx in 0..frame.header.n_targets {
                labels[idx] = map[idx];
            }
        }
    }

    Ok(labels)
}

async fn send_radar_timing(
    session: &Session,
    args: &Args,
    classify_time: &Duration,
    serialize_time: &Duration,
) -> Result<(), Box<dyn Error>> {
    let class_str = args.topic.clone() + "/classify_time";
    let serial_str = args.topic.clone() + "/serialize_time";
    let send_class = send_timing(&session, &class_str, &classify_time);
    let send_serialize = send_timing(&session, &serial_str, &serialize_time);
    let _ = join!(send_class, send_serialize);
    Ok(())
}

fn read_packet(can: &CanSocket) -> Result<Packet, drvegrd::Error> {
    match block_on(can.read_frame()) {
        Ok(CanFrame::Data(frame)) => {
            let id = match frame.id() {
                CanId::Standard(id) => id.as_raw() as u32,
                CanId::Extended(id) => id.as_raw(),
            };
            Ok(Packet {
                id,
                data: load_data(frame.data()),
            })
        }
        Ok(CanFrame::Remote(frame)) => panic!("Unexpected remote frame: {:?}", frame),
        Ok(CanFrame::Error(frame)) => panic!("Unexpected error frame: {:?}", frame),
        Err(err) => Err(drvegrd::Error::Io(err)),
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

async fn send_timing(
    session: &Session,
    topic: &String,
    time: &Duration,
) -> Result<(), Box<dyn Error>> {
    let msg = zenoh_ros_type::builtin_interfaces::Duration {
        sec: time.as_secs() as _,
        nanosec: time.subsec_nanos() as _,
    };
    let encoded = cdr::serialize::<_, _, CdrLe>(&msg, Infinite)?;
    session.put(topic, encoded).res().await.unwrap();
    Ok(())
}
