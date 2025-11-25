// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.

//! Zenoh Subscriber Viewer Example
//!
//! This example demonstrates how to subscribe to RadarPub's Zenoh topics and
//! visualize the processed radar data using Rerun. It connects to the EdgeFirst
//! Perception Middleware and displays:
//! - PointCloud2 messages (raw targets and clustered targets with tracking IDs)
//! - RadarCube 4D tensor data
//! - TF transform frames
//!
//! This is the recommended approach for integrating with EdgeFirst Studio and
//! the broader perception pipeline.

use clap::Parser;
use log::{debug, error, info};
use rerun::RecordingStream;
use std::net::Ipv4Addr;
use zenoh::Config;

#[derive(Parser, Debug, Clone)]
#[command(
    author,
    version,
    about = "Zenoh subscriber viewer with Rerun visualization"
)]
struct Args {
    /// Connect to remote Rerun viewer at this address
    #[arg(short, long)]
    connect: Option<Ipv4Addr>,

    /// Record Rerun data to file instead of live viewer
    #[arg(short, long)]
    record: Option<String>,

    /// Launch local Rerun viewer
    #[arg(short, long)]
    viewer: bool,

    /// Port for the Rerun viewer (remote or web server)
    #[arg(short, long)]
    port: Option<u16>,

    /// Zenoh mode: peer (default) or client
    #[arg(long, default_value = "peer")]
    zenoh_mode: String,

    /// Zenoh router address (for client mode)
    #[arg(long)]
    zenoh_router: Option<String>,

    /// Subscribe to raw targets topic
    #[arg(long, default_value = "true")]
    targets: bool,

    /// Subscribe to clustered targets topic
    #[arg(long, default_value = "true")]
    clusters: bool,

    /// Subscribe to radar cube topic
    #[arg(long)]
    cube: bool,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    let args = Args::parse();

    // Initialize Rerun recording stream
    let rr = if let Some(addr) = args.connect {
        let port = args.port.unwrap_or(9876);
        rerun::RecordingStreamBuilder::new("zenoh_viewer")
            .connect_grpc_opts(format!("rerun+http://{}:{}/proxy", addr, port))?
    } else if let Some(record) = args.record {
        rerun::RecordingStreamBuilder::new("zenoh_viewer").save(record)?
    } else if args.viewer {
        rerun::RecordingStreamBuilder::new("zenoh_viewer").spawn()?
    } else {
        return Err("No Rerun output specified (use --viewer, --connect, or --record)".into());
    };

    // Configure Zenoh
    let mut config = Config::default();

    if args.zenoh_mode == "client" {
        let router = args
            .zenoh_router
            .unwrap_or_else(|| "tcp/localhost:7447".to_string());
        config
            .insert_json5(
                "connect/endpoints",
                &serde_json::json!([router]).to_string(),
            )
            .unwrap();
    }

    info!("Opening Zenoh session in {} mode...", args.zenoh_mode);
    let session = zenoh::open(config).await.unwrap();

    // Subscribe to topics

    if args.targets {
        info!("Subscribing to /rt/radar/targets");
        let rr_clone = rr.clone();
        let sub = session
            .declare_subscriber("/rt/radar/targets")
            .await
            .unwrap();
        tokio::spawn(async move {
            loop {
                match sub.recv_async().await {
                    Ok(sample) => {
                        if let Err(e) =
                            handle_pointcloud(&rr_clone, "targets", &sample.payload().to_bytes())
                        {
                            error!("Error handling targets: {:?}", e);
                        }
                    }
                    Err(e) => {
                        error!("Subscriber error: {:?}", e);
                        break;
                    }
                }
            }
        });
    }

    if args.clusters {
        info!("Subscribing to /rt/radar/clusters");
        let rr_clone = rr.clone();
        let sub = session
            .declare_subscriber("/rt/radar/clusters")
            .await
            .unwrap();
        tokio::spawn(async move {
            loop {
                match sub.recv_async().await {
                    Ok(sample) => {
                        if let Err(e) =
                            handle_pointcloud(&rr_clone, "clusters", &sample.payload().to_bytes())
                        {
                            error!("Error handling clusters: {:?}", e);
                        }
                    }
                    Err(e) => {
                        error!("Subscriber error: {:?}", e);
                        break;
                    }
                }
            }
        });
    }

    if args.cube {
        info!("Subscribing to /rt/radar/cube");
        let rr_clone = rr.clone();
        let sub = session.declare_subscriber("/rt/radar/cube").await.unwrap();
        tokio::spawn(async move {
            loop {
                match sub.recv_async().await {
                    Ok(sample) => {
                        if let Err(e) = handle_radar_cube(&rr_clone, &sample.payload().to_bytes()) {
                            error!("Error handling radar cube: {:?}", e);
                        }
                    }
                    Err(e) => {
                        error!("Subscriber error: {:?}", e);
                        break;
                    }
                }
            }
        });
    }

    // Subscribe to TF transforms
    info!("Subscribing to /tf_static");
    let rr_clone = rr.clone();
    let _tf_sub = session.declare_subscriber("/tf_static").await.unwrap();
    tokio::spawn(async move {
        loop {
            match _tf_sub.recv_async().await {
                Ok(sample) => {
                    if let Err(e) = handle_transform(&rr_clone, &sample.payload().to_bytes()) {
                        error!("Error handling transform: {:?}", e);
                    }
                }
                Err(e) => {
                    error!("Subscriber error: {:?}", e);
                    break;
                }
            }
        }
    });

    info!("Zenoh viewer running. Press Ctrl+C to exit.");

    // Keep the application running
    tokio::signal::ctrl_c().await?;
    info!("Shutting down...");

    Ok(())
}

/// Handle PointCloud2 messages (targets or clusters)
fn handle_pointcloud(
    rr: &RecordingStream,
    entity_path: &str,
    payload: &[u8],
) -> Result<(), Box<dyn std::error::Error>> {
    // Deserialize PointCloud2 message from CDR
    let pointcloud: edgefirst_schemas::sensor_msgs::PointCloud2 = cdr::deserialize(payload)?;

    debug!(
        "Received PointCloud2: {} points, fields: {:?}",
        pointcloud.width * pointcloud.height,
        pointcloud
            .fields
            .iter()
            .map(|f| &f.name)
            .collect::<Vec<_>>()
    );

    // Parse point cloud data
    let points = parse_pointcloud2(&pointcloud)?;

    if !points.is_empty() {
        // Log to Rerun
        let positions: Vec<[f32; 3]> = points.iter().map(|p| [p.x, p.y, p.z]).collect();

        let mut point_cloud = rerun::Points3D::new(positions).with_radii([0.1]);

        // Add colors if available (for clustered data with track IDs)
        if let Some(colors) = extract_colors(&points) {
            point_cloud = point_cloud.with_colors(colors);
        }

        rr.log(format!("radar/{}", entity_path), &point_cloud)?;
    }

    Ok(())
}

/// Handle RadarCube messages
fn handle_radar_cube(
    rr: &RecordingStream,
    payload: &[u8],
) -> Result<(), Box<dyn std::error::Error>> {
    // Deserialize RadarCube message
    let cube: edgefirst_schemas::edgefirst_msgs::RadarCube = cdr::deserialize(payload)?;

    debug!(
        "Received RadarCube: timestamp {} with {} cube elements",
        cube.timestamp,
        cube.cube.len()
    );

    // Convert cube data to tensor for visualization
    let data = ndarray::Array::from_shape_vec(
        cube.shape.iter().map(|&x| x as usize).collect::<Vec<_>>(),
        cube.cube
            .iter()
            .map(|x| x.unsigned_abs())
            .collect::<Vec<_>>(),
    )?;

    let tensor = rerun::Tensor::try_from(data)?.with_dim_names(["SEQ", "RANGE", "RX", "DOPPLER"]);

    rr.log("radar/cube", &tensor)?;

    Ok(())
}

/// Handle TF transform messages
fn handle_transform(
    rr: &RecordingStream,
    payload: &[u8],
) -> Result<(), Box<dyn std::error::Error>> {
    // Deserialize TransformStamped message
    let tf: edgefirst_schemas::geometry_msgs::TransformStamped = cdr::deserialize(payload)?;

    debug!(
        "Received TF: {} -> {}",
        tf.header.frame_id, tf.child_frame_id
    );

    // Log transform to Rerun
    let translation = [
        tf.transform.translation.x as f32,
        tf.transform.translation.y as f32,
        tf.transform.translation.z as f32,
    ];

    let rotation = rerun::Quaternion::from_xyzw([
        tf.transform.rotation.x as f32,
        tf.transform.rotation.y as f32,
        tf.transform.rotation.z as f32,
        tf.transform.rotation.w as f32,
    ]);

    rr.log(
        format!("world/{}", tf.child_frame_id),
        &rerun::Transform3D::from_translation_rotation(translation, rotation),
    )?;

    Ok(())
}

/// Point structure for parsing PointCloud2
#[derive(Debug, Clone)]
struct Point {
    x: f32,
    y: f32,
    z: f32,
    intensity: Option<f32>,
    track_id: Option<u32>,
}

/// Parse PointCloud2 data into Point structures
fn parse_pointcloud2(
    msg: &edgefirst_schemas::sensor_msgs::PointCloud2,
) -> Result<Vec<Point>, Box<dyn std::error::Error>> {
    let point_step = msg.point_step as usize;
    let num_points = (msg.width * msg.height) as usize;
    let mut points = Vec::with_capacity(num_points);

    // Find field offsets
    let mut x_offset = None;
    let mut y_offset = None;
    let mut z_offset = None;
    let mut intensity_offset = None;
    let mut track_id_offset = None;

    for field in &msg.fields {
        match field.name.as_str() {
            "x" => x_offset = Some(field.offset as usize),
            "y" => y_offset = Some(field.offset as usize),
            "z" => z_offset = Some(field.offset as usize),
            "intensity" | "power" => intensity_offset = Some(field.offset as usize),
            "track_id" | "id" => track_id_offset = Some(field.offset as usize),
            _ => {}
        }
    }

    let x_off = x_offset.ok_or("Missing x field")?;
    let y_off = y_offset.ok_or("Missing y field")?;
    let z_off = z_offset.ok_or("Missing z field")?;

    for i in 0..num_points {
        let offset = i * point_step;
        let point_data = &msg.data[offset..offset + point_step];

        let x = f32::from_le_bytes(point_data[x_off..x_off + 4].try_into()?);
        let y = f32::from_le_bytes(point_data[y_off..y_off + 4].try_into()?);
        let z = f32::from_le_bytes(point_data[z_off..z_off + 4].try_into()?);

        let intensity = intensity_offset
            .map(|off| f32::from_le_bytes(point_data[off..off + 4].try_into().unwrap_or([0; 4])));

        let track_id = track_id_offset
            .map(|off| u32::from_le_bytes(point_data[off..off + 4].try_into().unwrap_or([0; 4])));

        points.push(Point {
            x,
            y,
            z,
            intensity,
            track_id,
        });
    }

    Ok(points)
}

/// Extract colors from points based on track IDs or intensity
fn extract_colors(points: &[Point]) -> Option<Vec<[u8; 4]>> {
    // If we have track IDs, use them for coloring
    if points.iter().any(|p| p.track_id.is_some()) {
        Some(
            points
                .iter()
                .map(|p| track_id_to_color(p.track_id.unwrap_or(0)))
                .collect(),
        )
    } else if points.iter().any(|p| p.intensity.is_some()) {
        // Otherwise use intensity
        Some(
            points
                .iter()
                .map(|p| intensity_to_color(p.intensity.unwrap_or(0.0)))
                .collect(),
        )
    } else {
        None
    }
}

/// Convert track ID to a distinct color
fn track_id_to_color(id: u32) -> [u8; 4] {
    // Use a simple hash to generate distinct colors for different track IDs
    let hue = ((id as f32 * 137.508) % 360.0) / 360.0; // Golden angle for better distribution
    hsv_to_rgb(hue, 0.8, 0.9)
}

/// Convert intensity to color using a colormap
fn intensity_to_color(intensity: f32) -> [u8; 4] {
    // Normalize intensity to 0-1 range (assuming typical range)
    let t = (intensity / 100.0).clamp(0.0, 1.0);
    colormap_viridis_srgb(t)
}

/// HSV to RGB conversion
fn hsv_to_rgb(h: f32, s: f32, v: f32) -> [u8; 4] {
    let c = v * s;
    let x = c * (1.0 - ((h * 6.0) % 2.0 - 1.0).abs());
    let m = v - c;

    let (r, g, b) = match (h * 6.0) as i32 {
        0 => (c, x, 0.0),
        1 => (x, c, 0.0),
        2 => (0.0, c, x),
        3 => (0.0, x, c),
        4 => (x, 0.0, c),
        _ => (c, 0.0, x),
    };

    [
        ((r + m) * 255.0) as u8,
        ((g + m) * 255.0) as u8,
        ((b + m) * 255.0) as u8,
        255,
    ]
}

/// Viridis colormap for intensity visualization
fn colormap_viridis_srgb(t: f32) -> [u8; 4] {
    use rerun::external::glam::Vec3A;

    const C0: Vec3A = Vec3A::new(0.277_727_34, 0.005_407_344_5, 0.334_099_8);
    const C1: Vec3A = Vec3A::new(0.105_093_04, 1.404_613_5, 1.384_590_1);
    const C2: Vec3A = Vec3A::new(-0.330_861_84, 0.214_847_56, 0.095_095_165);
    const C3: Vec3A = Vec3A::new(-4.634_230_6, -5.799_101, -19.332_441);
    const C4: Vec3A = Vec3A::new(6.228_27, 14.179_934, 56.690_55);
    const C5: Vec3A = Vec3A::new(4.776_385, -13.745_146, -65.353_035);
    const C6: Vec3A = Vec3A::new(-5.435_456, 4.645_852_6, 26.312_435);

    let t = t.clamp(0.0, 1.0);
    let c = C0 + t * (C1 + t * (C2 + t * (C3 + t * (C4 + t * (C5 + t * C6)))));
    let c = c * 255.0;

    [c.x as u8, c.y as u8, c.z as u8, 255]
}
