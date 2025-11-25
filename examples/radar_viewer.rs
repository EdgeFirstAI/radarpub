// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.

//! Direct Radar Viewer Example
//!
//! This example demonstrates how to connect directly to a Smart Micro DRVEGRD
//! radar sensor and visualize the data using Rerun. It supports:
//! - Live CAN interface reading for target data
//! - Live UDP interface reading for radar cube data
//! - PCAP file replay for offline analysis
//! - Numpy export for post-processing

use clap::Parser;
use log::{debug, error, trace};
use ndarray::{s, Array2};
use ndarray_npy::write_npy;
use num::complex::Complex32;
use rerun::RecordingStream;
use std::{fs::File, net::Ipv4Addr, thread};

// Import from radarpub library
use radarpub::{
    eth::{RadarCube, RadarCubeReader, SMSError, TransportHeaderSlice, SMS_PACKET_SIZE},
    net,
};

#[cfg(feature = "can")]
use radarpub::can;

#[derive(Parser, Debug, Clone)]
#[command(
    author,
    version,
    about = "Direct radar sensor viewer with Rerun visualization"
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

    /// Save Numpy files to this directory, one per frame
    #[arg(short, long)]
    numpy: Option<String>,

    /// Read from a PCAP file instead of a live interface
    #[arg()]
    pcap: Option<String>,

    /// Enable radar data cube streaming (UDP)
    #[arg(long)]
    cube: bool,

    /// CAN interface for target data (e.g., can0, vcan0)
    #[cfg(feature = "can")]
    #[arg(long)]
    device: Option<String>,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    let args = Args::parse();

    // Initialize Rerun recording stream
    let rr = if let Some(addr) = args.connect {
        let port = args.port.unwrap_or(9876);
        Some(
            rerun::RecordingStreamBuilder::new("radar_viewer")
                .connect_grpc_opts(format!("rerun+http://{}:{}/proxy", addr, port))?,
        )
    } else if let Some(record) = args.record {
        Some(rerun::RecordingStreamBuilder::new("radar_viewer").save(record)?)
    } else if args.viewer {
        Some(rerun::RecordingStreamBuilder::new("radar_viewer").spawn()?)
    } else {
        None
    };

    // Handle different data sources
    if let Some(pcap) = args.pcap {
        // Offline PCAP replay
        pcap_loop(&rr, &pcap, &args.numpy)?;
    } else {
        // Live radar data
        #[cfg(feature = "can")]
        if let Some(device) = args.device {
            let rr2 = rr.clone();

            if args.cube {
                // Cube data only
                let cube_thread =
                    thread::Builder::new()
                        .name("cube".to_string())
                        .spawn(move || {
                            tokio::runtime::Builder::new_current_thread()
                                .enable_all()
                                .build()
                                .unwrap()
                                .block_on(udp_loop(&rr, &args.numpy))
                                .unwrap();
                        })?;
                cube_thread.join().unwrap();
            } else {
                // CAN target data only
                let can_thread =
                    thread::Builder::new()
                        .name("can".to_string())
                        .spawn(move || {
                            tokio::runtime::Builder::new_current_thread()
                                .enable_all()
                                .build()
                                .unwrap()
                                .block_on(can_loop(&rr2, Some(device)));
                        })?;
                can_thread.join().unwrap();
            }

            return Ok(());
        }

        if args.cube {
            // Cube data without CAN
            let cube_thread = thread::Builder::new()
                .name("cube".to_string())
                .spawn(move || {
                    tokio::runtime::Builder::new_current_thread()
                        .enable_all()
                        .build()
                        .unwrap()
                        .block_on(udp_loop(&rr, &args.numpy))
                        .unwrap();
                })?;
            cube_thread.join().unwrap();
        } else {
            println!("Neither cube nor CAN interface selected, exiting...");
            println!("Use --cube for radar cube data or --device <can_interface> for target data");
        }
    }

    Ok(())
}

/// Format radar cube for visualization
///
/// Extracts a 2D slice from the 4D radar cube for display and optionally saves
/// to Numpy format
fn format_cube(
    cube: &RadarCube,
    numpy: &Option<String>,
) -> Result<Array2<i16>, Box<dyn std::error::Error>> {
    if let Some(numpy) = numpy {
        // Numpy requires complex arrays to be either f32 or f64
        let npdata = cube.data.mapv(|x| Complex32::new(x.re as f32, x.im as f32));
        write_npy(
            format!("{}/cube_{}.npy", numpy, cube.frame_counter),
            &npdata,
        )?;
    }

    // The radar cube shape is (sequence, range, rx antenna, doppler, complex).
    // For display purposes, take the first sequence, first rx antenna, and the real
    // portion
    let data = cube.data.slice(s![1, .., 0, ..]);

    // Convert to absolute values (Rerun cannot handle complex numbers)
    let data = data.mapv(|x| x.re.abs());

    trace!(
        "format_cube shape {:?} -> {:?}",
        cube.data.shape(),
        data.shape()
    );

    Ok(data)
}

/// Main loop for live UDP radar cube data
async fn udp_loop(
    rr: &Option<RecordingStream>,
    numpy: &Option<String>,
) -> Result<(), Box<dyn std::error::Error>> {
    if let Some(numpy) = numpy {
        std::fs::create_dir_all(numpy)?;
    }

    let (tx5, rx) = kanal::bounded_async(128);
    let tx63 = tx5.clone();

    // Spawn UDP receiver threads for ports 5 and 63
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

        for i in 0..n_msg {
            let start = i * SMS_PACKET_SIZE;
            let end = start + SMS_PACKET_SIZE;

            match reader.read(&msg[start..end]) {
                Ok(Some(cubemsg)) => {
                    let badcount = cubemsg
                        .data
                        .iter()
                        .filter(|x| x.re == 32767 || x.im == 32767)
                        .count();
                    let badrate = badcount as f64 / cubemsg.data.len() as f64;
                    let skiprate = cubemsg.packets_skipped as f64
                        / (cubemsg.packets_skipped + cubemsg.packets_captured) as f64;

                    if badcount != 0 {
                        error!(
                            "encountered {} invalid elements in the radar cube",
                            badcount
                        );
                    }

                    if cubemsg.packets_skipped != 0 {
                        error!("dropped {} packets", cubemsg.packets_skipped);
                    }

                    let cube = format_cube(&cubemsg, numpy)?;

                    if let Some(rr) = rr {
                        let tensor = rerun::Tensor::try_from(cube)?;
                        rr.log("cube", &tensor)?;

                        rr.log(
                            "cube/speed_per_bin",
                            &rerun::archetypes::Scalars::new([
                                cubemsg.bin_properties.speed_per_bin as f64,
                            ]),
                        )?;
                        rr.log(
                            "cube/range_per_bin",
                            &rerun::archetypes::Scalars::new([
                                cubemsg.bin_properties.range_per_bin as f64,
                            ]),
                        )?;
                        rr.log(
                            "cube/bin_per_speed",
                            &rerun::archetypes::Scalars::new([
                                cubemsg.bin_properties.bin_per_speed as f64,
                            ]),
                        )?;

                        rr.log("skiprate", &rerun::archetypes::Scalars::new([skiprate]))?;
                        rr.log("badrate", &rerun::archetypes::Scalars::new([badrate]))?;

                        rr.log(
                            "cubemsg",
                            &rerun::TextLog::new(format!(
                                "timestamp: {} captured: {} skipped: {} missing: {} badcount: {}",
                                cubemsg.timestamp,
                                cubemsg.packets_captured,
                                cubemsg.packets_skipped,
                                cubemsg.missing_data,
                                badcount
                            )),
                        )?;
                    }
                }
                Ok(None) => (),
                Err(err) => error!("Cube Error: {:?}", err),
            }
        }
    }
}

/// PCAP file replay loop
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

    for cap in pcarp::Capture::new(file) {
        match etherparse::SlicedPacket::from_ethernet(&cap.unwrap().data) {
            Err(err) => error!("Err {:?}", err),
            Ok(pkt) => {
                if let Some(etherparse::TransportSlice::Udp(udp)) = pkt.transport {
                    if TransportHeaderSlice::from_slice(udp.payload()).is_ok() {
                        match reader.read(udp.payload()) {
                            Ok(Some(cubemsg)) => {
                                let cube = format_cube(&cubemsg, numpy)?;

                                if let Some(rr) = rr {
                                    let tensor = rerun::Tensor::try_from(cube)?;
                                    rr.log("cube", &tensor)?;
                                }
                            }
                            Ok(None) => (),
                            // Ignore StartPattern errors when reading from pcap which includes
                            // non-SMS data
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

/// Live CAN target data loop
#[cfg(feature = "can")]
async fn can_loop(rr: &Option<RecordingStream>, device: Option<String>) {
    use rerun::Points3D;
    use tokio::task::yield_now;

    let iface = match device {
        Some(device) => device,
        None => loop {
            yield_now().await
        },
    };

    debug!("opening CAN interface {}", iface);
    let sock = socketcan::tokio::CanSocket::open(&iface).unwrap();

    loop {
        match can::read_message(&sock).await {
            Err(err) => println!("Error: {:?}", err),
            Ok(msg) => {
                trace!("radar CAN header {:?}", msg.header);

                if let Some(rr) = rr {
                    rr.log(
                        "radar/targets",
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

/// Convert spherical coordinates to Cartesian XYZ
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

/// Viridis colormap for power visualization
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
