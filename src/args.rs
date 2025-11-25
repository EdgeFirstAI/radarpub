// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.

use std::{fmt, io};

use clap::{Parser, ValueEnum};
use serde_json::json;
use tracing::level_filters::LevelFilter;
use zenoh::config::{Config, WhatAmI};

#[derive(Debug)]
pub enum Error {
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
pub enum CenterFrequency {
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
pub enum FrequencySweep {
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
pub enum RangeToggle {
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
pub enum DetectionSensitivity {
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
pub struct Args {
    /// The center frequency for the radar.
    #[arg(long, env, default_value = "medium")]
    pub center_frequency: CenterFrequency,

    /// The frequency sweep which controls the range of the radar.
    #[arg(long, env, default_value = "short")]
    pub frequency_sweep: FrequencySweep,

    /// The range toggle mode allows the radar to alternate between various
    /// frequencies.
    #[arg(long, env, default_value = "off")]
    pub range_toggle: RangeToggle,

    /// The detection sensitivity controls the radar's ability to detect
    /// targets.
    #[arg(long, env, default_value = "medium")]
    pub detection_sensitivity: DetectionSensitivity,

    /// Enable streaming the low-level radar data cube on the cube_topic.
    #[arg(long, env, default_value = "false")]
    pub cube: bool,

    /// Enable radar target clustering task.
    #[arg(long, env, default_value = "false")]
    pub clustering: bool,

    /// Clustering window size in frames (one frame is 55ms).
    #[arg(long, env, default_value = "6")]
    pub window_size: usize,

    /// Clustering DBSCAN distance limit (euclidean distance)
    #[arg(long, env, default_value = "1")]
    pub clustering_eps: f64,

    /// Clustering DBSCAN parameter scaling. Parameter order is x, y, z, speed.
    /// Set the appropriate axis to 0 to ignore that axis
    #[arg(
        long,
        env,
        default_value = "1 1 0 0",
        value_delimiter = ' ',
        num_args = 4
    )]
    pub clustering_param_scale: Vec<f32>,

    /// Clustering DBSCAN point limit. Minimum 3
    #[arg(long, env, default_value = "5")]
    pub clustering_point_limit: usize,

    /// mirror the radar data
    #[arg(long, env)]
    pub mirror: bool,

    /// can device connected to radar
    #[arg(long, default_value = "can0")]
    pub can: String,

    /// radar frame transform vector from base_link
    #[arg(
        long,
        env,
        default_value = "0 0 0",
        value_delimiter = ' ',
        num_args = 3
    )]
    pub radar_tf_vec: Vec<f64>,

    /// radar frame transform quaternion from base_link
    #[arg(
        long,
        env,
        default_value = "0 0 0 1",
        value_delimiter = ' ',
        num_args = 4
    )]
    pub radar_tf_quat: Vec<f64>,

    /// The name of the base frame
    #[arg(long, env, default_value = "base_link")]
    pub base_frame_id: String,

    /// The name of the radar frame
    #[arg(long, env, default_value = "radar")]
    pub radar_frame_id: String,

    /// radar targets topic name
    #[arg(long, default_value = "rt/radar/targets")]
    pub targets_topic: String,

    /// radar clusters topic name
    #[arg(long, default_value = "rt/radar/clusters")]
    pub clusters_topic: String,

    /// radar data cube topic name
    #[arg(long, default_value = "rt/radar/cube")]
    pub cube_topic: String,

    /// Application log level
    #[arg(long, env, default_value = "info")]
    pub rust_log: LevelFilter,

    /// Enable Tracy profiler broadcast
    #[arg(long, env)]
    pub tracy: bool,

    /// zenoh connection mode
    #[arg(long, env, default_value = "peer")]
    mode: WhatAmI,

    /// connect to zenoh endpoints
    #[arg(long, env)]
    connect: Vec<String>,

    /// listen to zenoh endpoints
    #[arg(long, env)]
    listen: Vec<String>,

    /// disable zenoh multicast scouting
    #[arg(long, env)]
    no_multicast_scouting: bool,
}

impl From<Args> for Config {
    fn from(args: Args) -> Self {
        let mut config = Config::default();

        config
            .insert_json5("mode", &json!(args.mode).to_string())
            .unwrap();

        if !args.connect.is_empty() {
            config
                .insert_json5("connect/endpoints", &json!(args.connect).to_string())
                .unwrap();
        }

        if !args.listen.is_empty() {
            config
                .insert_json5("listen/endpoints", &json!(args.listen).to_string())
                .unwrap();
        }

        if args.no_multicast_scouting {
            config
                .insert_json5("scouting/multicast/enabled", &json!(false).to_string())
                .unwrap();
        }

        config
            .insert_json5("scouting/multicast/interface", &json!("lo").to_string())
            .unwrap();

        config
    }
}
