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

/// Environment variables where an empty value is meaningful and must be
/// preserved by [`radarpub::scrub_empty_env`] (i.e. the argument has a
/// non-empty default but `""` is a documented "disable" sentinel).
///
/// No radarpub argument uses an empty string as a sentinel: every env-bound
/// argument with a non-empty default is a number, boolean, enum or frame ID
/// for which `""` is simply invalid, and the list arguments (`CONNECT`,
/// `LISTEN`) have no default so scrubbing them yields the same empty list.
pub const KEEP: &[&str] = &[];

/// Command-line arguments for EdgeFirst Radar Publisher.
///
/// This structure defines all configuration options for the radar node,
/// including radar parameters, clustering, Zenoh configuration, and
/// debugging options. Arguments can be specified via command line or
/// environment variables.
///
/// # Example
///
/// ```bash
/// # Via command line
/// edgefirst-radarpub --center-frequency medium --frequency-sweep short
///
/// # Via environment variables
/// export CENTER_FREQUENCY=medium
/// export FREQUENCY_SWEEP=short
/// edgefirst-radarpub
/// ```
#[derive(Parser, Debug, Clone)]
#[command(author, version, about, long_about = None)]
pub struct Args {
    /// The center frequency for the radar.
    #[arg(long, env = "CENTER_FREQUENCY", default_value = "medium")]
    pub center_frequency: CenterFrequency,

    /// The frequency sweep which controls the range of the radar.
    #[arg(long, env = "FREQUENCY_SWEEP", default_value = "short")]
    pub frequency_sweep: FrequencySweep,

    /// The range toggle mode allows the radar to alternate between various
    /// frequencies.
    #[arg(long, env = "RANGE_TOGGLE", default_value = "off")]
    pub range_toggle: RangeToggle,

    /// The detection sensitivity controls the radar's ability to detect
    /// targets.
    #[arg(long, env = "DETECTION_SENSITIVITY", default_value = "medium")]
    pub detection_sensitivity: DetectionSensitivity,

    /// Enable streaming the low-level radar data cube on the cube_topic.
    #[arg(long, env = "CUBE", default_value = "false")]
    pub cube: bool,

    /// Enable radar target clustering task.
    #[arg(long, env = "CLUSTERING", default_value = "false")]
    pub clustering: bool,

    /// Clustering window size in frames (one frame is 55ms).
    #[arg(long, env = "WINDOW_SIZE", default_value = "6")]
    pub window_size: usize,

    /// Clustering DBSCAN distance limit (euclidean distance)
    #[arg(long, env = "CLUSTERING_EPS", default_value = "1")]
    pub clustering_eps: f64,

    /// Clustering DBSCAN parameter scaling. Parameter order is x, y, z, speed.
    /// Set the appropriate axis to 0 to ignore that axis
    #[arg(
        long,
        env = "CLUSTERING_PARAM_SCALE",
        default_value = "1 1 0 0",
        value_delimiter = ' ',
        num_args = 4
    )]
    pub clustering_param_scale: Vec<f32>,

    /// Clustering DBSCAN point limit. Minimum 3
    #[arg(long, env = "CLUSTERING_POINT_LIMIT", default_value = "5")]
    pub clustering_point_limit: usize,

    /// Mirror the radar data
    #[arg(long, env = "MIRROR")]
    pub mirror: bool,

    /// Target list processing latency in nanoseconds, subtracted from the
    /// host receive time of the target list header to estimate the
    /// acquisition time. The default is two 55 ms radar cycles, from the
    /// DRVEGRD datasheet processing latency of 2 to 4 cycles.
    #[arg(long, env = "TARGETS_LATENCY", default_value = "110000000")]
    pub targets_latency: u64,

    /// Radar cube processing latency in nanoseconds, subtracted from the
    /// host receive time of the cube's start-of-frame packet to estimate the
    /// acquisition time. The default is two 55 ms radar cycles, from the
    /// DRVEGRD datasheet processing latency of 2 to 4 cycles.
    #[arg(long, env = "CUBE_LATENCY", default_value = "110000000")]
    pub cube_latency: u64,

    /// CAN device connected to radar
    #[arg(long, default_value = "can0")]
    pub can: String,

    /// Radar frame transform vector from base_link (x y z in meters)
    #[arg(
        long,
        env = "RADAR_TF_VEC",
        default_value = "0 0 0",
        value_delimiter = ' ',
        num_args = 3
    )]
    pub radar_tf_vec: Vec<f64>,

    /// Radar frame transform quaternion from base_link (x y z w)
    #[arg(
        long,
        env = "RADAR_TF_QUAT",
        default_value = "0 0 0 1",
        value_delimiter = ' ',
        num_args = 4
    )]
    pub radar_tf_quat: Vec<f64>,

    /// TF frame ID for robot base
    #[arg(long, env = "BASE_FRAME_ID", default_value = "base_link")]
    pub base_frame_id: String,

    /// TF frame ID for radar frame
    #[arg(long, env = "RADAR_FRAME_ID", default_value = "radar")]
    pub radar_frame_id: String,

    /// Radar targets topic name
    #[arg(long, default_value = "radar/targets")]
    pub targets_topic: String,

    /// Radar clusters topic name
    #[arg(long, default_value = "radar/clusters")]
    pub clusters_topic: String,

    /// Radar data cube topic name
    #[arg(long, default_value = "radar/cube")]
    pub cube_topic: String,

    /// Application log level
    #[arg(long, env = "RUST_LOG", default_value = "info")]
    pub rust_log: LevelFilter,

    /// Enable Tracy profiler broadcast
    #[arg(long, env = "TRACY")]
    pub tracy: bool,

    /// Zenoh participant mode (peer, client, or router)
    #[arg(long, env = "MODE", default_value = "peer")]
    mode: WhatAmI,

    /// Zenoh endpoints to connect to (can specify multiple)
    #[arg(long, env = "CONNECT")]
    connect: Vec<String>,

    /// Zenoh endpoints to listen on (can specify multiple)
    #[arg(long, env = "LISTEN")]
    listen: Vec<String>,

    /// Disable Zenoh multicast peer discovery
    #[arg(long, env = "NO_MULTICAST_SCOUTING")]
    no_multicast_scouting: bool,
}

/// System hostname used as the Zenoh session namespace.
///
/// Empty or `/`-containing hostnames would create unintended sub-keys, so we
/// fall back to `"localhost"` and warn. Two devices both falling back would
/// silently share a namespace; that is a deployment defect.
fn zenoh_namespace() -> String {
    let raw = gethostname::gethostname().to_string_lossy().into_owned();
    if raw.is_empty() || raw.contains('/') {
        tracing::warn!(
            hostname = %raw,
            "system hostname is empty or contains '/' — falling back to \"localhost\""
        );
        "localhost".into()
    } else {
        raw
    }
}

impl From<Args> for Config {
    fn from(args: Args) -> Self {
        let mut config = Config::default();

        // Session namespace = hostname: application keys are bare
        // (`radar/targets`) and the wire form is `{hostname}/radar/targets`.
        config
            .insert_json5("namespace", &json!(zenoh_namespace()).to_string())
            .unwrap();

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

#[cfg(test)]
mod tests {
    use super::*;
    use clap::Parser;
    use std::collections::HashMap;

    fn parse_cli() -> Args {
        Args::parse_from([
            "edgefirst-radarpub",
            "--targets-topic",
            "radar/targets",
            "--clusters-topic",
            "radar/clusters",
            "--cube-topic",
            "radar/cube",
        ])
    }

    #[test]
    fn zenoh_config_sets_namespace() {
        let ns = zenoh_namespace();
        assert!(!ns.is_empty(), "namespace should be non-empty");
        assert!(!ns.contains('/'), "namespace must not contain '/'");
        let rendered = Config::from(parse_cli()).to_string();
        assert!(
            rendered.contains(&ns),
            "config should include namespace {ns}: {rendered}"
        );
    }

    #[test]
    fn cli_topics_have_no_rt_prefix() {
        let args = parse_cli();
        assert_eq!(args.targets_topic, "radar/targets");
        assert_eq!(args.clusters_topic, "radar/clusters");
        assert_eq!(args.cube_topic, "radar/cube");
    }

    /// Env-bound arguments with a non-empty default where we have consciously
    /// decided that an empty value is NOT meaningful (so scrubbing to the
    /// default is correct).
    const SCRUB_REVIEWED: &[&str] = &[
        "CENTER_FREQUENCY",
        "FREQUENCY_SWEEP",
        "RANGE_TOGGLE",
        "DETECTION_SENSITIVITY",
        "CUBE",
        "CLUSTERING",
        "WINDOW_SIZE",
        "CLUSTERING_EPS",
        "CLUSTERING_PARAM_SCALE",
        "CLUSTERING_POINT_LIMIT",
        "TARGETS_LATENCY",
        "CUBE_LATENCY",
        "RADAR_TF_VEC",
        "RADAR_TF_QUAT",
        "BASE_FRAME_ID",
        "RADAR_FRAME_ID",
        "RUST_LOG",
        "MODE",
    ];

    #[test]
    fn every_env_arg_is_either_scrubbable_or_explicitly_kept() {
        use clap::CommandFactory;
        for arg in Args::command().get_arguments() {
            let Some(env) = arg.get_env() else { continue };
            let name = env.to_string_lossy().into_owned();
            let has_nonempty_default = arg
                .get_default_values()
                .first()
                .is_some_and(|d| !d.is_empty());
            if has_nonempty_default && !KEEP.contains(&name.as_str()) {
                assert!(
                    SCRUB_REVIEWED.contains(&name.as_str()),
                    "{name} has a non-empty default; decide whether empty is meaningful \
                     and add it to KEEP or SCRUB_REVIEWED"
                );
            }
        }
    }

    /// Fake environment lookup for `empty_env_vars`: tests must never mutate
    /// the real process environment because libtest runs them on parallel
    /// threads. The end-to-end scrub is covered by `tests/env_scrub.rs`,
    /// which runs single-threaded with `harness = false`.
    fn fake_env(vars: &[(&str, &str)]) -> impl Fn(&str) -> Option<String> {
        let vars: HashMap<String, String> = vars
            .iter()
            .map(|(k, v)| ((*k).to_owned(), (*v).to_owned()))
            .collect();
        move |name| vars.get(name).cloned()
    }

    #[test]
    fn empty_env_var_is_scrubbed() {
        // Mirrors an /etc/default/radarpub entry written as KEY="".
        let env = fake_env(&[("CLUSTERING_EPS", "")]);
        assert_eq!(
            radarpub::empty_env_vars::<Args>(KEEP, env),
            ["CLUSTERING_EPS"]
        );
    }

    #[test]
    fn non_empty_env_var_is_not_scrubbed() {
        let env = fake_env(&[("CLUSTERING_EPS", "2.5"), ("CLUSTERING", "true")]);
        assert!(radarpub::empty_env_vars::<Args>(KEEP, env).is_empty());
    }

    #[test]
    fn unset_env_var_is_not_scrubbed() {
        let env = fake_env(&[]);
        assert!(radarpub::empty_env_vars::<Args>(KEEP, env).is_empty());
    }

    #[test]
    fn kept_env_var_is_not_scrubbed_even_when_empty() {
        let env = fake_env(&[("CLUSTERING_EPS", ""), ("CLUSTERING", "")]);
        assert_eq!(
            radarpub::empty_env_vars::<Args>(&["CLUSTERING_EPS"], env),
            ["CLUSTERING"]
        );
    }

    #[test]
    fn env_var_not_bound_to_an_argument_is_never_scrubbed() {
        let env = fake_env(&[("UNRELATED_EMPTY_VAR", ""), ("WINDOW_SIZE", "")]);
        assert_eq!(radarpub::empty_env_vars::<Args>(KEEP, env), ["WINDOW_SIZE"]);
    }
}
