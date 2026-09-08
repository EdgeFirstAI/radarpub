// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.

//! RadarPub Library
//!
//! This library provides core functionality for interfacing with Smart Micro
//! DRVEGRD radar sensors and publishing data to the EdgeFirst Perception
//! Middleware via Zenoh.
//!
//! # Features
//!
//! - **CAN Interface** - Read radar target data via SocketCAN
//! - **Ethernet Interface** - Receive 4D radar cube data via UDP
//! - **Clustering** - DBSCAN spatial clustering for target grouping
//! - **Tracking** - ByteTrack algorithm with Kalman filtering
//! - **Zenoh Publishing** - ROS2-compatible message publishing
//!
//! # Examples
//!
//! See the `examples/` directory for complete usage examples:
//! - `radar_viewer` - Direct radar visualization with Rerun
//! - `zenoh_viewer` - Subscribe to Zenoh topics and visualize

#![warn(missing_docs)]

/// CAN interface and DRVEGRD protocol implementation
#[cfg(feature = "can")]
pub mod can;

/// Common types and utilities
pub mod common;

/// Ethernet/UDP radar cube reception
pub mod eth;

/// Network utilities for UDP communication
pub mod net;

/// Clustering and tracking algorithms
pub mod clustering;

/// Names of `C`'s env-bound arguments whose value, as reported by `var`, is
/// present but empty and not listed in `keep`.
///
/// Pure: the environment is only read through `var`, so this can be unit
/// tested with a fake lookup and no process-wide mutation.
///
/// # Example
///
/// ```
/// use clap::Parser;
///
/// #[derive(Parser)]
/// struct Args {
///     #[arg(long, env = "WINDOW_SIZE", default_value = "6")]
///     window_size: usize,
/// }
///
/// let fake = |name: &str| (name == "WINDOW_SIZE").then(String::new);
/// assert_eq!(radarpub::empty_env_vars::<Args>(&[], fake), ["WINDOW_SIZE"]);
/// assert!(radarpub::empty_env_vars::<Args>(&["WINDOW_SIZE"], fake).is_empty());
/// ```
pub fn empty_env_vars<C: clap::CommandFactory>(
    keep: &[&str],
    var: impl Fn(&str) -> Option<String>,
) -> Vec<String> {
    C::command()
        .get_arguments()
        .filter_map(|arg| arg.get_env().map(|e| e.to_string_lossy().into_owned()))
        .filter(|name| !keep.contains(&name.as_str()))
        .filter(|name| var(name).is_some_and(|v| v.is_empty()))
        .collect()
}

/// Treat an empty environment variable as unset, so clap's declared
/// `default_value` applies instead of failing to parse.
///
/// systemd `EnvironmentFile`s commonly write `KEY=""` to mean "unset", but
/// clap treats an empty variable as *present* and hands `""` to the value
/// parser, which fails for numeric, boolean and enum arguments and yields a
/// one-element `[""]` for list arguments. A custom value parser cannot fix
/// this because it cannot report "absent", so the variable must be removed
/// before clap sees it.
///
/// Only variables bound to `C`'s own arguments are considered; unrelated
/// process environment is left alone. `keep` names variables where an empty
/// value is meaningful and must be preserved.
///
/// # Safety
///
/// Must be called before any thread is spawned, i.e. before the tokio
/// runtime is built. Mutating the process environment is not thread-safe.
///
/// # Example
///
/// ```no_run
/// use clap::Parser;
///
/// #[derive(Parser)]
/// struct Args {
///     #[arg(long, env = "WINDOW_SIZE", default_value = "6")]
///     window_size: usize,
/// }
///
/// // SAFETY: single-threaded; runs before any runtime is built.
/// unsafe { radarpub::scrub_empty_env::<Args>(&[]) };
/// let args = Args::parse();
/// ```
pub unsafe fn scrub_empty_env<C: clap::CommandFactory>(keep: &[&str]) {
    for name in empty_env_vars::<C>(keep, |name| std::env::var(name).ok()) {
        std::env::remove_var(&name);
    }
}
