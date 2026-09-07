// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.

//! End-to-end check that `KEY=""` in the environment behaves as unset.
//!
//! Runs with `harness = false` so this `main` is the only thread in the
//! process when the environment is mutated, which `scrub_empty_env` requires.
#![allow(dead_code)] // args.rs's own #[cfg(test)] unit tests are compiled but never run here

// `Args` and `KEEP` live in a private module of the `edgefirst-radarpub`
// binary rather than the lib crate, so include the module source directly.
#[path = "../src/args.rs"]
mod args;
use args::{Args, KEEP};
use clap::Parser;
use radarpub::scrub_empty_env;

const ARGV: [&str; 1] = ["edgefirst-radarpub"];

fn main() {
    // A float, a boolean and an enum-like value, each with a non-empty
    // default; `KEEP` is empty for radarpub so all three must be scrubbed.
    for name in ["CLUSTERING_EPS", "CLUSTERING", "RUST_LOG"] {
        // SAFETY: single-threaded — this is `main` before any thread is spawned.
        std::env::set_var(name, "");
    }
    let before = Args::try_parse_from(ARGV);
    assert!(
        before.is_err(),
        "empty vars must fail to parse before scrubbing: {before:?}"
    );

    // SAFETY: still single-threaded.
    unsafe { scrub_empty_env::<Args>(KEEP) };
    for name in ["CLUSTERING_EPS", "CLUSTERING", "RUST_LOG"] {
        assert!(
            std::env::var_os(name).is_none(),
            "{name} should have been removed"
        );
    }
    let args = Args::try_parse_from(ARGV).expect("defaults must apply after scrubbing");
    assert_eq!(args.clustering_eps, 1.0);
    assert!(!args.clustering);
    assert_eq!(args.rust_log, tracing::level_filters::LevelFilter::INFO);
    println!("env_scrub: ok");
}
