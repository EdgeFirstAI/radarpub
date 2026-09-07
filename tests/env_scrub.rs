// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.

//! End-to-end check that `KEY=""` in the environment behaves as unset.
//!
//! Runs with `harness = false` so this `main` is the only thread in the
//! process when the environment is mutated, which `scrub_empty_env` requires.
//!
//! `main` speaks the small subset of the libtest CLI that `cargo test` and
//! `cargo nextest` use to enumerate (`--list --format terse`) and select
//! (`--exact <name>`, `--ignored`, positional filters) tests, so the target
//! is discovered and reported like any other test. nextest rejects a binary
//! whose `--list` output is not `<name>: test` lines.
#![allow(dead_code)] // args.rs's own #[cfg(test)] unit tests are compiled but never run here

// `Args` and `KEEP` live in a private module of the `edgefirst-radarpub`
// binary rather than the lib crate, so include the module source directly.
#[path = "../src/args.rs"]
mod args;
use args::{Args, KEEP};
use clap::Parser;
use radarpub::scrub_empty_env;

/// The single test this binary provides, as reported to the harness.
const TEST_NAME: &str = "empty_env_is_treated_as_unset";

/// A float, a boolean and an enum-like value, each with a non-empty default,
/// all written as `KEY=""` in /etc/default/radarpub. `KEEP` is empty for
/// radarpub so all three must be scrubbed.
const VARS: [&str; 3] = ["CLUSTERING_EPS", "CLUSTERING", "RUST_LOG"];
const ARGV: [&str; 1] = ["edgefirst-radarpub"];

/// libtest flags that consume the following argument, so it is not a filter.
const VALUE_FLAGS: [&str; 6] = [
    "--test-threads",
    "--format",
    "--skip",
    "--logfile",
    "--color",
    "--shuffle-seed",
];

/// What the harness asked this binary to do.
struct Request {
    list: bool,
    ignored: bool,
    exact: bool,
    filters: Vec<String>,
}

fn parse_request(argv: impl IntoIterator<Item = String>) -> Request {
    let mut req = Request {
        list: false,
        ignored: false,
        exact: false,
        filters: Vec::new(),
    };
    let mut argv = argv.into_iter();
    while let Some(arg) = argv.next() {
        match arg.as_str() {
            "--list" => req.list = true,
            "--ignored" => req.ignored = true,
            "--exact" => req.exact = true,
            flag if VALUE_FLAGS.contains(&flag) => {
                argv.next();
            }
            flag if flag.starts_with('-') => {}
            filter => req.filters.push(filter.to_owned()),
        }
    }
    req
}

fn selected(req: &Request) -> bool {
    req.filters.is_empty()
        || req.filters.iter().any(|f| {
            if req.exact {
                f == TEST_NAME
            } else {
                TEST_NAME.contains(f.as_str())
            }
        })
}

fn main() {
    let req = parse_request(std::env::args().skip(1));
    // This binary has no #[ignore]d tests, so `--ignored` selects nothing.
    if req.list {
        if !req.ignored && selected(&req) {
            println!("{TEST_NAME}: test");
        }
        return;
    }
    if req.ignored || !selected(&req) {
        return;
    }

    for name in VARS {
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
    for name in VARS {
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
