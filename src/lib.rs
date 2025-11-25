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
