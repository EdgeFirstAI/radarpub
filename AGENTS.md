# AGENTS.md - AI Assistant Development Guidelines

**Purpose:** Project-specific instructions for AI coding assistants (GitHub Copilot, Claude, Cursor, etc.)

**Organization Standards:** See [05-copilot-instructions.md](https://github.com/au-zone/sps) for Au-Zone universal rules

**Version:** 2.0
**Last Updated:** 2025-11-24

---

## Overview

This file provides **project-specific** guidelines. ALL projects must also follow:

- **Organization-wide:** [05-copilot-instructions.md](https://github.com/au-zone/sps) - License policy, security, Git/JIRA
- **Process docs:** 00-README through 11-cicd-pipelines in SPS repository
- **This file:** Project conventions, module structure, naming patterns, domain specifics

**Hierarchy:** Org standards (mandatory) → SPS processes (required) → This file (project-specific)

---

## Git Workflow

**Branch:** `<type>/PROJ-###[-desc]` (feature/bugfix/hotfix/release, JIRA key required)
**Commit:** `PROJ-###: Brief description` (50-72 chars, what done not how)
**PR:** main=2 approvals, develop=1. Link JIRA, squash features, merge commits for releases.

---

## ⚠️ CRITICAL RULES

### #1: NEVER Use cd Commands

```bash
# ✅ Modern tools work from root
cargo build --release
cargo test --workspace
cross build --target aarch64-unknown-linux-gnu --release

# ❌ AI loses context
cd target && ls  # Where are we now?
```

### #2: ALWAYS Use Virtual CAN for Testing

```bash
# ✅ Set up vcan for testing without hardware
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
RADAR_CAN_INTERFACE=vcan0 cargo test

# ❌ Running CAN tests without interface
cargo test  # Will fail!
```

**Why:** SocketCAN requires a CAN interface. Without hardware, vcan provides virtual interface.

### #3: Document Hardware-Specific Code

```rust
// ✅ CORRECT: Documented hardware dependency
// Smart Micro DRVEGRD UATv4 protocol: CAN extended frame format
// See: DRVEGRD Communication Protocol Specification v4.2, Section 3.1
let can_id = CanId::new(0x1FFFFFFF, true)?;

// ❌ WRONG: Magic numbers without explanation
let can_id = CanId::new(0x1FFFFFFF, true)?;
```

---

## Code Quality

**Edge-First:** 512MB-2GB RAM, ARM64, <50ms latency, 5-10yr lifecycle

**Standards:**

- Rust: Latest stable (currently 1.84.1), `cargo fmt`, `cargo clippy -- -D warnings`
- Zero warnings policy in CI/CD
- Follow Rust API guidelines

**Performance:** Stack>heap, zero-copy, arena allocators, NPU/GPU accel, profile on target

---

## Testing

**Coverage:** 70% min, 90%+ critical (measured: cargo-llvm-cov)
**Pyramid:** Unit 75%, Integration 20%, E2E 5%
**HIL:** Required for embedded (deploy to Maivin/Raivin, run suite)
**Edge Cases:** Null, bounds, concurrency, resource exhaustion

```rust
#[test]
fn test_edge_cases() {
    assert!(process(vec![]).is_ok());  // Empty
    assert!(validate(0).is_ok());       // Boundary
    assert!(parse("").is_err());        // Invalid
}
```

---

## License Policy (ZERO TOLERANCE)

**✅ Allowed:** MIT, Apache-2.0, BSD-2/3, ISC, 0BSD, Unlicense, Zlib, BSL-1.0
**⚠️ Conditional:** MPL-2.0/EPL-2.0 (deps ONLY), LGPL (dynamic, **FORBIDDEN in Rust**)
**❌ BLOCKED:** GPL, AGPL, SSPL, Commons Clause

**SBOM:** `make sbom` (scancode→CycloneDX). CI/CD blocks violations.

---

## Security

**Input:** Validate all, allowlists, size limits
**Creds:** NEVER hardcode. Env vars (ephemeral <48h) or vaults.
**Scans:** cargo audit. All MUST pass.
**Report:** support@au-zone.com "Security Vulnerability"

---

## Release

**Semver:** MAJOR.MINOR.PATCH
**CHANGELOG:** Update during dev under [Unreleased], move to [X.Y.Z] at release
**Pre-release:** `make pre-release` (lint+test+coverage+sbom), version ALL files, verify sync
**Tag:** After main merge + CI green. `git tag -a vX.Y.Z && git push origin vX.Y.Z`

**Version files to sync:** Cargo.toml, CHANGELOG.md

---

## Documentation

**Mandatory:** README (features/install/usage), ARCHITECTURE (HOW), SECURITY, CHANGELOG, LICENSE, NOTICE
**API:** 100% coverage (rustdoc) with Args/Returns/Errors/Examples
**Comments:** Public APIs, complex logic, performance, thread safety, hardware-specific

---

## Edge AI Specifics

**Platforms:** NXP i.MX8 Plus (Maivin/Raivin), ARM Cortex-A72
**Acceleration:** NPU (VSI), VPU (H.264/265)
**Memory:** Aligned DMA (64-byte), cache mgmt, zero-copy

---

## AI Assistant Practices

**Verify:** APIs exist, licenses OK, linters pass, test edges, match patterns
**Avoid:** Hallucinated APIs, GPL/AGPL, cd, hardcoded secrets, over-engineering
**Review:** ALL code. YOU are author (AI = tool). Test thoroughly.

---

## Project-Specific Conventions

### Technology Stack - RadarPub

**RadarPub - EdgeFirst Perception Radar Node**

- **Language**: Rust 1.70+ (currently 1.84.1)
- **Build system**: Cargo with feature flags
- **Key dependencies**:
  - `tokio` - Async runtime for multi-threaded execution
  - `zenoh` - ROS2-compatible publish/subscribe middleware
  - `socketcan` - Linux CAN bus interface
  - `ndarray` - 4D radar cube tensor operations
  - `dbscan` - Spatial clustering algorithm
  - `rerun` - 3D visualization tool (optional)
  - `edgefirst-schemas` - Message format definitions
  - `tracing-tracy` - Performance profiling (optional)
- **Target platforms**:
  - Primary: Linux aarch64 (ARM64) on Maivin/Raivin platforms
  - Development: Linux x86_64
  - Note: Linux-only due to socketcan dependency
- **Hardware requirements**:
  - Smart Micro DRVEGRD 169/174 radar sensor
  - CAN interface hardware (or vcan for testing)
  - UDP network interface for radar cube data

### Architecture

**Multi-threaded real-time sensor processing pipeline**

- **Pattern**: Multi-threaded async with Tokio + dedicated threads for performance-critical paths
- **Threading model**:
  - Main thread: Tokio async runtime (CAN communication, Zenoh publishing)
  - Cube thread: Dedicated thread for UDP radar cube reception
  - Cluster thread: Optional dedicated thread for DBSCAN clustering and tracking
  - TF static thread: Background thread for periodic transform publishing
  - Radar info thread: Background thread for periodic configuration publishing
- **Data flow**:

  ```text
  DRVEGRD Radar → CAN Interface → Target Parsing → Clustering → Zenoh (PointCloud2)
                → UDP Interface → Radar Cube Assembly → Zenoh (RadarCube)
  ```

- **Error handling**: Result types with custom `DrvegrdError` enum
- **Synchronization**: Kanal channels for inter-thread communication
- **Protocol implementations**:
  - CAN: Smart Micro DRVEGRD UATv4 protocol (src/can.rs)
  - Ethernet: SMS protocol for radar cube data (src/eth.rs)

### Module Structure

```text
src/
├── radarpub.rs       # Main entry point for radar publisher
├── drvegrdctl.rs     # Radar configuration utility
├── rerun.rs          # Visualization tool for PCAP playback
├── args.rs           # Command-line argument parsing
├── can.rs            # CAN interface and DRVEGRD protocol
├── eth.rs            # Ethernet/UDP radar cube reception
├── net.rs            # Network utilities
├── common.rs         # Shared types and utilities
└── clustering/       # Target clustering and tracking
    ├── mod.rs        # DBSCAN clustering
    ├── kalman.rs     # Kalman filter
    └── tracker.rs    # Multi-target tracking
```

### Build Commands

```bash
# Build release binary for ARM64 (production target)
cargo build --target aarch64-unknown-linux-gnu --release

# Build for local development
cargo build --release

# Build all binaries
cargo build --release --features "can,zenoh"
cargo build --release --bin drvegrdctl --features "can"
cargo build --release --bin drvegrd-rerun --features "rerun"

# Cross-compile for ARM64 using cross
cross build --target aarch64-unknown-linux-gnu --release

# Run all tests
cargo test --workspace

# Run tests with specific features
cargo test --features "can,zenoh"

# Generate documentation
cargo doc --no-deps --open

# Run clippy (zero warnings policy)
cargo clippy --all-targets -- -D warnings

# Format code
cargo fmt --all

# Security audit
cargo audit

# Build with Tracy profiling enabled
cargo build --release --features profiling

# Run pre-release checks
make pre-release
```

**Binary targets:**

- `radarpub` - Main radar publisher node (requires: can, zenoh)
- `drvegrdctl` - Radar configuration utility (requires: can)
- `drvegrd-rerun` - Visualization tool for PCAP playback (requires: rerun)

### Performance Targets

Real-time sensor processing for autonomous driving applications:

- **CAN frame latency**: Low-latency processing from reception to Zenoh publish
- **Radar cube processing**: Real-time (within frame period)
- **Target detection throughput**: Handles multiple targets per frame
- **Clustering latency**: Efficient DBSCAN + ByteTrack tracking
- **Memory footprint**: Optimized for edge deployment
- **CPU usage**: Efficient resource utilization on ARM platforms
- **Frame rate**: Matches radar sensor configuration

### Hardware Specifics

**Smart Micro DRVEGRD 169/174 Radar:**

- Frequency: 76-77 GHz (configurable via CAN)
- CAN interface: 500 kbps, extended frame format
- Ethernet: UDP broadcast for radar cube data
- Target list: Range, azimuth, elevation, doppler, RCS, power
- Radar cube: 4D tensor (chirp_types × range_gates × rx_channels × doppler_bins)

**Platform integration:**

- **Maivin**: Au-Zone edge AI platform with NXP i.MX 8M Plus
- **Raivin**: Automotive-grade variant
- **CAN hardware**: Requires Linux SocketCAN-compatible interface
- **Deployment**: systemd service, integrated with Yocto build system

**Development without hardware:**

- Use `vcan` (virtual CAN) for unit tests
- PCAP fixtures for radar cube replay
- Mock interfaces documented in tests/common/

### Testing Conventions

**Rust testing framework with hardware mocking**

**Unit tests:**

- Co-located in `#[cfg(test)] mod tests` at end of implementation files
- Test naming: `test_<function>_<scenario>` format
- Run with: `cargo test`

**Integration tests:**

- Separate `tests/` directory at project root
- Use PCAP fixtures from `tests/fixtures/` for replay
- Test full end-to-end pipelines with mocked hardware
- Run with: `cargo test --test integration_test_name`

**Test fixtures:**

- CAN traces: Recorded CAN frames for protocol testing
- PCAP files: UDP radar cube captures
- Synthetic data: Generated point clouds for clustering tests
- Place in `tests/fixtures/` or `tests/common/`

**Hardware-in-loop tests:**

- Manual QA testing on Maivin/Raivin platforms
- Automated via Yocto infrastructure (future)
- Not required for CI/CD (use mocks instead)

**Coverage target:**

- Minimum: 70% line coverage
- Critical paths (CAN parsing, clustering): 90%+
- Use `cargo-llvm-cov` for coverage reporting

**Testing without hardware:**

```bash
# Set up virtual CAN interface (Linux)
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

# Run tests with vcan
RADAR_CAN_INTERFACE=vcan0 cargo test
```

---

## Quick Reference

**Branch:** `feature/RADARPUB-123-desc`
**Commit:** `RADARPUB-123: Brief description`
**PR:** 2 approvals (main), 1 (develop)
**Licenses:** ✅ MIT/Apache/BSD | ❌ GPL/AGPL
**Tests:** 70% min, 90%+ critical
**Security:** support@au-zone.com
**Release:** Semver, make pre-release, wait CI, tag vX.Y.Z

---

**Process docs:** See [Au-Zone SPS repository](https://github.com/au-zone/sps) 00-README through 11-cicd-pipelines
**v2.0** | 2025-11-24 | sebastien@au-zone.com

*This file helps AI assistants contribute effectively to RadarPub while maintaining quality, security, and consistency.*
