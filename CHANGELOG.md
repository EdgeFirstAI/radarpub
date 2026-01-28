# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [1.6.1] - 2025-01-28

### Changed

- **CDR serialization migration**: Replaced direct `cdr` crate usage with `edgefirst_schemas::serde_cdr` API
  for consistent ROS2 message serialization across EdgeFirst components
- **Dependencies updated**:
  - edgefirst-schemas v1.4.0 → v1.5.2 (provides serde_cdr API)
  - Removed direct cdr dependency (now internal to edgefirst-schemas)
- **CI/CD improvements**:
  - Simplified test workflow (removed cross-compilation from tests, build handles that)
  - Updated SBOM generation with cargo-cyclonedx output handling
  - Fixed GPL license check to allow OR-alternatives (Apache-2.0 OR LGPL-2.1 OR MIT)
  - Updated SonarCloud configuration for EdgeFirstAI organization

### Added

- TESTING.md: Comprehensive manual hardware testing documentation
  - CAN and Ethernet mode setup instructions
  - Virtual CAN configuration for local testing
  - Troubleshooting guides for common issues

## [1.6.0] - 2025-01-25

### Changed

- **Repository migrated to EdgeFirstAI organization**: https://github.com/EdgeFirstAI/radarpub
- **Updated dependencies to match EdgeFirst samples**:
  - zenoh v1.6.2 → v1.3.4 (API compatibility with samples)
  - rerun v0.23.1 → v0.27.2 (with "clap" feature)
  - edgefirst-schemas v1.4.1 → v1.4.0
  - tokio v1.48.0 → v1.45.0
  - clap v4.5.53 → v4.5.52
- **Fixed zenoh API usage**: Config::default() + insert_json5(), recv_async() loops, payload().to_bytes()
- **Fixed rerun API deprecations**: Scalars archetype, removed set_time_seconds, connect_grpc_opts signature
- **Fixed RadarCube schema**: Updated to use shape/cube fields instead of width/height/data
- **Complete API documentation**: 100% rustdoc coverage for all public APIs
- **ARCHITECTURE.md rewritten**: Factual, concise documentation with mermaid packet diagrams
- Zero clippy warnings across all targets and features

### Added

- Comprehensive rustdoc documentation for all modules:
  - src/can.rs: CAN protocol and DRVEGRD UATv4 implementation
  - src/eth.rs: Ethernet/UDP SMS protocol for radar cube data
  - src/common.rs: Process priority and socket utilities
  - src/net.rs: Network UDP receiver functions
  - src/clustering/: DBSCAN clustering and ByteTrack tracking
- Mermaid packet format diagrams in ARCHITECTURE.md:
  - CAN Header Frame and Target Frame structures
  - SMS Transport Header, Debug Header, Port Header
  - Cube Header and Bin Properties structures

- **Examples directory**: Reorganized visualization examples
  - `radar_viewer` - Direct radar sensor visualization with Rerun
  - `zenoh_viewer` - Subscribe to Zenoh topics and visualize with Rerun
  - `examples/README.md` - Comprehensive example documentation
- **Library API**: Created `lib.rs` to expose modules for examples and external use
- **Open source readiness**:
  - `.github/PULL_REQUEST_TEMPLATE.md` - PR checklist for contributors
  - CODE_OF_CONDUCT.md reporting mechanism (conduct@au-zone.com)
- Open source migration: README, CONTRIBUTING, ARCHITECTURE, AGENTS documentation
- CODE_OF_CONDUCT.md (Contributor Covenant 3.0)
- SECURITY.md (security policy and vulnerability reporting)
- NOTICE.md (third-party attributions)
- Apache-2.0 license headers in all source files
- Comprehensive Mermaid diagrams for architecture visualization

### Removed

- **`drvegrd-rerun` binary**: Replaced with `radar_viewer` and `zenoh_viewer` examples
- `src/rerun.rs`: Functionality split into examples directory

### Fixed

- Markdown linting errors in documentation
- Corrected dual-channel architecture representation (CAN + UDP separate)
- Fixed all API examples to match actual function signatures

## [1.5.3] - 2025-11-22

### Fixed

- RadarCube scales array now matches published dimensions instead of raw SMS module scales

## [1.5.2] - 2024-11-XX

### Changed

- Updated Rerun dependency and API for new version

## [1.5.1] - 2024-11-XX

### Changed

- Updated dependencies to latest versions

## [1.5.0] - 2024-11-XX

### Changed

- Migrated to Zenoh 1.2

### Added

- Rust target-specific settings moved to .cargo/config.toml
- Memory profiling when profiling feature enabled
- Tracing instrumentation integrated

### Fixed

- CAN DRVEGRD messages now always treated as little endian
- Clippy warnings resolved

[Unreleased]: https://github.com/EdgeFirstAI/radarpub/compare/v1.6.1...HEAD
[1.6.1]: https://github.com/EdgeFirstAI/radarpub/compare/v1.6.0...v1.6.1
[1.6.0]: https://github.com/EdgeFirstAI/radarpub/releases/tag/v1.6.0
[1.5.3]: https://github.com/EdgeFirstAI/radarpub/compare/v1.5.2...v1.5.3
[1.5.2]: https://github.com/EdgeFirstAI/radarpub/compare/v1.5.1...v1.5.2
[1.5.1]: https://github.com/au-zone/radarpub/compare/v1.5.0...v1.5.1
[1.5.0]: https://github.com/au-zone/radarpub/releases/tag/v1.5.0
