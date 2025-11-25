# Contributing to RadarPub

Thank you for your interest in contributing to RadarPub! This project is part of the EdgeFirst Perception stack, advancing edge AI and real-time sensor processing capabilities for autonomous systems.

## Code of Conduct

Please read and follow our [Code of Conduct](CODE_OF_CONDUCT.md) before contributing. We are committed to providing a welcoming and inclusive environment for all contributors.

## Ways to Contribute

We welcome contributions in many forms:

- **💻 Code**: Features, bug fixes, performance improvements, refactoring
- **📖 Documentation**: Improvements, examples, tutorials, API clarifications
- **🧪 Testing**: Bug reports, test coverage improvements, hardware platform validation
- **💬 Community**: Answer questions, write blog posts, speak at meetups, share use cases
- **🎨 Examples**: Sample applications, integration guides, configuration templates

## Before You Start

1. **Check existing work**: Review [open issues](https://github.com/EdgeFirstAI/radarpub/issues) and [pull requests](https://github.com/EdgeFirstAI/radarpub/pulls) to avoid duplicate efforts
2. **Discuss significant changes**: For major features or architectural changes, open an issue for discussion before investing significant time
3. **Review the roadmap**: Check our [project roadmap](https://github.com/EdgeFirstAI/radarpub/projects) to understand planned direction
4. **Consider EdgeFirst integration**: Think about how changes might affect integration with EdgeFirst Studio and Perception Middleware

## Development Setup

### Prerequisites

**Required:**

- Rust toolchain 1.70 or later ([rustup](https://rustup.rs/))
- Linux development environment (required for SocketCAN)
- Git for version control

**Optional:**

- CAN interface hardware (or use virtual CAN for testing)
- Smart Micro DRVEGRD radar sensor (for hardware validation)
- [Tracy profiler](https://github.com/wolfpld/tracy) (for performance analysis)
- EdgeFirst Studio account (free tier available for integration testing)

### Initial Setup

```bash
# Clone the repository
git clone https://github.com/EdgeFirstAI/radarpub.git
cd radarpub

# Build the project
cargo build --release

# Run tests
cargo test --workspace

# Check code formatting
cargo fmt --all -- --check

# Run linter
cargo clippy --all-targets -- -D warnings
```

### Building for ARM64 (Maivin/Raivin)

```bash
# Install cross-compilation tool
cargo install cross

# Build for ARM64
cross build --target aarch64-unknown-linux-gnu --release

# Binary located at: target/aarch64-unknown-linux-gnu/release/radarpub
```

### Setting Up Virtual CAN (for Testing Without Hardware)

```bash
# Load vcan kernel module
sudo modprobe vcan

# Create virtual CAN interface
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

# Verify interface
ip link show vcan0

# Run tests with virtual CAN
RADAR_CAN_INTERFACE=vcan0 cargo test
```

### Running the Binary Targets

```bash
# Main radar publisher (requires CAN interface)
cargo run --bin radarpub --features "can,zenoh" -- \
  --can-interface vcan0 \
  --zenoh-mode peer

# Radar control utility
cargo run --bin drvegrdctl --features "can" -- \
  --can-interface vcan0 \
  status

# Visualization tool (PCAP replay)
cargo run --bin drvegrd-rerun --features "rerun" -- \
  --pcap-file testdata/radar_capture.pcap
```

## Contribution Process

### 1. Fork and Clone

```bash
# Fork the repository via GitHub web interface

# Clone your fork
git clone https://github.com/YOUR_USERNAME/radarpub.git
cd radarpub

# Add upstream remote
git remote add upstream https://github.com/au-zone/radarpub.git
```

### 2. Create a Feature Branch

Use a descriptive branch name:

```bash
# For features
git checkout -b feature/add-radar-xyz-support

# For bug fixes
git checkout -b bugfix/fix-can-timeout-issue

# For JIRA-tracked work (Au-Zone internal)
git checkout -b feature/EDGEAI-123-add-authentication
```

### 3. Make Your Changes

**Code Guidelines:**

- Follow Rust API Guidelines: https://rust-lang.github.io/api-guidelines/
- Run `cargo fmt --all` before committing (enforced in CI)
- Ensure `cargo clippy --all-targets -- -D warnings` passes (zero warnings policy)
- Add tests for new functionality (minimum 70% coverage)
- Update documentation (rustdoc comments for public APIs)

**Commit Guidelines:**

- Write clear, descriptive commit messages
- Use present tense ("Add feature" not "Added feature")
- Reference issue numbers when applicable
- Keep commits focused on a single logical change

**Example commits:**

```bash
git commit -m "Add support for XYZ radar sensor

- Implement XYZ protocol parser
- Add integration tests with PCAP fixtures
- Update documentation with sensor compatibility

Closes #123"
```

### 4. Add Tests

**Unit Tests:**

```rust
// Co-located at the end of implementation files
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_can_frame_valid_header() {
        let frame = create_test_frame();
        let result = parse_can_frame(&frame);
        assert!(result.is_ok());
        assert_eq!(result.unwrap().id, 0x123);
    }
}
```

**Integration Tests:**

```rust
// Separate file in tests/ directory
// tests/can_integration_test.rs
use radarpub::can::*;

#[test]
fn test_end_to_end_target_processing() {
    // Test full pipeline with mock CAN data
}
```

**Test Fixtures:**

- Place test data in `tests/fixtures/`
- Use PCAP files for UDP replay tests
- Use CAN trace files for protocol tests

### 5. Update Documentation

**Rustdoc Comments:**

```rust
/// Parse a CAN frame from the DRVEGRD radar sensor.
///
/// This function extracts target information from CAN frames following
/// the Smart Micro UATv4 protocol specification.
///
/// # Arguments
///
/// * `frame` - CAN frame containing radar target data
///
/// # Returns
///
/// Returns `Ok(Target)` on success, or `Err(Error)` if the frame
/// is malformed or contains invalid data.
///
/// # Examples
///
/// ```
/// use radarpub::can::{parse_can_frame, Target};
/// let frame = // ... create test frame
/// let target = parse_can_frame(&frame)?;
/// assert_eq!(target.range, 25.5);
/// ```
///
/// # Errors
///
/// This function will return an error if:
/// - CRC validation fails
/// - Frame header is invalid
/// - Data length is incorrect
pub fn parse_can_frame(frame: &CanFrame) -> Result<Target, Error> {
    // implementation
}
```

**Update README if:**

- User-visible behavior changes
- New CLI options added
- New features available

### 6. Run All Checks

```bash
# Format code
cargo fmt --all

# Run linter (zero warnings required)
cargo clippy --all-targets -- -D warnings

# Run all tests
cargo test --workspace

# Check documentation builds
cargo doc --no-deps

# Run security audit
cargo audit

# Check test coverage (optional but recommended)
cargo install cargo-llvm-cov
cargo llvm-cov --workspace --html
```

### 7. Submit Pull Request

**Before submitting:**

- [ ] All tests pass locally
- [ ] Code is formatted with `cargo fmt`
- [ ] No clippy warnings
- [ ] Documentation is updated
- [ ] Commit messages are clear
- [ ] Branch is rebased on latest `main`

**Pull Request Template:**

```markdown
## Summary
Brief description of what this PR does and why.

## Related Issues
Closes #123
Related to #456

## Changes
- Added XYZ feature
- Fixed ABC bug
- Updated documentation

## Testing
- [ ] Unit tests added/updated
- [ ] Integration tests pass
- [ ] Manual testing completed on:
  - [ ] x86_64 Linux
  - [ ] ARM64 (Maivin/Raivin)
  - [ ] Virtual CAN
  - [ ] Physical radar hardware

## Checklist
- [ ] Code follows project style guidelines
- [ ] Tests added for new functionality
- [ ] Documentation updated
- [ ] No secrets or credentials committed
- [ ] PR title follows format: `<type>: Brief description`

## Screenshots (if applicable)
<!-- Add screenshots for UI changes or visual output -->
```

**Submit via GitHub:**

```bash
# Push your branch
git push origin feature/your-feature-name

# Create PR via GitHub web interface
```

## Code Style Guidelines

### Rust-Specific

- **Follow Rust 2021 edition idioms**
- **Use `cargo fmt`** - Enforced in CI, uses project's `rustfmt.toml`
- **Zero clippy warnings** - CI fails on any warnings
- **Prefer owned types in public APIs** - Avoid lifetime complexity for consumers
- **Use `Result<T, Error>` for fallible operations** - Never panic in library code
- **Document all public items** - Modules, structs, functions, traits

### Naming Conventions

- **Types**: `PascalCase` (e.g., `RadarTarget`, `CanFrame`)
- **Functions/methods**: `snake_case` (e.g., `parse_can_frame`, `read_radar_cube`)
- **Constants**: `SCREAMING_SNAKE_CASE` (e.g., `SMS_PACKET_SIZE`, `MAX_TARGETS`)
- **Modules**: `snake_case` (e.g., `clustering`, `can`, `eth`)

### Error Handling

```rust
// Use custom error types with thiserror
#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("Invalid CAN frame header: {0}")]
    InvalidHeader(String),

    #[error("CRC validation failed")]
    CrcMismatch,

    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),
}

// Return Result for fallible operations
pub fn risky_operation() -> Result<Data, Error> {
    // ...
}
```

### Performance Considerations

This is real-time sensor processing code. Consider:

- **Minimize allocations** in hot paths
- **Reuse buffers** where possible
- **Profile with Tracy** for latency-critical code
- **Avoid blocking** in async contexts
- **Use zero-copy** parsing when feasible

## Testing Requirements

### Coverage Standards

- **Minimum**: 70% line coverage for the project
- **Critical paths**: 90%+ coverage for CAN parsing, clustering, tracking
- **All public APIs**: Must have tests demonstrating usage

### Test Organization

```text
radarpub/
├── src/
│   └── can.rs                    # Unit tests at end of file
└── tests/
    ├── fixtures/                 # Test data
    │   ├── can_trace.txt
    │   └── radar_capture.pcap
    ├── common/                   # Shared test utilities
    │   └── mod.rs
    └── integration_test.rs       # Integration tests
```

### Running Tests

```bash
# All tests
cargo test --workspace

# Specific test
cargo test test_parse_can_frame

# With logging output
cargo test -- --nocapture

# Integration tests only
cargo test --test integration_test

# With coverage
cargo llvm-cov --workspace --html
open target/llvm-cov/html/index.html
```

## Documentation Guidelines

### Rustdoc Requirements

All public items must have documentation:

- **Modules** - Purpose and usage overview
- **Structs/Enums** - What they represent
- **Functions** - What they do, arguments, returns, errors, examples
- **Public fields** - Meaning and constraints

### Building Documentation

```bash
# Generate documentation
cargo doc --no-deps --open

# Check for warnings (enforced in CI)
RUSTDOCFLAGS="-D warnings" cargo doc --no-deps
```

### External Documentation

User guides and tutorials are maintained separately at https://doc.edgefirst.ai/perception/radarpub/

## Review Process

### What Reviewers Look For

1. **Correctness**: Does it work? Are edge cases handled?
2. **Testing**: Adequate test coverage? Tests pass?
3. **Code quality**: Readable? Follows style guidelines?
4. **Documentation**: Public APIs documented? Changes explained?
5. **Performance**: Any regression concerns?
6. **Breaking changes**: Properly marked and justified?

### Approval Requirements

- **1 approval required** for feature branches
- **2 approvals required** for merging to `main`
- All CI checks must pass (build, test, lint, audit)

### Addressing Feedback

- Respond to all review comments
- Push additional commits to address feedback
- Mark conversations as resolved once addressed
- Request re-review when ready

## Release Process

Releases are managed by project maintainers:

1. Version bump in `Cargo.toml`
2. Update `CHANGELOG.md`
3. Create annotated git tag: `git tag -a v0.2.0 -m "Release v0.2.0"`
4. Push tag: `git push origin v0.2.0`
5. GitHub Actions automatically builds and publishes release artifacts

## Getting Help

### Development Questions

- **GitHub Discussions**: https://github.com/EdgeFirstAI/radarpub/discussions
- **Architecture questions**: See [ARCHITECTURE.md](ARCHITECTURE.md)
- **EdgeFirst integration**: https://doc.edgefirst.ai/perception/

### Report Issues

- **Bugs**: Use [bug report template](https://github.com/EdgeFirstAI/radarpub/issues/new?template=bug_report.md)
- **Features**: Use [feature request template](https://github.com/EdgeFirstAI/radarpub/issues/new?template=feature_request.md)

### Contact

- **Community support**: GitHub Discussions
- **Security issues**: support@au-zone.com with subject "Security Vulnerability - RadarPub" (see [SECURITY.md](SECURITY.md))
- **Code of Conduct violations**: support@au-zone.com with subject "Code of Conduct Violation" (see [CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md))
- **Commercial inquiries**: support@au-zone.com

## License

By contributing to RadarPub, you agree that your contributions will be licensed under the Apache License 2.0. No contributor license agreement (CLA) is required.

See [LICENSE](LICENSE) for the full license text.

---

**Thank you for contributing to RadarPub and the EdgeFirst Perception ecosystem!**
