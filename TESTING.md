# Testing

This document describes the testing strategy for EdgeFirst Radar Publisher, including
automated CI testing and manual hardware testing procedures.

## Overview

EdgeFirst Radar Publisher uses a two-tier testing approach:

1. **Automated Testing (CI)**: Unit tests and static analysis on GitHub runners
2. **Manual Testing**: Hardware integration testing with SmartMicro DRVEGRD radar sensors

## Automated Testing (CI)

The GitHub Actions workflow (`.github/workflows/test.yml`) runs on every push and PR:

| Job | Description |
|-----|-------------|
| Format Check | Verifies code formatting with `cargo fmt` |
| Clippy Lint | Static analysis with `cargo clippy` |
| Unit Tests | Runs `cargo test` with coverage collection |
| Security Audit | Checks dependencies for known vulnerabilities |
| License Check | Verifies all dependencies have compatible licenses |

### Running Tests Locally

```bash
# Run all unit tests (requires can feature for full test suite)
cargo test --features can,zenoh

# Run tests with verbose output
cargo test --features can,zenoh -- --nocapture

# Run specific test module
cargo test --features can can::tests

# Check formatting
cargo fmt --check

# Run Clippy lints (all features)
cargo clippy --all-features -- -D warnings
```

### Virtual CAN for Testing

Some tests require a CAN interface. On Linux, you can use a virtual CAN:

```bash
# Load vcan module
sudo modprobe vcan

# Create virtual CAN interface
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

# Set environment variable for tests
export RADAR_CAN_INTERFACE=vcan0

# Run tests
cargo test --features can
```

## Manual Hardware Testing

Integration testing requires a SmartMicro DRVEGRD radar sensor connected via CAN or Ethernet.

### Prerequisites

- Linux system with CAN or Ethernet connectivity to the radar
- SmartMicro DRVEGRD-169 or DRVEGRD-171 radar sensor
- Rust stable toolchain
- For CAN: SocketCAN-compatible interface (e.g., PEAK PCAN-USB)
- For Ethernet: Network access to radar's IP address

### Building

```bash
# Build release binaries with CAN and Zenoh support
cargo build --release --features can,zenoh

# Build the control utility
cargo build --release --bin drvegrdctl --features can

# Build the Rerun visualization tool (optional)
cargo build --release --bin drvegrd-rerun --features rerun
```

### Configuring CAN Interface

```bash
# Set up physical CAN interface (1Mbps for DRVEGRD)
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0

# Verify interface is active
ip -details link show can0
```

### Running the Publisher (CAN Mode)

```bash
# Basic usage with CAN interface
./target/release/radarpub --can can0

# Enable clustering
./target/release/radarpub --can can0 --clustering

# Specify custom topic prefix
./target/release/radarpub --can can0 --prefix rt/radar

# Connect to specific Zenoh router
./target/release/radarpub --can can0 --connect tcp/192.168.1.1:7447
```

### Running the Publisher (Ethernet Mode)

```bash
# Connect to radar via Ethernet
./target/release/radarpub --eth 192.168.10.10

# With custom port (default is 55555)
./target/release/radarpub --eth 192.168.10.10 --eth-port 55555

# Enable radar cube streaming
./target/release/radarpub --eth 192.168.10.10 --radar-cube
```

### Using drvegrdctl Utility

The control utility can query and configure the radar:

```bash
# Query sensor information
./target/release/drvegrdctl --can can0 info

# Get current configuration
./target/release/drvegrdctl --can can0 config

# Set radar parameters
./target/release/drvegrdctl --can can0 set-range 50.0

# Reset sensor
./target/release/drvegrdctl --can can0 reset
```

### Verifying Output

Use Zenoh tools to verify messages are being published:

```bash
# Install zenoh tools if needed
cargo install zenoh

# Subscribe to target messages
z_sub -k "rt/radar/targets"

# Subscribe to cluster messages
z_sub -k "rt/radar/clusters"

# Subscribe to all radar topics
z_sub -k "rt/radar/**"
```

### Expected Console Output

When running correctly, you should see:

```
INFO zenoh::net::runtime: Using ZID: ...
INFO zenoh::net::runtime::orchestrator: Zenoh can be reached at: tcp/...
INFO radarpub: Starting radar publisher...
INFO radarpub: Connecting to CAN interface: can0
INFO radarpub: Radar initialized successfully
```

### Common Test Scenarios

#### 1. Basic CAN Connectivity Test

```bash
# Verify CAN messages are being received
candump can0 | head -20

# Run publisher and check for target output
timeout 30 ./target/release/radarpub --can can0
```

#### 2. Ethernet Connectivity Test

```bash
# Ping radar IP
ping 192.168.10.10

# Run publisher with Ethernet backend
timeout 30 ./target/release/radarpub --eth 192.168.10.10
```

#### 3. Target Detection Test

```bash
# Position a reflective object in front of radar
# Run with clustering disabled to see raw targets
./target/release/radarpub --can can0

# Verify targets appear in Zenoh
z_sub -k "rt/radar/targets"
```

#### 4. Clustering Performance

```bash
# Test with clustering enabled
./target/release/radarpub --can can0 --clustering

# Subscribe to cluster output
z_sub -k "rt/radar/clusters"
```

#### 5. Radar Cube Streaming (Ethernet only)

```bash
# Enable radar cube (range-Doppler data)
./target/release/radarpub --eth 192.168.10.10 --radar-cube

# Subscribe to radar cube messages
z_sub -k "rt/radar/cube"
```

## Troubleshooting

### No CAN Data Received

1. **Check CAN interface status:**
   ```bash
   ip -details link show can0
   # Should show state UP, bitrate 1000000
   ```

2. **Verify CAN messages on the bus:**
   ```bash
   candump can0
   # Should show frames from radar (IDs 0x300-0x3FF typically)
   ```

3. **Check CAN bus termination:**
   - Ensure 120Ω termination resistors are present
   - DRVEGRD has internal termination (check jumper/config)

4. **Verify correct bitrate:**
   ```bash
   # DRVEGRD uses 1Mbps CAN
   sudo ip link set can0 type can bitrate 1000000
   ```

### No Ethernet Data Received

1. **Check network connectivity:**
   ```bash
   ping 192.168.10.10
   ```

2. **Verify UDP port is open:**
   ```bash
   sudo ufw allow 55555/udp
   ```

3. **Check radar is configured for Ethernet output:**
   - Consult SmartMicro documentation for Ethernet configuration

### Performance Issues

1. **CAN buffer overflows:**
   ```bash
   # Increase socket buffer size
   sudo sysctl -w net.core.rmem_max=8388608
   ```

2. **Reduce processing load:**
   ```bash
   # Disable clustering if not needed
   ./target/release/radarpub --can can0
   # Don't use --clustering flag
   ```

### Zenoh Communication Issues

1. **Check Zenoh is discovering peers:**
   ```bash
   z_scout
   ```

2. **Disable multicast if network doesn't support it:**
   ```bash
   ./target/release/radarpub --can can0 --no-multicast-scouting --connect tcp/<router>:7447
   ```

## Unit Tests

The following test modules are available:

### CAN Protocol Tests (`can::tests`)

- `test_parse_targets`: Tests CAN message parsing for target data
- `test_request_crc`: Tests CRC calculation for requests

### Clustering Tests (`clustering::*::tests`)

- `clustering::tracker::tests::filter`: Tests object tracking
- `clustering::kalman::tests::filter`: Tests Kalman filter implementation
- `clustering::kalman::tests::gating`: Tests gating logic

Run with:
```bash
cargo test --features can clustering
cargo test --features can can::tests
```

## See Also

- [README.md](README.md) - Usage documentation and quick start
- [CONTRIBUTING.md](CONTRIBUTING.md) - Development guidelines
- [ARCHITECTURE.md](ARCHITECTURE.md) - System architecture
