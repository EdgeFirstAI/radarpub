# RadarPub - EdgeFirst Radar Node

[![Build Status](https://github.com/EdgeFirstAI/radarpub/workflows/CI/badge.svg)](https://github.com/EdgeFirstAI/radarpub/actions)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![EdgeFirst](https://img.shields.io/badge/EdgeFirst-Perception-green)](https://doc.edgefirst.ai/perception/)

**Real-time radar sensor processing node for the EdgeFirst Perception Middleware**

RadarPub bridges automotive radar sensors to the EdgeFirst Perception stack, providing real-time target detection, clustering, tracking, and 4D radar cube publishing over Zenoh middleware. Designed for edge AI applications requiring low-latency sensor fusion on resource-constrained platforms.

## Features

### Core Capabilities

- **Smart Micro DRVEGRD Protocol Support** - Complete CAN and Ethernet/UDP protocol implementation
- **Real-Time Target Processing** - Low-latency processing from CAN reception to Zenoh publish
- **4D Radar Cube Publishing** - Full radar data tensor (range × azimuth × elevation × doppler)
- **Advanced Clustering** - DBSCAN spatial clustering for target grouping
- **Multi-Object Tracking** - ByteTrack algorithm with Kalman filtering for consistent track IDs
- **ROS2-Compatible Output** - PointCloud2 and TransformStamped message formats via Zenoh

### EdgeFirst Perception Integration

- **Seamless Zenoh Integration** - Publishes to EdgeFirst Perception topics for sensor fusion
- **Hardware-Optimized** - Validated on Maivin and Raivin edge AI platforms
- **Tracy Profiling Support** - Performance instrumentation for real-time analysis
- **Flexible Configuration** - Runtime parameter adjustment via CLI or control utility

### Supported Hardware

- **Radar Sensor**: Smart Micro DRVEGRD 169/174 (76-77 GHz automotive radar)
- **Platforms**: Maivin (NXP i.MX 8M Plus), Raivin (automotive-grade), Linux ARM64/x86_64
- **Interfaces**: CAN (500 kbps), Ethernet/UDP (radar cube data)

## Quick Start

### Prerequisites

- Linux system with SocketCAN support (kernel 2.6.25+)
- CAN interface hardware (or virtual CAN for testing)
- Rust toolchain 1.70+ (for building from source)

### Installation

**From Binary Release:**

```bash
# Download latest release for ARM64 (Maivin/Raivin)
wget https://github.com/EdgeFirstAI/radarpub/releases/latest/download/radarpub-aarch64
chmod +x radarpub-aarch64
sudo mv radarpub-aarch64 /usr/local/bin/radarpub
```

**From Source:**

```bash
git clone https://github.com/EdgeFirstAI/radarpub.git
cd radarpub
cargo build --release --features "can,zenoh"
sudo cp target/release/radarpub /usr/local/bin/
```

**Cross-Compile for ARM64:**

```bash
# Install cross-compilation tool
cargo install cross

# Build for ARM64
cross build --target aarch64-unknown-linux-gnu --release
```

### Basic Usage

**1. Set up CAN interface (if not already configured):**

```bash
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

**2. Run RadarPub with default settings:**

```bash
radarpub --can-interface can0 --zenoh-mode peer
```

**3. View published topics:**

```bash
# Using Zenoh bridge or subscriber
z_sub -t "/rt/radar/**"
```

### Published Zenoh Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/rt/radar/targets` | sensor_msgs/PointCloud2 | Raw target detections (x, y, z, speed, power, rcs) |
| `/rt/radar/clusters` | sensor_msgs/PointCloud2 | Clustered targets with tracking IDs |
| `/rt/radar/cube` | edgefirst_msgs/RadarCube | Full 4D radar data cube (complex i16) |
| `/rt/tf_static` | geometry_msgs/TransformStamped | Radar sensor frame transform |
| `/rt/radar/info` | edgefirst_msgs/RadarInfo | Radar configuration and parameters |

### Performance Characteristics

RadarPub is optimized for real-time sensor processing on resource-constrained edge platforms. The system is designed to handle:

- Multiple radar targets per frame at typical sensor frame rates (10 Hz)
- Concurrent CAN and UDP data streams
- Optional DBSCAN clustering and ByteTrack tracking
- Radar cube tensor processing

Actual performance depends on hardware configuration, sensor settings, and enabled features. For deployment planning and system integration, contact support@au-zone.com.

For architecture details and tuning guidelines, see [ARCHITECTURE.md](ARCHITECTURE.md).

### Configuration Options

```bash
# Basic configuration
radarpub \
  --can-interface can0 \
  --zenoh-mode peer \
  --log-level info

# Enable clustering and tracking
radarpub \
  --can-interface can0 \
  --enable-clustering \
  --cluster-epsilon 0.5 \
  --cluster-min-points 3

# Adjust radar parameters (requires drvegrdctl)
drvegrdctl --can-interface can0 set-frequency 76.5
drvegrdctl --can-interface can0 set-sensitivity high
```

For complete configuration options, see the [User Guide](https://doc.edgefirst.ai/perception/radarpub/).

## Examples

RadarPub includes comprehensive examples demonstrating different integration patterns:

### 1. Direct Radar Viewer (`radar_viewer`)

Connect directly to a radar sensor and visualize data with Rerun:

```bash
# Live radar with CAN and cube data
cargo run --example radar_viewer --features rerun -- --device can0 --cube --viewer

# Replay PCAP file for analysis
cargo run --example radar_viewer --features rerun -- radar_capture.pcap --viewer

# Record visualization to file
cargo run --example radar_viewer --features rerun -- --device can0 --record output.rrd
```

**Use cases:**
- Hardware validation and debugging
- Offline analysis of recorded data
- Direct sensor integration without middleware

### 2. Zenoh Subscriber Viewer (`zenoh_viewer`)

Subscribe to RadarPub's Zenoh topics and visualize processed data:

```bash
# Subscribe to local RadarPub instance
cargo run --example zenoh_viewer --features rerun -- --viewer

# Connect to specific topics
cargo run --example zenoh_viewer --features rerun -- --targets --clusters --viewer

# Connect to remote Zenoh router
cargo run --example zenoh_viewer --features rerun -- \
  --zenoh-mode client \
  --zenoh-router tcp/192.168.1.100:7447 \
  --viewer
```

**Use cases:**
- EdgeFirst Perception pipeline integration
- Multi-node distributed systems
- Sensor fusion visualization

See [examples/README.md](examples/README.md) for complete documentation and additional examples.

## EdgeFirst Ecosystem

RadarPub is a core component of the [EdgeFirst Perception Middleware](https://doc.edgefirst.ai/perception/), providing radar sensor integration for autonomous systems and robotics applications.

### Integration with EdgeFirst Suite

- **[EdgeFirst Perception](https://doc.edgefirst.ai/perception/)** - Multi-sensor fusion middleware
  - Combine radar data with camera, LiDAR, and IMU sensors
  - Unified coordinate frame transformations
  - Real-time sensor synchronization

- **[EdgeFirst Studio](https://edgefirst.studio)** - MLOps Platform
  - Deploy and manage perception pipelines at scale
  - Monitor sensor health and performance
  - A/B testing and gradual rollouts
  - Free tier available for development

- **[EdgeFirst Modules](https://www.edgefirst.ai/edgefirstmodules)** - Hardware Platforms
  - Maivin: Edge AI development platform (NXP i.MX 8M Plus)
  - Raivin: Automotive-grade edge AI platform
  - Custom hardware design services

## Documentation

- 📖 **User Guide**: [https://doc.edgefirst.ai/perception/radarpub/](https://doc.edgefirst.ai/perception/radarpub/)
- 🔧 **API Reference**: [https://docs.rs/radarpub/](https://docs.rs/radarpub/)
- 🏗️ **Architecture**: [ARCHITECTURE.md](ARCHITECTURE.md) (internal design documentation)
- 🚀 **EdgeFirst Perception**: [https://doc.edgefirst.ai/perception/](https://doc.edgefirst.ai/perception/)

## Support

### Community Resources

- 📚 **Documentation**: [https://doc.edgefirst.ai/perception/radarpub/](https://doc.edgefirst.ai/perception/radarpub/)
- 💬 **GitHub Discussions**: [Ask questions and share ideas](https://github.com/EdgeFirstAI/radarpub/discussions)
- 🐛 **Issue Tracker**: [Report bugs and request features](https://github.com/EdgeFirstAI/radarpub/issues)

### Commercial Support & Services

For production deployments and enterprise requirements, Au-Zone Technologies offers:

- **Training & Workshops** - Accelerate your team's expertise with EdgeFirst Perception
- **Custom Development** - Extend RadarPub or integrate additional radar sensors
- **Integration Services** - Seamless integration with your existing autonomy stack
- **Enterprise Support** - SLAs, priority fixes, and dedicated engineering support
- **Hardware Services** - Custom carrier boards and platform optimization

📧 **Contact**: support@au-zone.com | 🌐 **Learn more**: [au-zone.com](https://au-zone.com)

## Contributing

We welcome contributions from the community! Please see [CONTRIBUTING.md](CONTRIBUTING.md) for:

- Development setup and build instructions
- Code style guidelines and testing requirements
- Pull request process and review guidelines

This project follows our [Code of Conduct](CODE_OF_CONDUCT.md). By participating, you agree to uphold this code.

## Security

For security vulnerabilities, please see [SECURITY.md](SECURITY.md) or email **support@au-zone.com** with subject "Security Vulnerability - RadarPub".

We take security seriously and aim to respond to reports within 48 hours.

## License

Licensed under the Apache License, Version 2.0. See [LICENSE](LICENSE) for details.

Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.

Third-party dependencies and attributions are documented in [NOTICE.md](NOTICE.md).

## Acknowledgments

- **EdgeFirst Perception Team** - For middleware architecture and integration support
- **Smart Micro** - For DRVEGRD radar protocol documentation
- **Zenoh Project** - For exceptional real-time middleware
- **Community Contributors** - See [CONTRIBUTORS.md](CONTRIBUTORS.md)
- **Open Source Projects** - See [NOTICE.md](NOTICE.md) for complete attribution

---

**Built with ❤️ by Au-Zone Technologies** | **Empowering Edge AI for Autonomous Systems**
