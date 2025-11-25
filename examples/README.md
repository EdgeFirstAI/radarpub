# RadarPub Examples

This directory contains example applications demonstrating how to use RadarPub and integrate with the EdgeFirst Perception Middleware.

## Examples

### 1. `radar_viewer.rs` - Direct Radar Visualization

Connects directly to a Smart Micro DRVEGRD radar sensor via CAN and/or UDP and visualizes the data in Rerun.

**Features:**
- Real-time CAN target data visualization
- 4D radar cube tensor display
- PCAP file replay for offline analysis
- Numpy export for post-processing

**Usage:**
```bash
# Live radar with CAN and cube data
cargo run --example radar_viewer --features rerun -- --device can0 --cube --viewer

# Replay PCAP file
cargo run --example radar_viewer --features rerun -- radar_data.pcap --viewer

# Record to file
cargo run --example radar_viewer --features rerun -- --device can0 --record output.rrd
```

**Requirements:**
- Linux with SocketCAN (for live CAN)
- Smart Micro DRVEGRD radar sensor (for live data)
- Or PCAP file with recorded radar data

---

### 2. `zenoh_viewer.rs` - Zenoh Subscriber Visualization

Subscribes to RadarPub's Zenoh topics and visualizes the processed radar data in Rerun. This demonstrates integration with the EdgeFirst Perception Middleware.

**Features:**
- Subscribe to PointCloud2 topics (targets/clusters)
- Subscribe to RadarCube topic
- Subscribe to TF transforms
- Real-time visualization of processed data

**Usage:**
```bash
# Subscribe to local RadarPub instance
cargo run --example zenoh_viewer --features rerun -- --viewer

# Connect to remote Zenoh router
cargo run --example zenoh_viewer --features rerun -- --connect <router-ip> --viewer

# Record to file
cargo run --example zenoh_viewer --features rerun -- --record output.rrd
```

**Requirements:**
- Running RadarPub instance publishing to Zenoh
- Or Zenoh router with radar data

---

## Building Examples

Build all examples:
```bash
cargo build --examples --features rerun
```

Build specific example:
```bash
cargo build --example radar_viewer --features rerun
cargo build --example zenoh_viewer --features rerun
```

## Additional Resources

- [RadarPub README](../README.md) - Main project documentation
- [EdgeFirst Perception Docs](https://doc.edgefirst.ai/perception/) - Integration guide
- [Rerun Documentation](https://www.rerun.io/docs) - Visualization framework
- [Zenoh Documentation](https://zenoh.io/docs/) - Pub/sub middleware
