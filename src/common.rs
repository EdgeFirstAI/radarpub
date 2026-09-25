// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.

use edgefirst_schemas::builtin_interfaces::Time;
use std::{
    fmt,
    net::UdpSocket,
    time::{Duration, SystemTime, UNIX_EPOCH},
};
use tracing::warn;

/// Errors from wall-clock timestamp helpers.
#[derive(Debug)]
pub enum TimestampError {
    /// System clock is before the Unix epoch.
    SystemTime(std::time::SystemTimeError),
    /// System clock seconds exceed i32 range (Y2038).
    TimestampOverflow,
}

impl fmt::Display for TimestampError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            TimestampError::SystemTime(err) => {
                write!(f, "system clock before Unix epoch: {}", err)
            }
            TimestampError::TimestampOverflow => {
                write!(f, "system clock seconds exceed i32 range")
            }
        }
    }
}

impl std::error::Error for TimestampError {}

impl From<std::time::SystemTimeError> for TimestampError {
    fn from(err: std::time::SystemTimeError) -> Self {
        TimestampError::SystemTime(err)
    }
}

/// Gets the current wall-clock timestamp in nanoseconds.
///
/// Uses `SystemTime` (backed by `CLOCK_REALTIME` on Linux) for ROS 2
/// compatible Header stamps. Wall-clock time is required across all
/// EdgeFirst services to enable temporal synchronization in downstream
/// consumers (fusion, webui, rosbag correlation).
///
/// Returns an error if the system clock is before the Unix epoch or
/// if seconds exceed the `i32` range used by `builtin_interfaces::Time`
/// (Y2038 overflow).
pub fn timestamp() -> Result<u64, TimestampError> {
    let duration = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_err(TimestampError::SystemTime)?;

    let secs = duration.as_secs();
    if secs > i32::MAX as u64 {
        return Err(TimestampError::TimestampOverflow);
    }

    Ok(duration.as_nanos() as u64)
}

/// Converts a wall-clock instant to `builtin_interfaces::Time`.
///
/// Returns [`TimestampError::SystemTime`] if the instant is before the Unix
/// epoch and [`TimestampError::TimestampOverflow`] if seconds exceed the
/// `i32` range used by ROS 2 `Time` (Y2038).
pub fn stamp_from_system_time(time: SystemTime) -> Result<Time, TimestampError> {
    let duration = time.duration_since(UNIX_EPOCH)?;
    if duration.as_secs() > i32::MAX as u64 {
        return Err(TimestampError::TimestampOverflow);
    }

    Ok(Time {
        sec: duration.as_secs() as i32,
        nanosec: duration.subsec_nanos(),
    })
}

/// Estimates the acquisition time of a measurement from its host receive
/// time.
///
/// `rx_time` is the `CLOCK_REALTIME` instant at which the first message of
/// the measurement was received (kernel `SO_TIMESTAMPNS` or a clock read
/// immediately after the receive call), and `latency` is the sensor's
/// processing latency between acquisition and transmission. The DRVEGRD has
/// no clock synchronized to the host, so this is the best available
/// estimate of the acquisition instant.
///
/// # Errors
///
/// Same as [`stamp_from_system_time`]; a latency larger than the time since
/// the epoch yields [`TimestampError::SystemTime`].
pub fn acquisition_stamp(rx_time: SystemTime, latency: Duration) -> Result<Time, TimestampError> {
    let time = rx_time.checked_sub(latency).unwrap_or(UNIX_EPOCH);
    stamp_from_system_time(time)
}

/// Set real-time FIFO scheduler priority for current thread.
///
/// Configures SCHED_FIFO with priority 10 on Linux for low-latency processing.
/// No-op on non-Linux platforms.
#[cfg(target_os = "linux")]
pub fn set_process_priority() {
    let mut param = libc::sched_param { sched_priority: 10 };
    let pid = unsafe { libc::pthread_self() };
    let err = unsafe {
        libc::pthread_setschedparam(pid, libc::SCHED_FIFO, &mut param as *mut libc::sched_param)
    };
    if err != 0 {
        let err = std::io::Error::last_os_error();
        warn!("unable to set udp_read real-time fifo scheduler: {}", err);
    }
}

#[cfg(not(target_os = "linux"))]
pub fn set_process_priority() {}

/// Configure UDP socket receive buffer size.
///
/// Uses `SO_RCVBUFFORCE`, which ignores the `net.core.rmem_max` limit but
/// requires `CAP_NET_ADMIN`, and falls back to `SO_RCVBUF`, which the kernel
/// silently caps at `net.core.rmem_max`. Logs a warning when the granted
/// buffer is smaller than requested; the radar cube stream drops packets
/// with the default 208 KiB limit.
///
/// # Arguments
/// * `socket` - UDP socket to configure
/// * `size` - Buffer size in bytes
///
/// # Returns
/// Configured socket
#[cfg(target_os = "linux")]
pub fn set_socket_bufsize(socket: UdpSocket, size: usize) -> UdpSocket {
    use std::os::fd::AsRawFd;

    let fd = socket.as_raw_fd();
    let requested = size as libc::c_int;
    let set = |option: libc::c_int| -> std::io::Result<()> {
        let err = unsafe {
            libc::setsockopt(
                fd,
                libc::SOL_SOCKET,
                option,
                &requested as *const _ as *const libc::c_void,
                std::mem::size_of_val(&requested) as libc::socklen_t,
            )
        };
        match err {
            0 => Ok(()),
            _ => Err(std::io::Error::last_os_error()),
        }
    };

    if let Err(force_err) = set(libc::SO_RCVBUFFORCE) {
        if let Err(err) = set(libc::SO_RCVBUF) {
            warn!(
                "setsockopt SO_RCVBUFFORCE failed: {}, SO_RCVBUF failed: {}",
                force_err, err
            );
        }
    }

    // The kernel reports double the usable size to account for bookkeeping.
    let mut granted: libc::c_int = 0;
    let mut len = std::mem::size_of_val(&granted) as libc::socklen_t;
    let err = unsafe {
        libc::getsockopt(
            fd,
            libc::SOL_SOCKET,
            libc::SO_RCVBUF,
            &mut granted as *mut _ as *mut libc::c_void,
            &mut len,
        )
    };
    if err == 0 && granted / 2 < requested {
        warn!(
            "UDP receive buffer is {} bytes, requested {}; run with CAP_NET_ADMIN or raise net.core.rmem_max",
            granted / 2,
            requested
        );
    }

    socket
}

#[cfg(not(target_os = "linux"))]
pub fn set_socket_bufsize(socket: UdpSocket, _size: usize) -> UdpSocket {
    socket
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_timestamp_matches_system_time() {
        use std::time::{SystemTime, UNIX_EPOCH};

        let before = SystemTime::now().duration_since(UNIX_EPOCH).unwrap();
        let ns = timestamp().unwrap();
        let after = SystemTime::now().duration_since(UNIX_EPOCH).unwrap();

        assert!(ns >= before.as_nanos() as u64);
        assert!(ns <= after.as_nanos() as u64);
    }

    #[test]
    fn test_timestamp_conversion_invariants() {
        let ns = timestamp().unwrap();

        let secs = ns / 1_000_000_000;
        assert!(secs <= i32::MAX as u64, "timestamp exceeds i32::MAX");

        let subsec_ns = ns % 1_000_000_000;
        assert!(subsec_ns < 1_000_000_000, "nanoseconds out of range");
    }

    #[test]
    fn test_timestamp_error_display() {
        let err = TimestampError::TimestampOverflow;
        assert_eq!(err.to_string(), "system clock seconds exceed i32 range");
    }

    #[test]
    fn test_stamp_from_system_time() {
        let time = UNIX_EPOCH + Duration::new(1_700_000_000, 123_456_789);
        let stamp = stamp_from_system_time(time).unwrap();
        assert_eq!(stamp.sec, 1_700_000_000);
        assert_eq!(stamp.nanosec, 123_456_789);
    }

    #[test]
    fn test_stamp_from_system_time_overflow() {
        let time = UNIX_EPOCH + Duration::from_secs(i32::MAX as u64 + 1);
        assert!(matches!(
            stamp_from_system_time(time),
            Err(TimestampError::TimestampOverflow)
        ));
    }

    #[test]
    fn test_stamp_from_system_time_pre_epoch() {
        let time = UNIX_EPOCH - Duration::from_secs(1);
        assert!(matches!(
            stamp_from_system_time(time),
            Err(TimestampError::SystemTime(_))
        ));
    }

    #[test]
    fn test_acquisition_stamp_subtracts_latency() {
        let rx = UNIX_EPOCH + Duration::new(1_700_000_000, 50_000_000);
        let stamp = acquisition_stamp(rx, Duration::from_nanos(110_000_000)).unwrap();
        assert_eq!(stamp.sec, 1_699_999_999);
        assert_eq!(stamp.nanosec, 940_000_000);
    }

    #[test]
    fn test_acquisition_stamp_zero_latency() {
        let rx = UNIX_EPOCH + Duration::new(1_700_000_000, 1);
        let stamp = acquisition_stamp(rx, Duration::ZERO).unwrap();
        assert_eq!((stamp.sec, stamp.nanosec), (1_700_000_000, 1));
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn test_set_socket_bufsize_grants_at_least_default() {
        use std::os::fd::AsRawFd;

        let socket = set_socket_bufsize(UdpSocket::bind("127.0.0.1:0").unwrap(), 64 * 1024);
        let mut granted: libc::c_int = 0;
        let mut len = std::mem::size_of_val(&granted) as libc::socklen_t;
        let err = unsafe {
            libc::getsockopt(
                socket.as_raw_fd(),
                libc::SOL_SOCKET,
                libc::SO_RCVBUF,
                &mut granted as *mut _ as *mut libc::c_void,
                &mut len,
            )
        };
        assert_eq!(err, 0);
        assert!(granted / 2 >= 64 * 1024, "granted {granted}");
    }

    #[test]
    fn test_acquisition_stamp_latency_before_epoch() {
        let rx = UNIX_EPOCH + Duration::from_millis(10);
        assert!(acquisition_stamp(rx, Duration::from_millis(110)).is_err());
    }
}
