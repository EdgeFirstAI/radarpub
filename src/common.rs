// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.

use edgefirst_schemas::builtin_interfaces::Time;
use std::{fmt, net::UdpSocket};
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

/// Converts a Unix timestamp in microseconds to `builtin_interfaces::Time`.
pub fn stamp_from_micros(us: u64) -> Time {
    Time {
        sec: (us / 1_000_000) as i32,
        nanosec: ((us % 1_000_000) * 1_000) as u32,
    }
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
/// # Arguments
/// * `socket` - UDP socket to configure
/// * `size` - Buffer size in bytes
///
/// # Returns
/// Configured socket
#[cfg(target_os = "linux")]
pub fn set_socket_bufsize(socket: UdpSocket, size: usize) -> UdpSocket {
    use std::os::fd::{FromRawFd, IntoRawFd};

    let fd = socket.into_raw_fd();
    let size = size as libc::c_int;
    let err = unsafe {
        libc::setsockopt(
            fd,
            libc::SOL_SOCKET,
            libc::SO_RCVBUF,
            &size as *const _ as *const libc::c_void,
            std::mem::size_of_val(&size) as libc::socklen_t,
        )
    };
    if err != 0 {
        warn!(
            "setsockopt SO_RCVBUF failed: {}",
            std::io::Error::last_os_error()
        );
    }

    unsafe { UdpSocket::from_raw_fd(fd) }
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
    fn test_stamp_from_micros() {
        let stamp = stamp_from_micros(1_700_000_000_123_456);
        assert_eq!(stamp.sec, 1_700_000_000);
        assert_eq!(stamp.nanosec, 123_456_000);
    }
}
