// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.

use crate::eth::SMS_PACKET_SIZE;
use kanal::AsyncSender;
use std::time::SystemTime;
use tokio::net::UdpSocket;
use tracing::error;

/// A batch of SMS datagrams with the host receive time of each.
pub struct Datagrams {
    /// Datagrams stored back to back, each in a slot of `SMS_PACKET_SIZE`
    /// bytes.
    pub data: Vec<u8>,
    /// Host receive time (`CLOCK_REALTIME`) of each datagram, from the kernel
    /// `SO_TIMESTAMPNS` control message when available, else read right after
    /// the receive call.
    pub rx_time: Vec<SystemTime>,
}

impl Datagrams {
    /// Iterates over the datagrams as `(receive time, datagram slot)`.
    pub fn iter(&self) -> impl Iterator<Item = (SystemTime, &[u8])> {
        self.rx_time
            .iter()
            .copied()
            .zip(self.data.as_chunks::<SMS_PACKET_SIZE>().0)
            .map(|(rx_time, packet)| (rx_time, packet.as_slice()))
    }
}

/// UDP receiver for radar cube data on port 50005.
///
/// Runs with real-time FIFO priority and forwards batches of SMS protocol
/// packets to the processing channel.
///
/// # Arguments
/// * `tx` - Async channel sender for received packets
pub async fn port5(tx: AsyncSender<Datagrams>) {
    crate::common::set_process_priority();
    receive(50005, Wait::Poll, tx).await
}

/// UDP receiver for radar cube data on port 50063.
///
/// Receives Smart Micro SMS protocol packets and forwards to processing
/// channel.
///
/// # Arguments
/// * `tx` - Async channel sender for received packets
pub async fn port63(tx: AsyncSender<Datagrams>) {
    receive(50063, Wait::Ready, tx).await
}

/// How a receiver waits when its socket is empty.
#[derive(Clone, Copy)]
#[cfg_attr(not(target_os = "linux"), allow(dead_code))]
enum Wait {
    /// Sleep briefly so datagrams accumulate and each recvmmsg call returns a
    /// large batch. Used for the cube stream (about 17k packets/s), where
    /// waking on every packet costs enough CPU to overflow the socket buffer.
    Poll,
    /// Wake on socket readiness, for low-rate streams.
    Ready,
}

/// The Linux receiver uses the recvmmsg system call to enable bulk reads of
/// UDP packets, with a kernel receive timestamp for each packet.
#[cfg(target_os = "linux")]
async fn receive(port: u16, wait: Wait, tx: AsyncSender<Datagrams>) {
    use crate::common::set_socket_bufsize;
    use std::{io::ErrorKind, os::fd::AsRawFd, thread, time::Duration};
    use tokio::io::Interest;
    use tracing::warn;

    const POLL_TIME: Duration = Duration::from_micros(250);

    let sock = UdpSocket::bind(("0.0.0.0", port)).await.unwrap();
    let sock = set_socket_bufsize(sock.into_std().unwrap(), 2 * 1024 * 1024);
    let sock = UdpSocket::from_std(sock).unwrap();

    if let Err(err) = linux::enable_rx_timestamps(sock.as_raw_fd()) {
        warn!(
            "port {}: kernel receive timestamps unavailable, using the clock after each receive: {}",
            port, err
        );
    }

    let mut receiver = linux::MmsgReceiver::new();

    loop {
        let received = match wait {
            Wait::Poll => receiver.recv(sock.as_raw_fd()),
            Wait::Ready => {
                sock.async_io(Interest::READABLE, || receiver.recv(sock.as_raw_fd()))
                    .await
            }
        };

        match received {
            Ok(datagrams) => {
                if let Err(e) = tx.send(datagrams).await {
                    error!("port {} error: {:?}", port, e);
                }
            }
            Err(err) if err.kind() == ErrorKind::WouldBlock => thread::sleep(POLL_TIME),
            Err(err) if err.kind() == ErrorKind::Interrupted => (),
            Err(err) => error!("port {} error: {:?}", port, err),
        }
    }
}

#[cfg(not(target_os = "linux"))]
async fn receive(port: u16, _wait: Wait, tx: AsyncSender<Datagrams>) {
    let sock = UdpSocket::bind(("0.0.0.0", port)).await.unwrap();
    let mut buf = vec![0; SMS_PACKET_SIZE];

    loop {
        match sock.recv_from(&mut buf).await {
            Ok(_) => {
                let datagrams = Datagrams {
                    data: buf.clone(),
                    rx_time: vec![SystemTime::now()],
                };
                if let Err(e) = tx.send(datagrams).await {
                    error!("port {} write error: {:?}", port, e);
                }
            }
            Err(e) => error!("port {} read error: {:?}", port, e),
        }
    }
}

#[cfg(target_os = "linux")]
mod linux {
    use super::{Datagrams, SMS_PACKET_SIZE};
    use std::{
        io, mem,
        os::fd::RawFd,
        time::{Duration, SystemTime, UNIX_EPOCH},
    };

    /// Maximum number of datagrams read by one recvmmsg call.
    const VLEN: usize = 64;

    /// Control message buffer for one datagram, large enough for an
    /// `SCM_TIMESTAMPNS` message. The `u64` elements keep the buffer aligned
    /// for `cmsghdr`.
    type CmsgBuf = [u64; 8];

    /// Enables kernel receive timestamps (`SO_TIMESTAMPNS`) on a socket.
    ///
    /// The kernel stamps each datagram with `CLOCK_REALTIME` when it enters
    /// the network stack, so the receive time does not depend on when the
    /// application reads the socket. The kernel turns timestamping on through
    /// deferred work, so datagrams received in the first moments after this
    /// call may be stamped when read instead.
    pub fn enable_rx_timestamps(fd: RawFd) -> io::Result<()> {
        let enable: libc::c_int = 1;
        let err = unsafe {
            libc::setsockopt(
                fd,
                libc::SOL_SOCKET,
                libc::SO_TIMESTAMPNS,
                &enable as *const _ as *const libc::c_void,
                mem::size_of_val(&enable) as libc::socklen_t,
            )
        };
        match err {
            0 => Ok(()),
            _ => Err(io::Error::last_os_error()),
        }
    }

    /// Reusable buffers for reading batches of datagrams with recvmmsg.
    pub struct MmsgReceiver {
        mmsgs: Vec<libc::mmsghdr>,
        iovecs: Vec<libc::iovec>,
        cmsgs: Vec<CmsgBuf>,
        buf: Vec<u8>,
    }

    impl MmsgReceiver {
        pub fn new() -> Self {
            MmsgReceiver {
                // SAFETY: mmsghdr and iovec are plain C structs for which all
                // zero bytes (null pointers, zero lengths) is a valid value.
                mmsgs: vec![unsafe { mem::zeroed() }; VLEN],
                iovecs: vec![unsafe { mem::zeroed() }; VLEN],
                cmsgs: vec![[0; 8]; VLEN],
                buf: vec![0; VLEN * SMS_PACKET_SIZE],
            }
        }

        /// Reads up to `VLEN` datagrams without blocking.
        ///
        /// Each datagram's receive time is its `SCM_TIMESTAMPNS` control
        /// message, or the clock read right after recvmmsg returns when the
        /// kernel did not provide one.
        ///
        /// # Errors
        ///
        /// Returns the recvmmsg error, including `WouldBlock` when no
        /// datagram is queued.
        pub fn recv(&mut self, fd: RawFd) -> io::Result<Datagrams> {
            for i in 0..VLEN {
                self.iovecs[i].iov_base =
                    self.buf[i * SMS_PACKET_SIZE..].as_mut_ptr() as *mut libc::c_void;
                self.iovecs[i].iov_len = SMS_PACKET_SIZE;
                // SAFETY: see new().
                self.mmsgs[i] = unsafe { mem::zeroed() };
                let hdr = &mut self.mmsgs[i].msg_hdr;
                hdr.msg_iov = &mut self.iovecs[i];
                hdr.msg_iovlen = 1;
                hdr.msg_control = self.cmsgs[i].as_mut_ptr() as *mut libc::c_void;
                hdr.msg_controllen = mem::size_of::<CmsgBuf>() as _;
            }

            let n = unsafe {
                libc::recvmmsg(
                    fd,
                    self.mmsgs.as_mut_ptr(),
                    VLEN as u32,
                    libc::MSG_DONTWAIT,
                    std::ptr::null_mut(),
                )
            };
            let now = SystemTime::now();
            if n < 0 {
                return Err(io::Error::last_os_error());
            }
            let n = n as usize;

            let rx_time = self.mmsgs[..n]
                .iter()
                // SAFETY: the kernel filled in these headers and their control
                // buffers, which are still alive in self.cmsgs.
                .map(|mmsg| unsafe { cmsg_timestamp(&mmsg.msg_hdr) }.unwrap_or(now))
                .collect();

            Ok(Datagrams {
                data: self.buf[..n * SMS_PACKET_SIZE].to_vec(),
                rx_time,
            })
        }
    }

    /// Returns the `SCM_TIMESTAMPNS` receive time of a received message.
    ///
    /// # Safety
    ///
    /// `hdr` must describe a message filled in by recvmsg or recvmmsg whose
    /// control buffer is still valid.
    unsafe fn cmsg_timestamp(hdr: &libc::msghdr) -> Option<SystemTime> {
        let mut cmsg = libc::CMSG_FIRSTHDR(hdr);
        while !cmsg.is_null() {
            if (*cmsg).cmsg_level == libc::SOL_SOCKET && (*cmsg).cmsg_type == libc::SCM_TIMESTAMPNS
            {
                let ts = std::ptr::read_unaligned(libc::CMSG_DATA(cmsg) as *const libc::timespec);
                return timespec_to_system_time(&ts);
            }
            cmsg = libc::CMSG_NXTHDR(hdr, cmsg);
        }
        None
    }

    fn timespec_to_system_time(ts: &libc::timespec) -> Option<SystemTime> {
        let secs = u64::try_from(ts.tv_sec).ok()?;
        let nanos = u32::try_from(ts.tv_nsec).ok()?;
        UNIX_EPOCH.checked_add(Duration::new(secs, nanos))
    }

    #[cfg(test)]
    mod tests {
        use super::*;
        use std::{net::UdpSocket, os::fd::AsRawFd, thread};

        #[test]
        fn test_recv_uses_kernel_timestamps() {
            let rx = UdpSocket::bind("127.0.0.1:0").unwrap();
            rx.set_nonblocking(true).unwrap();
            enable_rx_timestamps(rx.as_raw_fd()).unwrap();
            let tx = UdpSocket::bind("127.0.0.1:0").unwrap();

            // The kernel enables receive timestamping through deferred work;
            // until it runs, datagrams are stamped when read instead.
            thread::sleep(Duration::from_millis(50));

            let before = SystemTime::now();
            for i in 0..3u8 {
                tx.send_to(&[i; 16], rx.local_addr().unwrap()).unwrap();
            }
            let sent = SystemTime::now();

            // Delay the read so a clock read after recvmmsg would land well
            // after `sent`; only a kernel timestamp can fall before it.
            thread::sleep(Duration::from_millis(50));

            let mut receiver = MmsgReceiver::new();
            let datagrams = receiver.recv(rx.as_raw_fd()).unwrap();

            assert_eq!(datagrams.rx_time.len(), 3);
            for (i, (rx_time, slot)) in datagrams.iter().enumerate() {
                assert!(rx_time >= before && rx_time <= sent, "datagram {i}");
                assert_eq!(&slot[..16], &[i as u8; 16]);
            }
            assert!(datagrams.rx_time.is_sorted());
        }

        #[test]
        fn test_recv_would_block_when_empty() {
            let rx = UdpSocket::bind("127.0.0.1:0").unwrap();
            let mut receiver = MmsgReceiver::new();
            let err = receiver.recv(rx.as_raw_fd()).err().unwrap();
            assert_eq!(err.kind(), io::ErrorKind::WouldBlock);
        }

        #[test]
        fn test_timespec_to_system_time() {
            let ts = libc::timespec {
                tv_sec: 1_700_000_000,
                tv_nsec: 5,
            };
            assert_eq!(
                timespec_to_system_time(&ts),
                Some(UNIX_EPOCH + Duration::new(1_700_000_000, 5))
            );

            let ts = libc::timespec {
                tv_sec: -1,
                tv_nsec: 0,
            };
            assert_eq!(timespec_to_system_time(&ts), None);
        }
    }
}
