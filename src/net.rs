// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.

use crate::eth::SMS_PACKET_SIZE;
use kanal::AsyncSender;
use tokio::net::UdpSocket;
use tracing::error;

/// The port5 implementation on Linux uses the recvmmsg system call to enable
/// bulk reads of UDP packets.  This is not available on other platforms.
#[cfg(target_os = "linux")]
pub async fn port5(tx: AsyncSender<Vec<u8>>) {
    use std::{os::fd::AsRawFd, thread, time::Duration};

    use crate::common::{set_process_priority, set_socket_bufsize};

    const VLEN: usize = 64;
    const RETRY_TIME: Duration = Duration::from_micros(250);

    let mut mmsgs = vec![
        libc::mmsghdr {
            msg_hdr: libc::msghdr {
                msg_name: std::ptr::null_mut(),
                msg_namelen: 0,
                msg_iov: std::ptr::null_mut(),
                msg_iovlen: 0,
                msg_control: std::ptr::null_mut(),
                msg_controllen: 0,
                msg_flags: 0,
            },
            msg_len: 0,
        };
        VLEN
    ];
    let mut iovecs = vec![
        libc::iovec {
            iov_base: std::ptr::null_mut(),
            iov_len: 0,
        };
        VLEN
    ];
    let mut buf = vec![0; VLEN * SMS_PACKET_SIZE];

    set_process_priority();
    let sock = UdpSocket::bind("0.0.0.0:50005").await.unwrap();
    let sock = set_socket_bufsize(sock.into_std().unwrap(), 2 * 1024 * 1024);
    let sock = UdpSocket::from_std(sock).unwrap();

    loop {
        for i in 0..VLEN {
            iovecs[i].iov_base = buf[i * SMS_PACKET_SIZE..].as_mut_ptr() as *mut libc::c_void;
            iovecs[i].iov_len = SMS_PACKET_SIZE;
            mmsgs[i].msg_hdr.msg_iov = &mut iovecs[i];
            mmsgs[i].msg_hdr.msg_iovlen = 1;
            mmsgs[i].msg_hdr.msg_name = std::ptr::null_mut();
            mmsgs[i].msg_hdr.msg_namelen = 0;
            mmsgs[i].msg_hdr.msg_control = std::ptr::null_mut();
            mmsgs[i].msg_hdr.msg_controllen = 0;
            mmsgs[i].msg_hdr.msg_flags = 0;
            mmsgs[i].msg_len = 0;
        }

        match unsafe {
            libc::recvmmsg(
                sock.as_raw_fd(),
                mmsgs.as_mut_ptr(),
                VLEN as u32,
                0,
                std::ptr::null_mut(),
            )
        } {
            -1 => {
                let err = std::io::Error::last_os_error();
                match err.kind() {
                    std::io::ErrorKind::Interrupted => (),
                    std::io::ErrorKind::WouldBlock => thread::sleep(RETRY_TIME),
                    _ => error!("port5 error: {:?}", err),
                }
            }
            n => match tx.send(buf[..n as usize * SMS_PACKET_SIZE].to_vec()).await {
                Ok(_) => (),
                Err(e) => error!("port5 error: {:?}", e),
            },
        }
    }
}

#[cfg(not(target_os = "linux"))]
pub async fn port5(tx: AsyncSender<Vec<u8>>) {
    let sock = UdpSocket::bind("0.0.0.0:50005").await.unwrap();
    let mut buf = [0; SMS_PACKET_SIZE];

    loop {
        match sock.recv_from(&mut buf).await {
            Ok((n, _)) => match tx.send(buf.to_vec()).await {
                Ok(_) => (),
                Err(e) => error!("port5 write error: {:?}", e),
            },
            Err(e) => error!("port5 read error: {:?}", e),
        }
    }
}

/// UDP receiver for radar cube data on port 50063.
///
/// Receives Smart Micro SMS protocol packets and forwards to processing
/// channel.
///
/// # Arguments
/// * `tx` - Async channel sender for received packets
pub async fn port63(tx: AsyncSender<Vec<u8>>) {
    let sock = UdpSocket::bind("0.0.0.0:50063").await.unwrap();
    let mut buf = [0; SMS_PACKET_SIZE];

    loop {
        match sock.recv_from(&mut buf).await {
            Ok(_) => match tx.send(buf.to_vec()).await {
                Ok(_) => (),
                Err(e) => error!("port63 write error: {:?}", e),
            },
            Err(e) => error!("port63 read error: {:?}", e),
        }
    }
}
