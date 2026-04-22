use clap::{Parser, ValueEnum};
use std::io;
use std::mem;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{mpsc, Arc};
use std::time::{Duration, Instant};

// ── SocketCAN constants & structs ──────────────────────────────────────────

const AF_CAN: i32 = 29;
const PF_CAN: i32 = AF_CAN;
const CAN_RAW: i32 = 1;
// libc::Ioctl is c_ulong on glibc, c_int on musl — use the alias for portability.
const SIOCGIFINDEX: libc::Ioctl = 0x8933 as libc::Ioctl;

const SOL_CAN_RAW: libc::c_int = 101;
const CAN_RAW_FILTER: libc::c_int = 1;
const CAN_RAW_ERR_FILTER: libc::c_int = 2;
const CAN_RAW_FD_FRAMES: libc::c_int = 5;

const CAN_EFF_FLAG: u32 = 0x8000_0000;
const CAN_ERR_FLAG: u32 = 0x2000_0000;
const CAN_SFF_MASK: u32 = 0x0000_07FF;
const CAN_EFF_MASK: u32 = 0x1FFF_FFFF;
const CAN_MAX_DLC: u8 = 8;

// CAN error-frame classes (bits within can_id when CAN_ERR_FLAG is set)
const CAN_ERR_BUSOFF: u32 = 0x0000_0040;
const CAN_ERR_RESTARTED: u32 = 0x0000_0100;

#[repr(C)]
#[derive(Clone, Copy)]
struct CanFrame {
    can_id: u32,
    can_dlc: u8,
    __pad: u8,
    __res0: u8,
    __res1: u8,
    data: [u8; 8],
}

#[repr(C)]
struct SockaddrCan {
    can_family: libc::sa_family_t,
    can_ifindex: libc::c_int,
    can_addr: [u8; 8], // union, unused for raw
}

#[repr(C)]
struct Ifreq {
    ifr_name: [u8; libc::IFNAMSIZ],
    ifr_ifindex: libc::c_int,
}

// ── CLI ────────────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, ValueEnum)]
enum IdMode {
    /// Cycle through IDs sequentially
    Sequential,
    /// Pick IDs at random
    Random,
}

#[derive(Debug, Clone, Copy, ValueEnum)]
enum DataMode {
    /// Pseudo-random payload bytes (fast xorshift)
    Random,
    /// All zeros
    Zero,
    /// Incrementing counter byte repeated across payload
    Counter,
    /// 64-bit big-endian sequence number (0, 1, 2, ...)
    Sequence,
    /// All 0xFF
    Ones,
    /// CANcorder quality-test protocol (0xCAFE magic + seq16 + timestamp16 + test-id + checksum)
    QualityTest,
}

#[derive(Debug, Clone, Copy, ValueEnum)]
enum IdKind {
    /// Standard 11-bit IDs only
    Standard,
    /// Extended 29-bit IDs only
    Extended,
    /// Mix of both standard and extended
    Mixed,
}

/// mcangen — Mickey's high-performance CAN frame generator
///
/// Sends CAN frames with configurable IDs, DLC, data patterns, and rate
/// to a SocketCAN interface. Designed for testing and benchmarking.
#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Cli {
    /// CAN interface name (e.g. vcan0, can0)
    interface: String,

    /// Exact number of frames to send (0 = unlimited)
    #[arg(short = 'n', long, default_value_t = 0)]
    count: u64,

    /// Target frames per second (0 = send as fast as possible)
    #[arg(short = 'r', long = "rate", default_value_t = 5)]
    fps: u64,

    /// Minimum DLC (0–8)
    #[arg(long, default_value_t = 0, value_parser = clap::value_parser!(u8).range(0..=8))]
    dlc_min: u8,

    /// Maximum DLC (0–8)
    #[arg(long, default_value_t = 8, value_parser = clap::value_parser!(u8).range(0..=8))]
    dlc_max: u8,

    /// Minimum CAN ID (hex or decimal)
    #[arg(long, value_parser = parse_hex_u32, conflicts_with = "id")]
    id_min: Option<u32>,

    /// Maximum CAN ID (hex or decimal). Defaults to 0x7FF (standard) or 0x1FFFFFFF (extended/mixed).
    #[arg(long, value_parser = parse_hex_u32, conflicts_with = "id")]
    id_max: Option<u32>,

    /// Exact CAN ID (hex or decimal). Required for quality-test mode.
    #[arg(long, value_parser = parse_hex_u32, required_if_eq("data_mode", "quality-test"))]
    id: Option<u32>,

    /// ID type: standard (11-bit), extended (29-bit), or mixed
    #[arg(long, value_enum, default_value_t = IdKind::Standard)]
    id_kind: IdKind,

    /// Ensure extended (29-bit) IDs are always > 0x7FF so they can't be mistaken for standard IDs
    #[arg(long, default_value_t = true, action = clap::ArgAction::Set)]
    ext_id_above_sff: bool,

    /// How CAN IDs are selected
    #[arg(long, value_enum, default_value_t = IdMode::Random)]
    id_mode: IdMode,

    /// How frame data bytes are filled
    #[arg(long, value_enum, default_value_t = DataMode::Random)]
    data_mode: DataMode,

    /// RNG seed for reproducible runs (0 = random seed)
    #[arg(short = 's', long, default_value_t = 0)]
    seed: u64,

    /// Print a stats line every N frames (0 = only at end)
    #[arg(short = 'p', long = "progress", default_value_t = 0)]
    progress: u64,

    /// Suppress all output except errors
    #[arg(short = 'q', long)]
    quiet: bool,

    /// Dump sent frames to stdout in candump format
    #[arg(long)]
    dump: bool,

    /// Enable burst mode: alternate between high-rate and low-rate periods (emulates ECU reprogramming)
    #[arg(long)]
    burst: bool,

    /// Burst mode: frames per second during high-rate phase
    #[arg(long, default_value_t = 5000)]
    burst_high_rate: u64,

    /// Burst mode: frames per second during low-rate phase
    #[arg(long, default_value_t = 50)]
    burst_low_rate: u64,

    /// Burst mode: duration of high-rate phase in milliseconds
    #[arg(long, default_value_t = 2000)]
    burst_high_ms: u64,

    /// Burst mode: duration of low-rate phase in milliseconds
    #[arg(long, default_value_t = 500)]
    burst_low_ms: u64,

    /// Test ID byte for quality-test data mode (0–255)
    #[arg(long, default_value_t = 0, value_parser = clap::value_parser!(u8))]
    test_id: u8,

    /// UDS flash mode: simulate a realistic ECU reprogramming session.
    /// Both tester and ECU frames appear on the bus with proper ISO-TP
    /// framing and timing.  In this mode, -n sets the number of sessions
    /// (0 = loop forever).
    #[arg(long, verbatim_doc_comment)]
    uds_flash: bool,

    /// [UDS flash] Tester request CAN ID (hex or decimal)
    #[arg(long, default_value = "0x7E0", value_parser = parse_hex_u32)]
    tester_id: u32,

    /// [UDS flash] ECU response CAN ID (hex or decimal)
    #[arg(long, default_value = "0x7E8", value_parser = parse_hex_u32)]
    ecu_id: u32,

    /// [UDS flash] Timing multiplier (2.0 = double speed, 0.5 = half)
    #[arg(long, default_value_t = 1.0)]
    speed: f64,

    /// [UDS flash] Transfer blocks per session (0 = random 50–150)
    #[arg(long, default_value_t = 0)]
    transfer_blocks: u32,

    /// [UDS flash] Skip OBD-II polling between sessions
    #[arg(long)]
    no_obd: bool,

    /// [UDS flash] Disable error injection (clean sessions only)
    #[arg(long)]
    no_errors: bool,

    /// On BUS-OFF, cycle the interface (down + up) via netlink to recover.
    /// Needed for drivers that don't implement do_set_mode (notably gs_usb /
    /// candleLight / Canable v1) — for those, restart-ms and the manual
    /// `ip link … type can restart` trigger both return EOPNOTSUPP, so the
    /// kernel cannot self-recover. Requires CAP_NET_ADMIN; on EPERM the flag
    /// is silently disabled with a one-shot warning.
    #[arg(long)]
    auto_restart: bool,
}

fn parse_hex_u32(s: &str) -> Result<u32, String> {
    if let Some(hex) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        u32::from_str_radix(hex, 16).map_err(|e| e.to_string())
    } else {
        s.parse::<u32>().map_err(|e| e.to_string())
    }
}

// ── Socket helpers ─────────────────────────────────────────────────────────

fn open_can_socket(ifname: &str) -> io::Result<i32> {
    unsafe {
        let fd = libc::socket(PF_CAN, libc::SOCK_RAW, CAN_RAW);
        if fd < 0 {
            return Err(io::Error::last_os_error());
        }

        // Resolve interface index
        let mut ifr: Ifreq = mem::zeroed();
        let name_bytes = ifname.as_bytes();
        if name_bytes.len() >= libc::IFNAMSIZ {
            libc::close(fd);
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "interface name too long",
            ));
        }
        ifr.ifr_name[..name_bytes.len()].copy_from_slice(name_bytes);

        if libc::ioctl(fd, SIOCGIFINDEX, &mut ifr as *mut Ifreq) < 0 {
            let e = io::Error::last_os_error();
            libc::close(fd);
            return Err(e);
        }

        let mut addr: SockaddrCan = mem::zeroed();
        addr.can_family = AF_CAN as libc::sa_family_t;
        addr.can_ifindex = ifr.ifr_ifindex;

        if libc::bind(
            fd,
            &addr as *const SockaddrCan as *const libc::sockaddr,
            mem::size_of::<SockaddrCan>() as libc::socklen_t,
        ) < 0
        {
            let e = io::Error::last_os_error();
            libc::close(fd);
            return Err(e);
        }

        // Enable CAN-FD reception/transmission (non-fatal if unsupported)
        let enable: libc::c_int = 1;
        libc::setsockopt(
            fd,
            SOL_CAN_RAW,
            CAN_RAW_FD_FRAMES,
            &enable as *const libc::c_int as *const libc::c_void,
            mem::size_of::<libc::c_int>() as libc::socklen_t,
        );

        // Increase send buffer for burst performance
        let sndbuf: libc::c_int = 1 << 20; // 1 MiB
        libc::setsockopt(
            fd,
            libc::SOL_SOCKET,
            libc::SO_SNDBUF,
            &sndbuf as *const libc::c_int as *const libc::c_void,
            mem::size_of::<libc::c_int>() as libc::socklen_t,
        );

        Ok(fd)
    }
}

/// Open a CAN_RAW socket configured for error-frame monitoring only: the
/// normal RX filter is cleared (no classic frames delivered, so the kernel
/// receive buffer can't back up) and the error filter enables BUSOFF and
/// RESTARTED notifications. A 1s SO_RCVTIMEO lets the monitor thread wake up
/// periodically to re-check the shutdown flag.
fn open_err_socket(ifname: &str) -> io::Result<i32> {
    unsafe {
        let fd = libc::socket(PF_CAN, libc::SOCK_RAW, CAN_RAW);
        if fd < 0 {
            return Err(io::Error::last_os_error());
        }

        let mut ifr: Ifreq = mem::zeroed();
        let name_bytes = ifname.as_bytes();
        if name_bytes.len() >= libc::IFNAMSIZ {
            libc::close(fd);
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "interface name too long",
            ));
        }
        ifr.ifr_name[..name_bytes.len()].copy_from_slice(name_bytes);

        if libc::ioctl(fd, SIOCGIFINDEX, &mut ifr as *mut Ifreq) < 0 {
            let e = io::Error::last_os_error();
            libc::close(fd);
            return Err(e);
        }

        // Drop all normal frames — we only care about error frames here.
        if libc::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, std::ptr::null(), 0) < 0 {
            let e = io::Error::last_os_error();
            libc::close(fd);
            return Err(e);
        }

        let err_mask: u32 = CAN_ERR_BUSOFF | CAN_ERR_RESTARTED;
        if libc::setsockopt(
            fd,
            SOL_CAN_RAW,
            CAN_RAW_ERR_FILTER,
            &err_mask as *const u32 as *const libc::c_void,
            mem::size_of::<u32>() as libc::socklen_t,
        ) < 0
        {
            let e = io::Error::last_os_error();
            libc::close(fd);
            return Err(e);
        }

        let timeout = libc::timeval {
            tv_sec: 1,
            tv_usec: 0,
        };
        libc::setsockopt(
            fd,
            libc::SOL_SOCKET,
            libc::SO_RCVTIMEO,
            &timeout as *const libc::timeval as *const libc::c_void,
            mem::size_of::<libc::timeval>() as libc::socklen_t,
        );

        let mut addr: SockaddrCan = mem::zeroed();
        addr.can_family = AF_CAN as libc::sa_family_t;
        addr.can_ifindex = ifr.ifr_ifindex;
        if libc::bind(
            fd,
            &addr as *const SockaddrCan as *const libc::sockaddr,
            mem::size_of::<SockaddrCan>() as libc::socklen_t,
        ) < 0
        {
            let e = io::Error::last_os_error();
            libc::close(fd);
            return Err(e);
        }

        Ok(fd)
    }
}

const BATCH_SIZE: usize = 64;

/// Send multiple CAN frames in a single syscall using sendmmsg.
/// Returns the number of frames successfully sent.
#[inline]
fn send_frames_batch(fd: i32, frames: &[CanFrame]) -> io::Result<usize> {
    let count = frames.len().min(BATCH_SIZE);
    if count == 0 {
        return Ok(0);
    }

    let mut iovecs: [libc::iovec; BATCH_SIZE] = unsafe { mem::zeroed() };
    let mut msgvec: [libc::mmsghdr; BATCH_SIZE] = unsafe { mem::zeroed() };

    for i in 0..count {
        iovecs[i] = libc::iovec {
            iov_base: &frames[i] as *const CanFrame as *mut libc::c_void,
            iov_len: mem::size_of::<CanFrame>(),
        };
        msgvec[i].msg_hdr.msg_iov = &mut iovecs[i];
        msgvec[i].msg_hdr.msg_iovlen = 1;
    }

    let ret = unsafe { libc::sendmmsg(fd, msgvec.as_mut_ptr(), count as u32, 0) };
    if ret < 0 {
        Err(io::Error::last_os_error())
    } else {
        Ok(ret as usize)
    }
}

#[inline(always)]
fn send_frame(fd: i32, frame: &CanFrame) -> io::Result<()> {
    let ret = unsafe {
        libc::write(
            fd,
            frame as *const CanFrame as *const libc::c_void,
            mem::size_of::<CanFrame>(),
        )
    };
    if ret < 0 {
        Err(io::Error::last_os_error())
    } else {
        Ok(())
    }
}

// ── Interface vanishing / reconnect ────────────────────────────────────────

/// Errors that indicate the CAN interface has gone away (admin-down, removed,
/// or otherwise unreachable). ENOBUFS is intentionally NOT included — it just
/// means the kernel send queue is full on a busy bus.
#[inline]
fn is_iface_down_err(e: &io::Error) -> bool {
    matches!(
        e.raw_os_error(),
        Some(code) if code == libc::ENETDOWN
            || code == libc::ENODEV
            || code == libc::ENXIO
            || code == libc::ENETUNREACH
    )
}

/// Check whether the named interface is administratively up. Returns `None`
/// if the interface is not known to the kernel (yet). A CAN controller in
/// BUS-OFF state is still `IFF_UP` — that case is caught separately by the
/// error-frame monitor.
fn iface_is_up(ifname: &str) -> Option<bool> {
    use nix::ifaddrs::getifaddrs;
    use nix::net::if_::InterfaceFlags;

    let addrs = getifaddrs().ok()?;
    let mut seen = false;
    for a in addrs {
        if a.interface_name == ifname {
            seen = true;
            if a.flags.contains(InterfaceFlags::IFF_UP) {
                return Some(true);
            }
        }
    }
    if seen { Some(false) } else { None }
}

/// Close the dead fd and keep retrying `open_can_socket()` with exponential
/// backoff (100ms → 30s cap). Updates `*fd` in place. After each successful
/// open the interface's `IFF_UP` flag is rechecked; if the interface exists
/// but is administratively down, the new fd is discarded and we keep backing
/// off. This prevents a tight loop when `ip link set <iface> down` leaves the
/// device resolvable but unusable. Note: classic CAN frames are sent here
/// regardless of whether the new interface is FD-capable, so a non-FD return
/// on reattach is intentionally tolerated.
fn reconnect_socket(fd: &mut i32, ifname: &str, quiet: bool) {
    unsafe {
        libc::close(*fd);
    }
    *fd = -1;

    if !quiet {
        eprintln!(
            "\nmcangen: interface '{}' is gone — reattaching with backoff…",
            ifname
        );
    }

    let mut delay = Duration::from_millis(100);
    let max_delay = Duration::from_secs(30);
    let outage_start = Instant::now();
    let mut attempts: u32 = 0;

    loop {
        attempts += 1;
        match open_can_socket(ifname) {
            Ok(new_fd) => {
                // Even if open succeeds, refuse to declare victory while the
                // iface is admin-down — writes would fail immediately and we
                // would spin.
                if iface_is_up(ifname) != Some(true) {
                    unsafe {
                        libc::close(new_fd);
                    }
                    std::thread::sleep(delay);
                    delay = (delay * 2).min(max_delay);
                    continue;
                }
                if !quiet {
                    eprintln!(
                        "mcangen: interface '{}' back after {:.1}s ({} attempt{}) — resuming",
                        ifname,
                        outage_start.elapsed().as_secs_f64(),
                        attempts,
                        if attempts == 1 { "" } else { "s" },
                    );
                }
                *fd = new_fd;
                return;
            }
            Err(_) => {
                std::thread::sleep(delay);
                delay = (delay * 2).min(max_delay);
            }
        }
    }
}

// ── Netlink: cycle the interface to recover from BUS-OFF ──────────────────

/// Resolve an interface name to its kernel ifindex.
fn if_nametoindex(ifname: &str) -> io::Result<u32> {
    let cstr = std::ffi::CString::new(ifname)
        .map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "interface name has nul byte"))?;
    let idx = unsafe { libc::if_nametoindex(cstr.as_ptr()) };
    if idx == 0 {
        Err(io::Error::last_os_error())
    } else {
        Ok(idx)
    }
}

/// Send an RTM_NEWLINK request that toggles only the IFF_UP bit on `ifindex`,
/// and synchronously parse the kernel's NLMSG_ERROR ack. Returns:
///   - `Ok(())` on success (kernel returned error=0).
///   - `Err(EPERM)` when CAP_NET_ADMIN is missing.
///   - other `io::Error` for transport failures or kernel errors.
fn nl_set_iface_up(ifindex: u32, up: bool) -> io::Result<()> {
    unsafe {
        let fd = libc::socket(
            libc::AF_NETLINK,
            libc::SOCK_RAW | libc::SOCK_CLOEXEC,
            libc::NETLINK_ROUTE,
        );
        if fd < 0 {
            return Err(io::Error::last_os_error());
        }

        // Bind locally — kernel auto-assigns nl_pid since we leave it 0.
        let mut local: libc::sockaddr_nl = mem::zeroed();
        local.nl_family = libc::AF_NETLINK as libc::sa_family_t;
        if libc::bind(
            fd,
            &local as *const libc::sockaddr_nl as *const libc::sockaddr,
            mem::size_of::<libc::sockaddr_nl>() as libc::socklen_t,
        ) < 0
        {
            let e = io::Error::last_os_error();
            libc::close(fd);
            return Err(e);
        }

        // Build the request: nlmsghdr + ifinfomsg, no attributes.
        let hdr_len = mem::size_of::<libc::nlmsghdr>();
        let info_len = mem::size_of::<libc::ifinfomsg>();
        let total_len = hdr_len + info_len;

        let mut buf = [0u8; 64];
        let hdr = libc::nlmsghdr {
            nlmsg_len: total_len as u32,
            nlmsg_type: libc::RTM_NEWLINK,
            nlmsg_flags: (libc::NLM_F_REQUEST | libc::NLM_F_ACK) as u16,
            nlmsg_seq: 1,
            nlmsg_pid: 0,
        };
        std::ptr::write_unaligned(buf.as_mut_ptr() as *mut libc::nlmsghdr, hdr);

        let mut info: libc::ifinfomsg = mem::zeroed();
        info.ifi_family = libc::AF_UNSPEC as u8;
        info.ifi_index = ifindex as libc::c_int;
        info.ifi_flags = if up { libc::IFF_UP as libc::c_uint } else { 0 };
        info.ifi_change = libc::IFF_UP as libc::c_uint;
        std::ptr::write_unaligned(
            buf.as_mut_ptr().add(hdr_len) as *mut libc::ifinfomsg,
            info,
        );

        let mut kernel: libc::sockaddr_nl = mem::zeroed();
        kernel.nl_family = libc::AF_NETLINK as libc::sa_family_t;
        // nl_pid = 0 → kernel; nl_groups = 0 → unicast

        let n = libc::sendto(
            fd,
            buf.as_ptr() as *const libc::c_void,
            total_len,
            0,
            &kernel as *const libc::sockaddr_nl as *const libc::sockaddr,
            mem::size_of::<libc::sockaddr_nl>() as libc::socklen_t,
        );
        if n < 0 {
            let e = io::Error::last_os_error();
            libc::close(fd);
            return Err(e);
        }

        let mut rbuf = [0u8; 4096];
        let n = libc::recv(
            fd,
            rbuf.as_mut_ptr() as *mut libc::c_void,
            rbuf.len(),
            0,
        );
        let recv_err = if n < 0 {
            Some(io::Error::last_os_error())
        } else {
            None
        };
        libc::close(fd);
        if let Some(e) = recv_err {
            return Err(e);
        }

        let n = n as usize;
        if n < hdr_len {
            return Err(io::Error::other("netlink response shorter than nlmsghdr"));
        }
        let resp = std::ptr::read_unaligned(rbuf.as_ptr() as *const libc::nlmsghdr);
        if i32::from(resp.nlmsg_type) != libc::NLMSG_ERROR {
            return Err(io::Error::other(format!(
                "unexpected netlink response type {}",
                resp.nlmsg_type
            )));
        }
        if n < hdr_len + mem::size_of::<i32>() {
            return Err(io::Error::other("netlink error response truncated"));
        }
        // First field of nlmsgerr is `int error` — negative errno on failure, 0 on ack.
        let err_code = std::ptr::read_unaligned(rbuf.as_ptr().add(hdr_len) as *const i32);
        if err_code == 0 {
            Ok(())
        } else {
            Err(io::Error::from_raw_os_error(-err_code))
        }
    }
}

/// Bring the interface down, briefly settle, then bring it back up. For
/// gs_usb-class drivers this is the only way to reset the controller after
/// BUS-OFF; the kernel can't do it because there is no `do_set_mode`.
fn cycle_iface(ifname: &str) -> io::Result<()> {
    let ifindex = if_nametoindex(ifname)?;
    nl_set_iface_up(ifindex, false)?;
    // Give the driver a moment to tear the device down before re-arming. 150ms
    // is generous for USB; locally-bussed CAN controllers settle even faster.
    std::thread::sleep(Duration::from_millis(150));
    nl_set_iface_up(ifindex, true)
}

// ── Fast RNG (xorshift64*) ─────────────────────────────────────────────────

struct Rng(u64);

impl Rng {
    fn new(seed: u64) -> Self {
        Self(if seed == 0 {
            fastrand::u64(..) | 1
        } else {
            seed
        })
    }

    #[inline(always)]
    fn next(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    #[inline(always)]
    fn next_u32(&mut self) -> u32 {
        self.next() as u32
    }

    #[inline(always)]
    fn range_u32(&mut self, lo: u32, hi: u32) -> u32 {
        if lo == hi {
            return lo;
        }
        let span = hi - lo + 1;
        lo + (self.next_u32() % span)
    }

    #[inline(always)]
    fn range_u8(&mut self, lo: u8, hi: u8) -> u8 {
        if lo == hi {
            return lo;
        }
        let span = (hi - lo + 1) as u32;
        lo + (self.next_u32() % span) as u8
    }

    fn uniform(&mut self, lo: f64, hi: f64) -> f64 {
        let t = (self.next() >> 11) as f64 / (1u64 << 53) as f64;
        lo + t * (hi - lo)
    }

    fn delay_us(&mut self, lo_ms: f64, hi_ms: f64) -> u64 {
        (self.uniform(lo_ms, hi_ms) * 1000.0) as u64
    }

    fn chance(&mut self, p: f64) -> bool {
        self.uniform(0.0, 1.0) < p
    }

    fn fill_bytes(&mut self, buf: &mut [u8]) {
        for chunk in buf.chunks_mut(8) {
            let r = self.next().to_ne_bytes();
            chunk.copy_from_slice(&r[..chunk.len()]);
        }
    }
}

// ── Precise rate-limited sleep ─────────────────────────────────────────────

/// Sleep until `target` time using clock_nanosleep for precision, then
/// busy-spin only the final ~100µs for sub-microsecond accuracy.
/// Much lower CPU usage than the previous 2ms busy-spin approach.
#[inline]
fn wait_until(target: Instant) {
    let now = Instant::now();
    if now >= target {
        return;
    }
    let remaining = target - now;
    // Use clock_nanosleep for the bulk — it's more precise than thread::sleep
    // because it goes through the high-resolution timer path in the kernel.
    // Leave 100µs for the final spin to absorb scheduling jitter.
    if remaining > Duration::from_micros(200) {
        let sleep_dur = remaining - Duration::from_micros(100);
        let ts = libc::timespec {
            tv_sec: sleep_dur.as_secs() as libc::time_t,
            tv_nsec: sleep_dur.subsec_nanos() as libc::c_long,
        };
        unsafe {
            libc::clock_nanosleep(libc::CLOCK_MONOTONIC, 0, &ts, std::ptr::null_mut());
        }
    }
    // Busy-spin the final stretch for accuracy
    while Instant::now() < target {
        std::hint::spin_loop();
    }
}

// ── UDS Flash simulation ──────────────────────────────────────────────────

const ISOTP_PAD: u8 = 0xCC;

struct TimedFrame {
    can_id: u32, // 0 = delay-only marker (no frame sent)
    data: [u8; 8],
    pre_delay_us: u64,
}

// ── ISO-TP framing helpers ────────────────────────────────────────────────

fn push_sf(frames: &mut Vec<TimedFrame>, can_id: u32, payload: &[u8], delay_us: u64) {
    debug_assert!(payload.len() <= 7);
    let mut data = [ISOTP_PAD; 8];
    data[0] = payload.len() as u8;
    data[1..1 + payload.len()].copy_from_slice(payload);
    frames.push(TimedFrame {
        can_id,
        data,
        pre_delay_us: delay_us,
    });
}

fn push_nrc(frames: &mut Vec<TimedFrame>, ecu: u32, sid: u8, nrc: u8, delay_us: u64) {
    push_sf(frames, ecu, &[0x7F, sid, nrc], delay_us);
}

fn push_multi(
    frames: &mut Vec<TimedFrame>,
    sender: u32,
    responder: u32,
    payload: &[u8],
    ff_delay_us: u64,
    fc_delay_us: u64,
    cf_delay_us: u64,
) {
    let total = payload.len();

    // First Frame
    let mut ff = [0u8; 8];
    ff[0] = 0x10 | ((total >> 8) & 0x0F) as u8;
    ff[1] = (total & 0xFF) as u8;
    let n = total.min(6);
    ff[2..2 + n].copy_from_slice(&payload[..n]);
    frames.push(TimedFrame {
        can_id: sender,
        data: ff,
        pre_delay_us: ff_delay_us,
    });

    // Flow Control
    let mut fc = [ISOTP_PAD; 8];
    fc[0] = 0x30;
    fc[1] = 0x00; // block_size = unlimited
    fc[2] = 0x0A; // st_min = 10 ms
    frames.push(TimedFrame {
        can_id: responder,
        data: fc,
        pre_delay_us: fc_delay_us,
    });

    // Consecutive Frames
    let mut off = 6;
    let mut seq = 1u8;
    while off < total {
        let mut cf = [ISOTP_PAD; 8];
        cf[0] = 0x20 | (seq & 0x0F);
        let end = (off + 7).min(total);
        cf[1..1 + end - off].copy_from_slice(&payload[off..end]);
        frames.push(TimedFrame {
            can_id: sender,
            data: cf,
            pre_delay_us: cf_delay_us,
        });
        off += 7;
        seq = (seq + 1) & 0x0F;
    }
}

// ── UDS session phases ────────────────────────────────────────────────────

fn gen_session_control(
    f: &mut Vec<TimedFrame>,
    tester: u32,
    ecu: u32,
    session: u8,
    errors: bool,
    rng: &mut Rng,
) {
    let req = [0x10, session];
    if errors && rng.chance(0.03) {
        push_sf(f, tester, &req, rng.delay_us(20.0, 50.0));
        push_nrc(f, ecu, 0x10, 0x21, rng.delay_us(15.0, 30.0)); // busy
        push_sf(f, tester, &req, rng.delay_us(100.0, 200.0)); // retry
    } else {
        push_sf(f, tester, &req, rng.delay_us(20.0, 50.0));
    }
    // P2=25ms, P2*=5000ms
    push_sf(
        f,
        ecu,
        &[0x50, session, 0x00, 0x19, 0x01, 0xF4],
        rng.delay_us(15.0, 40.0),
    );
}

fn gen_read_identification(f: &mut Vec<TimedFrame>, tester: u32, ecu: u32, rng: &mut Rng) {
    const IDS: [(u16, &[u8]); 5] = [
        (0xF190, b"WVWZZZ3CZWE123456"), // VIN
        (0xF18C, b"ECU12345678"),       // Serial Number
        (0xF195, b"SW_V02.15.003"),     // Software Version
        (0xF193, b"HW_REV_C"),          // Hardware Version
        (0xF187, b"PART_1K0907115B"),   // Part Number
    ];
    for &(did, val) in &IDS {
        push_sf(
            f,
            tester,
            &[0x22, (did >> 8) as u8, did as u8],
            rng.delay_us(30.0, 80.0),
        );

        let mut resp = Vec::with_capacity(3 + val.len());
        resp.extend_from_slice(&[0x62, (did >> 8) as u8, did as u8]);
        resp.extend_from_slice(val);

        if resp.len() <= 7 {
            push_sf(f, ecu, &resp, rng.delay_us(10.0, 30.0));
        } else {
            push_multi(
                f,
                ecu,
                tester,
                &resp,
                rng.delay_us(10.0, 30.0),
                rng.delay_us(5.0, 15.0),
                rng.delay_us(5.0, 15.0),
            );
        }
    }
}

fn gen_security_access(
    f: &mut Vec<TimedFrame>,
    tester: u32,
    ecu: u32,
    errors: bool,
    rng: &mut Rng,
) {
    let lvl: u8 = 0x11; // programming security level
    if errors && rng.chance(0.15) {
        // Failed first attempt
        push_sf(f, tester, &[0x27, lvl], rng.delay_us(20.0, 40.0));
        let s = rng.next_u32().to_be_bytes();
        push_sf(
            f,
            ecu,
            &[0x67, lvl, s[0], s[1], s[2], s[3]],
            rng.delay_us(100.0, 200.0),
        );
        let k = rng.next_u32().to_be_bytes(); // wrong key
        push_sf(
            f,
            tester,
            &[0x27, lvl + 1, k[0], k[1], k[2], k[3]],
            rng.delay_us(50.0, 100.0),
        );
        push_nrc(f, ecu, 0x27, 0x35, rng.delay_us(30.0, 80.0)); // invalidKey
        f.push(TimedFrame {
            can_id: 0,
            data: [0; 8],
            pre_delay_us: rng.delay_us(500.0, 1000.0),
        });
    }
    // Successful attempt
    push_sf(f, tester, &[0x27, lvl], rng.delay_us(20.0, 40.0));
    let s = rng.next_u32().to_be_bytes();
    push_sf(
        f,
        ecu,
        &[0x67, lvl, s[0], s[1], s[2], s[3]],
        rng.delay_us(100.0, 250.0),
    );
    let key = [s[0] ^ 0xCA, s[1] ^ 0xCA, s[2] ^ 0xCA, s[3] ^ 0xCA];
    push_sf(
        f,
        tester,
        &[0x27, lvl + 1, key[0], key[1], key[2], key[3]],
        rng.delay_us(50.0, 100.0),
    );
    push_sf(f, ecu, &[0x67, lvl + 1], rng.delay_us(80.0, 200.0));
}

fn gen_check_preconditions(
    f: &mut Vec<TimedFrame>,
    tester: u32,
    ecu: u32,
    errors: bool,
    rng: &mut Rng,
) {
    let req = [0x31, 0x01, 0xFF, 0x00]; // routine 0xFF00
    if errors && rng.chance(0.02) {
        push_sf(f, tester, &req, rng.delay_us(30.0, 60.0));
        push_nrc(f, ecu, 0x31, 0x22, rng.delay_us(20.0, 50.0)); // conditionsNotCorrect
        f.push(TimedFrame {
            can_id: 0,
            data: [0; 8],
            pre_delay_us: rng.delay_us(300.0, 600.0),
        });
        push_sf(f, tester, &req, rng.delay_us(30.0, 60.0)); // retry
    } else {
        push_sf(f, tester, &req, rng.delay_us(30.0, 60.0));
    }
    push_sf(
        f,
        ecu,
        &[0x71, 0x01, 0xFF, 0x00, 0x00],
        rng.delay_us(50.0, 150.0),
    );
}

fn gen_erase_memory(f: &mut Vec<TimedFrame>, tester: u32, ecu: u32, rng: &mut Rng) {
    // Routine 0xFF01, addr 0x00080000, size 0x00040000
    let payload: [u8; 13] = [
        0x31, 0x01, 0xFF, 0x01, 0x44, 0x00, 0x08, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00,
    ];
    push_multi(
        f,
        tester,
        ecu,
        &payload,
        rng.delay_us(20.0, 40.0),
        rng.delay_us(5.0, 15.0),
        rng.delay_us(5.0, 15.0),
    );

    // Pending responses while flash is being erased
    let num_pending = rng.range_u32(3, 8);
    for _ in 0..num_pending {
        push_nrc(f, ecu, 0x31, 0x78, rng.delay_us(400.0, 700.0)); // responsePending
    }
    push_sf(
        f,
        ecu,
        &[0x71, 0x01, 0xFF, 0x01, 0x00],
        rng.delay_us(300.0, 600.0),
    );
}

fn gen_request_download(f: &mut Vec<TimedFrame>, tester: u32, ecu: u32, rng: &mut Rng) {
    // No compression/encryption, 4-byte addr + 4-byte size
    let payload: [u8; 11] = [
        0x34, 0x00, 0x44, 0x00, 0x08, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00,
    ];
    push_multi(
        f,
        tester,
        ecu,
        &payload,
        rng.delay_us(20.0, 40.0),
        rng.delay_us(5.0, 15.0),
        rng.delay_us(5.0, 15.0),
    );
    // max block length 258
    push_sf(f, ecu, &[0x74, 0x20, 0x01, 0x02], rng.delay_us(30.0, 80.0));
}

fn gen_transfer_data(
    f: &mut Vec<TimedFrame>,
    tester: u32,
    ecu: u32,
    num_blocks: usize,
    errors: bool,
    rng: &mut Rng,
) {
    let mut ctr: u8 = 0;
    let mut error_done = false;

    for i in 0..num_blocks {
        ctr = ctr.wrapping_add(1);
        let bsz = rng.range_u32(200, 256) as usize;

        // Random "firmware" payload: [SID=0x36, blockCounter, data…]
        let mut payload = vec![0x36, ctr];
        let start = payload.len();
        payload.resize(start + bsz, 0);
        rng.fill_bytes(&mut payload[start..]);

        // One-shot error injection per session
        if errors && !error_done && i > 5 && rng.chance(0.02) {
            error_done = true;
            let wrong = ctr.wrapping_add(rng.range_u8(1, 5));
            let mut bad = payload.clone();
            bad[1] = wrong;
            push_multi(
                f,
                tester,
                ecu,
                &bad,
                rng.delay_us(15.0, 30.0),
                rng.delay_us(3.0, 8.0),
                rng.delay_us(3.0, 8.0),
            );
            push_nrc(f, ecu, 0x36, 0x73, rng.delay_us(20.0, 50.0)); // wrongBlockSequence
        }

        // Normal (or retry) transfer
        push_multi(
            f,
            tester,
            ecu,
            &payload,
            rng.delay_us(15.0, 30.0),
            rng.delay_us(3.0, 8.0),
            rng.delay_us(3.0, 8.0),
        );

        // Occasionally a pending before the positive ack
        if rng.chance(0.05) {
            push_nrc(f, ecu, 0x36, 0x78, rng.delay_us(100.0, 200.0));
        }
        push_sf(f, ecu, &[0x76, ctr], rng.delay_us(10.0, 25.0));
    }
}

fn gen_dtc_sequence(f: &mut Vec<TimedFrame>, tester: u32, ecu: u32, rng: &mut Rng) {
    // Read DTC count
    let n = rng.range_u8(3, 7);
    push_sf(f, tester, &[0x19, 0x01, 0xFF], rng.delay_us(30.0, 60.0));
    push_sf(
        f,
        ecu,
        &[0x59, 0x01, 0xFF, 0x01, 0x00, n],
        rng.delay_us(15.0, 40.0),
    );

    // Read DTCs by status mask (multi-frame when > 1 DTC)
    push_sf(f, tester, &[0x19, 0x02, 0xFF], rng.delay_us(30.0, 60.0));
    let mut resp = vec![0x59u8, 0x02, 0xFF];
    for _ in 0..n {
        resp.push(rng.next() as u8); // DTC high
        resp.push(rng.next() as u8); // DTC low
        resp.push(0x00); // failure type
        resp.push(0x08 | (rng.next() as u8 & 0x2C)); // status
    }
    if resp.len() <= 7 {
        push_sf(f, ecu, &resp, rng.delay_us(20.0, 50.0));
    } else {
        push_multi(
            f,
            ecu,
            tester,
            &resp,
            rng.delay_us(20.0, 50.0),
            rng.delay_us(5.0, 15.0),
            rng.delay_us(5.0, 15.0),
        );
    }

    // Clear all DTCs (group 0xFFFFFF)
    push_sf(
        f,
        tester,
        &[0x14, 0xFF, 0xFF, 0xFF],
        rng.delay_us(30.0, 60.0),
    );
    push_nrc(f, ecu, 0x14, 0x78, rng.delay_us(150.0, 300.0)); // pending
    push_sf(f, ecu, &[0x54], rng.delay_us(100.0, 250.0));

    // Verify cleared
    push_sf(f, tester, &[0x19, 0x01, 0xFF], rng.delay_us(30.0, 60.0));
    push_sf(
        f,
        ecu,
        &[0x59, 0x01, 0xFF, 0x01, 0x00, 0x00],
        rng.delay_us(15.0, 40.0),
    );
}

// ── Full UDS session ──────────────────────────────────────────────────────

fn gen_uds_session(
    tester: u32,
    ecu: u32,
    num_blocks: usize,
    errors: bool,
    rng: &mut Rng,
) -> Vec<TimedFrame> {
    let mut f: Vec<TimedFrame> = Vec::with_capacity(num_blocks * 40 + 200);

    // Phase 1: Extended diagnostic session
    gen_session_control(&mut f, tester, ecu, 0x03, errors, rng);

    // Phase 2: Read ECU identification
    gen_read_identification(&mut f, tester, ecu, rng);

    // Phase 3: Programming session
    gen_session_control(&mut f, tester, ecu, 0x02, errors, rng);

    // Phase 4: Security access (seed & key)
    gen_security_access(&mut f, tester, ecu, errors, rng);

    // Phase 5: Disable normal communication
    push_sf(
        &mut f,
        tester,
        &[0x28, 0x03, 0x01],
        rng.delay_us(20.0, 40.0),
    );
    push_sf(&mut f, ecu, &[0x68, 0x03], rng.delay_us(10.0, 25.0));

    // Phase 6: Disable DTC setting
    push_sf(&mut f, tester, &[0x85, 0x02], rng.delay_us(20.0, 40.0));
    push_sf(&mut f, ecu, &[0xC5, 0x02], rng.delay_us(10.0, 25.0));

    // Phase 7: Check programming preconditions
    gen_check_preconditions(&mut f, tester, ecu, errors, rng);

    // Phase 8: Erase memory
    gen_erase_memory(&mut f, tester, ecu, rng);

    // Phase 9: Request download
    gen_request_download(&mut f, tester, ecu, rng);

    // Phase 10: Transfer data blocks
    gen_transfer_data(&mut f, tester, ecu, num_blocks, errors, rng);

    // Phase 11: Request transfer exit
    push_sf(&mut f, tester, &[0x37], rng.delay_us(20.0, 40.0));
    push_sf(&mut f, ecu, &[0x77], rng.delay_us(30.0, 80.0));

    // Phase 12: Check programming dependencies
    push_sf(
        &mut f,
        tester,
        &[0x31, 0x01, 0xFF, 0x02],
        rng.delay_us(30.0, 60.0),
    );
    push_nrc(&mut f, ecu, 0x31, 0x78, rng.delay_us(200.0, 400.0));
    push_sf(
        &mut f,
        ecu,
        &[0x71, 0x01, 0xFF, 0x02, 0x00],
        rng.delay_us(100.0, 300.0),
    );

    // Phase 13–15: Read DTCs → clear → verify
    gen_dtc_sequence(&mut f, tester, ecu, rng);

    // Phase 16: ECU hard reset
    push_sf(&mut f, tester, &[0x11, 0x01], rng.delay_us(30.0, 60.0));
    push_sf(&mut f, ecu, &[0x51, 0x01], rng.delay_us(20.0, 50.0));

    f
}

// ── OBD-II inter-session traffic ──────────────────────────────────────────

fn gen_obd_polling(ecu: u32, cycles: usize, rng: &mut Rng) -> Vec<TimedFrame> {
    let mut f = Vec::with_capacity(cycles * 10);
    let obd_rx: u32 = 0x7DF; // functional addressing

    let mut coolant: f64 = 85.0;
    let mut rpm: f64 = 800.0;
    let mut speed: f64 = 0.0;
    let mut throttle: f64 = 15.0;

    for _ in 0..cycles {
        // Drift simulated vehicle state
        coolant = (coolant + rng.uniform(-1.0, 1.0)).clamp(70.0, 105.0);
        rpm = (rpm + rng.uniform(-50.0, 50.0)).clamp(650.0, 1200.0);
        speed = (speed + rng.uniform(-1.0, 1.0)).clamp(0.0, 20.0);
        throttle = (throttle + rng.uniform(-2.0, 2.0)).clamp(12.0, 25.0);

        // Engine RPM (PID 0x0C)
        if rng.chance(0.7) {
            f.push(TimedFrame {
                can_id: obd_rx,
                data: [0x02, 0x01, 0x0C, 0, 0, 0, 0, 0],
                pre_delay_us: rng.delay_us(15.0, 35.0),
            });
            let r = (rpm * 4.0) as u16;
            f.push(TimedFrame {
                can_id: ecu,
                data: [0x04, 0x41, 0x0C, (r >> 8) as u8, r as u8, 0, 0, 0],
                pre_delay_us: rng.delay_us(5.0, 25.0),
            });
        }
        // Coolant temp (PID 0x05)
        if rng.chance(0.6) {
            f.push(TimedFrame {
                can_id: obd_rx,
                data: [0x02, 0x01, 0x05, 0, 0, 0, 0, 0],
                pre_delay_us: rng.delay_us(15.0, 35.0),
            });
            f.push(TimedFrame {
                can_id: ecu,
                data: [0x03, 0x41, 0x05, (coolant as i32 + 40) as u8, 0, 0, 0, 0],
                pre_delay_us: rng.delay_us(5.0, 25.0),
            });
        }
        // Vehicle speed (PID 0x0D)
        if rng.chance(0.5) {
            f.push(TimedFrame {
                can_id: obd_rx,
                data: [0x02, 0x01, 0x0D, 0, 0, 0, 0, 0],
                pre_delay_us: rng.delay_us(15.0, 35.0),
            });
            f.push(TimedFrame {
                can_id: ecu,
                data: [0x03, 0x41, 0x0D, speed as u8, 0, 0, 0, 0],
                pre_delay_us: rng.delay_us(5.0, 25.0),
            });
        }
        // Throttle (PID 0x11)
        if rng.chance(0.5) {
            f.push(TimedFrame {
                can_id: obd_rx,
                data: [0x02, 0x01, 0x11, 0, 0, 0, 0, 0],
                pre_delay_us: rng.delay_us(15.0, 35.0),
            });
            f.push(TimedFrame {
                can_id: ecu,
                data: [
                    0x03,
                    0x41,
                    0x11,
                    (throttle * 255.0 / 100.0) as u8,
                    0,
                    0,
                    0,
                    0,
                ],
                pre_delay_us: rng.delay_us(5.0, 25.0),
            });
        }

        // Inter-cycle delay (~5–10 Hz scan rate)
        f.push(TimedFrame {
            can_id: 0,
            data: [0; 8],
            pre_delay_us: rng.delay_us(80.0, 150.0),
        });
    }
    f
}

// ── Frame playback ────────────────────────────────────────────────────────

fn play_timed_frames(
    fd: &mut i32,
    ifname: &str,
    quiet: bool,
    frames: &[TimedFrame],
    speed: f64,
    live: &LiveState,
    dump_tx: &Option<mpsc::SyncSender<CanFrame>>,
) -> (u64, u64) {
    let mut sent: u64 = 0;
    let mut errs: u64 = 0;
    let mut next = Instant::now();

    for tf in frames {
        if live.bus_off.load(Ordering::Relaxed) {
            errs += 1;
            live.errors.store(errs, Ordering::Relaxed);
            wait_while_bus_off(live);
            next = Instant::now();
        }

        let us = (tf.pre_delay_us as f64 / speed) as u64;
        next += Duration::from_micros(us);
        wait_until(next);

        if tf.can_id == 0 {
            continue; // delay-only marker
        }

        let frame = CanFrame {
            can_id: tf.can_id,
            can_dlc: 8,
            __pad: 0,
            __res0: 0,
            __res1: 0,
            data: tf.data,
        };
        match send_frame(*fd, &frame) {
            Ok(()) => {
                sent += 1;
                live.sent.store(sent, Ordering::Relaxed);
                if let Some(ref tx) = dump_tx {
                    let _ = tx.try_send(frame);
                }
            }
            Err(e) => {
                errs += 1;
                live.errors.store(errs, Ordering::Relaxed);
                if is_iface_down_err(&e) {
                    reconnect_socket(fd, ifname, quiet);
                    next = Instant::now();
                } else if errs <= 3 {
                    eprintln!("warning: write failed: {}", e);
                }
            }
        }
    }
    (sent, errs)
}

// ── UDS Flash runner ──────────────────────────────────────────────────────

fn run_uds_flash(
    fd: &mut i32,
    cli: &Cli,
    live: &LiveState,
    dump_tx: &Option<mpsc::SyncSender<CanFrame>>,
) {
    let tester = cli.tester_id;
    let ecu = cli.ecu_id;
    let speed = cli.speed;
    let errors = !cli.no_errors;
    let unlimited = cli.count == 0;
    let mut rng = Rng::new(cli.seed);

    if !cli.quiet {
        let blocks_str = if cli.transfer_blocks == 0 {
            "random".to_string()
        } else {
            cli.transfer_blocks.to_string()
        };
        eprintln!(
            "mcangen: UDS flash — iface={} tester=0x{:03X} ecu=0x{:03X} speed={:.1}x blocks={} errors={} obd={}",
            cli.interface, tester, ecu, speed, blocks_str,
            if errors { "on" } else { "off" },
            if cli.no_obd { "off" } else { "on" },
        );
    }

    let t_start = Instant::now();
    let mut sessions: u64 = 0;
    let mut total_sent: u64 = 0;
    let mut total_errs: u64 = 0;

    loop {
        if !unlimited && sessions >= cli.count {
            break;
        }
        sessions += 1;

        let num_blocks = if cli.transfer_blocks == 0 {
            rng.range_u32(50, 150) as usize
        } else {
            cli.transfer_blocks as usize
        };

        let uds_frames = gen_uds_session(tester, ecu, num_blocks, errors, &mut rng);
        let s_start = Instant::now();
        let (sent, errs) = play_timed_frames(
            fd,
            &cli.interface,
            cli.quiet,
            &uds_frames,
            speed,
            live,
            dump_tx,
        );
        total_sent += sent;
        total_errs += errs;

        if !cli.quiet {
            eprintln!(
                "mcangen: session #{} — {} blocks, {} frames in {:.1}s",
                sessions,
                num_blocks,
                sent,
                s_start.elapsed().as_secs_f64(),
            );
        }

        // OBD-II polling between sessions
        if !cli.no_obd && (unlimited || sessions < cli.count) {
            let obd_cycles = rng.range_u32(15, 30) as usize;
            let obd_frames = gen_obd_polling(ecu, obd_cycles, &mut rng);
            let (s, e) = play_timed_frames(
                fd,
                &cli.interface,
                cli.quiet,
                &obd_frames,
                speed,
                live,
                dump_tx,
            );
            total_sent += s;
            total_errs += e;

            let pause = Duration::from_secs_f64(rng.uniform(1.0, 2.0) / speed);
            std::thread::sleep(pause);
        }
    }

    if !cli.quiet {
        let secs = t_start.elapsed().as_secs_f64();
        eprintln!(
            "mcangen: done — {} sessions, {} frames in {:.1}s ({} errors)",
            sessions, total_sent, secs, total_errs,
        );
    }
}

// ── Live monitoring ───────────────────────────────────────────────────────

struct LiveState {
    sent: AtomicU64,
    errors: AtomicU64,
    bus_off: AtomicBool,
    running: AtomicBool,
}

impl LiveState {
    fn new() -> Arc<Self> {
        Arc::new(Self {
            sent: AtomicU64::new(0),
            errors: AtomicU64::new(0),
            bus_off: AtomicBool::new(false),
            running: AtomicBool::new(true),
        })
    }
}

/// Close the monitor socket and reopen it once the iface is admin-up again,
/// with the same 100ms→30s backoff as the send-path reconnect. Returns
/// `false` if the run was asked to stop while we were waiting.
fn err_socket_reopen(fd: &mut i32, ifname: &str, state: &LiveState) -> bool {
    unsafe {
        libc::close(*fd);
    }
    *fd = -1;
    let mut delay = Duration::from_millis(100);
    let max_delay = Duration::from_secs(30);
    loop {
        if !state.running.load(Ordering::Relaxed) {
            return false;
        }
        match open_err_socket(ifname) {
            Ok(nfd) if iface_is_up(ifname) == Some(true) => {
                *fd = nfd;
                return true;
            }
            Ok(nfd) => unsafe {
                libc::close(nfd);
            },
            Err(_) => {}
        }
        std::thread::sleep(delay);
        delay = (delay * 2).min(max_delay);
    }
}

/// Watches the CAN controller state by reading error frames from a dedicated
/// socket. Flips `state.bus_off` on CAN_ERR_BUSOFF / CAN_ERR_RESTARTED.
/// Self-heals across interface outages (close + reopen with the same
/// 100ms→30s backoff as the send path). On a successful reopen we
/// optimistically clear `bus_off` — if the controller is still off we will
/// re-receive the BUSOFF error frame and flip it back on.
///
/// When `auto_restart` is set, BUS-OFF triggers a netlink down/up cycle of
/// the iface (working around drivers that don't implement do_set_mode, e.g.
/// gs_usb). On EPERM the flag is disabled for the rest of the run with a
/// one-shot warning. Repeated rapid restarts back off exponentially so a
/// permanently bad bus can't pin the CPU.
fn err_monitor_thread(state: Arc<LiveState>, ifname: String, quiet: bool, auto_restart: bool) {
    let mut fd = match open_err_socket(&ifname) {
        Ok(fd) => fd,
        Err(e) => {
            if !quiet {
                eprintln!(
                    "warning: bus-state monitor disabled for '{}' ({}) — BUS-OFF will not be detected",
                    ifname, e,
                );
            }
            return;
        }
    };

    let mut auto_restart = auto_restart;
    let mut next_restart_delay = Duration::from_millis(200);
    let mut last_restart_time: Option<Instant> = None;

    let mut frame: CanFrame = unsafe { mem::zeroed() };
    while state.running.load(Ordering::Relaxed) {
        let n = unsafe {
            libc::read(
                fd,
                &mut frame as *mut CanFrame as *mut libc::c_void,
                mem::size_of::<CanFrame>(),
            )
        };
        if n < 0 {
            let e = io::Error::last_os_error();
            match e.raw_os_error() {
                Some(code) if code == libc::EAGAIN || code == libc::EWOULDBLOCK || code == libc::EINTR => {
                    continue;
                }
                _ => {}
            }
            if is_iface_down_err(&e) {
                state.bus_off.store(false, Ordering::Relaxed);
                if !err_socket_reopen(&mut fd, &ifname, &state) {
                    return;
                }
                continue;
            }
            if !quiet {
                eprintln!("warning: bus-state monitor terminated — {}", e);
            }
            break;
        }

        if (n as usize) < mem::size_of::<CanFrame>() {
            continue;
        }
        if frame.can_id & CAN_ERR_FLAG == 0 {
            // RX filter is empty so this shouldn't happen, but ignore if it does.
            continue;
        }
        let err_class = frame.can_id & !CAN_ERR_FLAG;
        if err_class & CAN_ERR_BUSOFF != 0 {
            let was_off = state.bus_off.swap(true, Ordering::Relaxed);
            if !was_off && !quiet {
                let suffix = if auto_restart {
                    " (--auto-restart will cycle the interface)"
                } else {
                    " (recover with: ip link set <iface> type can restart, or set restart-ms > 0)"
                };
                eprintln!(
                    "\nmcangen: BUS-OFF on '{}' — pausing transmission{}",
                    ifname, suffix,
                );
            }

            if auto_restart {
                // Adaptive backoff: if the previous restart was long enough
                // ago, treat this as a fresh event; otherwise keep doubling.
                let now = Instant::now();
                match last_restart_time {
                    Some(t) if now.duration_since(t) < Duration::from_secs(30) => {
                        next_restart_delay = (next_restart_delay * 2).min(Duration::from_secs(60));
                    }
                    _ => {
                        next_restart_delay = Duration::from_millis(200);
                    }
                }
                std::thread::sleep(next_restart_delay);
                last_restart_time = Some(Instant::now());

                match cycle_iface(&ifname) {
                    Ok(()) => {
                        if !quiet {
                            eprintln!(
                                "mcangen: cycled '{}' via netlink — re-attaching",
                                ifname
                            );
                        }
                        // The iface bounce invalidates our monitor socket.
                        // Re-open via the standard reopen helper, which also
                        // waits for IFF_UP. Optimistically clear bus_off; if
                        // the bus is still bad we'll get another BUSOFF frame
                        // shortly and flip back.
                        state.bus_off.store(false, Ordering::Relaxed);
                        if !err_socket_reopen(&mut fd, &ifname, &state) {
                            return;
                        }
                    }
                    Err(e) if e.raw_os_error() == Some(libc::EPERM) => {
                        if !quiet {
                            eprintln!(
                                "mcangen: --auto-restart needs CAP_NET_ADMIN; disabling for this run (try: sudo setcap cap_net_admin,cap_net_raw=eip <mcangen-binary>)"
                            );
                        }
                        auto_restart = false;
                    }
                    Err(e) => {
                        if !quiet {
                            eprintln!(
                                "mcangen: --auto-restart cycle of '{}' failed: {}",
                                ifname, e
                            );
                        }
                    }
                }
            }
        }
        if err_class & CAN_ERR_RESTARTED != 0 && state.bus_off.swap(false, Ordering::Relaxed) && !quiet {
            eprintln!("\nmcangen: bus restarted on '{}' — resuming", ifname);
        }
    }
    unsafe {
        libc::close(fd);
    }
}

/// Block while `state.bus_off` is set, using a short exponential backoff
/// (100ms → 5s). Returns `true` if we actually waited (caller should reset
/// rate-limit baselines), `false` if the bus was online from the start.
fn wait_while_bus_off(state: &LiveState) -> bool {
    if !state.bus_off.load(Ordering::Relaxed) {
        return false;
    }
    let mut delay = Duration::from_millis(100);
    let max_delay = Duration::from_secs(5);
    while state.bus_off.load(Ordering::Relaxed) && state.running.load(Ordering::Relaxed) {
        std::thread::sleep(delay);
        delay = (delay * 2).min(max_delay);
    }
    true
}

fn stats_thread(state: Arc<LiveState>, interface: String) {
    let start = Instant::now();
    let mut prev_sent: u64 = 0;

    while state.running.load(Ordering::Relaxed) {
        std::thread::sleep(Duration::from_secs(1));
        if !state.running.load(Ordering::Relaxed) {
            break;
        }
        let sent = state.sent.load(Ordering::Relaxed);
        let errors = state.errors.load(Ordering::Relaxed);
        let elapsed = start.elapsed().as_secs_f64();
        let instant_fps = (sent - prev_sent) as f64;
        let avg_fps = if elapsed > 0.0 {
            sent as f64 / elapsed
        } else {
            0.0
        };
        prev_sent = sent;

        let state_tag = if state.bus_off.load(Ordering::Relaxed) {
            " | BUS-OFF"
        } else {
            ""
        };
        eprint!(
            "\r{}: {} frames | {:.0} fps (avg {:.0}) | {} err | {:.1}s{}    ",
            interface, sent, instant_fps, avg_fps, errors, elapsed, state_tag,
        );
    }
    eprintln!();
}

fn dump_thread(rx: mpsc::Receiver<CanFrame>, interface: String) {
    use std::io::Write;

    while let Ok(frame) = rx.recv() {
        let is_ext = frame.can_id & CAN_EFF_FLAG != 0;
        let id = frame.can_id & if is_ext { CAN_EFF_MASK } else { CAN_SFF_MASK };
        let dlc = frame.can_dlc as usize;

        let mut line = String::with_capacity(64);
        if is_ext {
            std::fmt::Write::write_fmt(
                &mut line,
                format_args!("  {}  {:08X}   [{}]", interface, id, dlc),
            )
            .unwrap();
        } else {
            std::fmt::Write::write_fmt(
                &mut line,
                format_args!("  {}  {:03X}   [{}]", interface, id, dlc),
            )
            .unwrap();
        }
        for i in 0..dlc {
            std::fmt::Write::write_fmt(&mut line, format_args!(" {:02X}", frame.data[i])).unwrap();
        }

        let stdout = std::io::stdout();
        let mut out = stdout.lock();
        let _ = writeln!(out, "{}", line);
    }
}

// ── Main ───────────────────────────────────────────────────────────────────

fn main() {
    let cli = Cli::parse();

    // Validate
    if cli.dlc_min > cli.dlc_max {
        eprintln!(
            "error: --dlc-min ({}) must be <= --dlc-max ({})",
            cli.dlc_min, cli.dlc_max
        );
        std::process::exit(1);
    }

    if cli.burst {
        if cli.burst_high_rate == 0 {
            eprintln!("error: --burst-high-rate must be > 0");
            std::process::exit(1);
        }
        if cli.burst_low_rate == 0 {
            eprintln!("error: --burst-low-rate must be > 0");
            std::process::exit(1);
        }
        if cli.fps > 0 {
            eprintln!("error: --rate and --burst are mutually exclusive; use --burst-high-rate and --burst-low-rate");
            std::process::exit(1);
        }
    }

    if cli.uds_flash && cli.burst {
        eprintln!("error: --uds-flash and --burst are mutually exclusive");
        std::process::exit(1);
    }

    if cli.uds_flash && cli.speed <= 0.0 {
        eprintln!("error: --speed must be positive");
        std::process::exit(1);
    }

    // Quality-test protocol requires exactly 8 data bytes
    let dlc_min = if matches!(cli.data_mode, DataMode::QualityTest) {
        8
    } else {
        cli.dlc_min
    };
    let dlc_max = if matches!(cli.data_mode, DataMode::QualityTest) {
        8
    } else {
        cli.dlc_max
    };

    let id_ceiling = match cli.id_kind {
        IdKind::Standard => CAN_SFF_MASK,
        IdKind::Extended | IdKind::Mixed => CAN_EFF_MASK,
    };
    let (id_min, id_max) = if let Some(id) = cli.id {
        let id = id.min(id_ceiling);
        (id, id)
    } else {
        let id_min = cli.id_min.unwrap_or(0).min(id_ceiling);
        let id_max = cli.id_max.unwrap_or(id_ceiling).min(id_ceiling);
        (id_min, id_max)
    };
    if id_min > id_max {
        eprintln!(
            "error: --id-min (0x{:X}) must be <= --id-max (0x{:X})",
            id_min, id_max
        );
        std::process::exit(1);
    }

    let unlimited = cli.count == 0;
    let max_rate = cli.fps == 0;

    let mut fd = open_can_socket(&cli.interface).unwrap_or_else(|e| {
        eprintln!(
            "error: failed to open CAN socket on '{}': {}",
            cli.interface, e
        );
        eprintln!("hint: make sure the interface exists (ip link show) and you have CAP_NET_RAW");
        std::process::exit(1);
    });

    // ── Live monitoring threads ──
    let live = LiveState::new();

    // Bus-state monitor: listens for CAN error frames so we know when the
    // controller goes BUS-OFF (which is invisible to write()).
    let err_monitor_handle = {
        let s = Arc::clone(&live);
        let iface = cli.interface.clone();
        let quiet = cli.quiet;
        let auto_restart = cli.auto_restart;
        Some(
            std::thread::Builder::new()
                .name("can-err".into())
                .spawn(move || err_monitor_thread(s, iface, quiet, auto_restart))
                .unwrap(),
        )
    };

    let show_stats = !cli.quiet && !cli.dump && unsafe { libc::isatty(libc::STDERR_FILENO) == 1 };
    let stats_handle = if show_stats {
        let s = Arc::clone(&live);
        let iface = cli.interface.clone();
        Some(
            std::thread::Builder::new()
                .name("stats".into())
                .spawn(move || stats_thread(s, iface))
                .unwrap(),
        )
    } else {
        None
    };

    let dump_tx: Option<mpsc::SyncSender<CanFrame>> = if cli.dump {
        let (tx, rx) = mpsc::sync_channel::<CanFrame>(4096);
        let iface = cli.interface.clone();
        std::thread::Builder::new()
            .name("dump".into())
            .spawn(move || dump_thread(rx, iface))
            .unwrap();
        Some(tx)
    } else {
        None
    };

    if cli.uds_flash {
        run_uds_flash(&mut fd, &cli, &live, &dump_tx);
        live.running.store(false, Ordering::Relaxed);
        drop(dump_tx);
        if let Some(h) = stats_handle {
            let _ = h.join();
        }
        if let Some(h) = err_monitor_handle {
            let _ = h.join();
        }
        unsafe {
            libc::close(fd);
        }
        return;
    }

    let interval = if max_rate {
        Duration::ZERO
    } else {
        Duration::from_secs_f64(1.0 / cli.fps as f64)
    };

    let mut rng = Rng::new(cli.seed);
    let mut frame: CanFrame = unsafe { mem::zeroed() };
    let mut seq_id = id_min;
    let mut counter: u8 = 0;
    let mut seqnum: u64 = 0;
    let mut qt_seqnum: u16 = 0;
    let mut sent: u64 = 0;
    let mut errors: u64 = 0;

    // Burst mode timing
    let burst_high_interval = if cli.burst {
        Duration::from_secs_f64(1.0 / cli.burst_high_rate as f64)
    } else {
        Duration::ZERO
    };
    let burst_low_interval = if cli.burst {
        Duration::from_secs_f64(1.0 / cli.burst_low_rate as f64)
    } else {
        Duration::ZERO
    };
    let burst_high_dur = Duration::from_millis(cli.burst_high_ms);
    let burst_low_dur = Duration::from_millis(cli.burst_low_ms);
    let mut burst_phase_high = true;
    let mut burst_phase_start: Instant;

    if !cli.quiet {
        let rate_str = if cli.burst {
            format!(
                "burst {}/{} fps ({}ms/{}ms)",
                cli.burst_high_rate, cli.burst_low_rate, cli.burst_high_ms, cli.burst_low_ms,
            )
        } else if max_rate {
            "max".to_string()
        } else {
            format!("{} fps", cli.fps)
        };
        eprintln!(
            "mcangen: iface={} count={} rate={} id_kind={:?} id_mode={:?} data={:?} ids=0x{:X}..0x{:X} dlc={}..{}",
            cli.interface,
            if unlimited { "unlimited".to_string() } else { cli.count.to_string() },
            rate_str,
            cli.id_kind,
            cli.id_mode,
            cli.data_mode,
            id_min,
            id_max,
            dlc_min,
            dlc_max,
        );
    }

    let t_start = Instant::now();
    let mut next_send = t_start;
    burst_phase_start = t_start;

    // ── Max-rate path: use sendmmsg batching for throughput ───────────
    if max_rate {
        let mut batch: [CanFrame; BATCH_SIZE] = [frame; BATCH_SIZE];

        'max_rate: loop {
            if live.bus_off.load(Ordering::Relaxed) {
                // Frames sent while BUS-OFF wouldn't reach the wire — pause
                // and count an error rather than lying about throughput.
                errors += 1;
                live.errors.store(errors, Ordering::Relaxed);
                wait_while_bus_off(&live);
                continue;
            }

            // Fill a batch of frames
            let batch_count = if unlimited {
                BATCH_SIZE
            } else {
                BATCH_SIZE.min((cli.count - sent) as usize)
            };
            if batch_count == 0 {
                break;
            }

            for bf in batch.iter_mut().take(batch_count) {
                let raw_id = match cli.id_mode {
                    IdMode::Random => rng.range_u32(id_min, id_max),
                    IdMode::Sequential => {
                        let id = seq_id;
                        seq_id = if seq_id >= id_max { id_min } else { seq_id + 1 };
                        id
                    }
                };
                let use_extended = match cli.id_kind {
                    IdKind::Standard => false,
                    IdKind::Extended => true,
                    IdKind::Mixed => rng.next() & 1 == 0,
                };
                bf.can_id = if use_extended {
                    let mut id = raw_id & CAN_EFF_MASK;
                    if cli.ext_id_above_sff && id <= CAN_SFF_MASK {
                        id = CAN_SFF_MASK + 1 + (id % (CAN_EFF_MASK - CAN_SFF_MASK));
                    }
                    id | CAN_EFF_FLAG
                } else {
                    raw_id & CAN_SFF_MASK
                };
                bf.can_dlc = rng.range_u8(dlc_min, dlc_max).min(CAN_MAX_DLC);
                match cli.data_mode {
                    DataMode::Random => {
                        let r1 = rng.next();
                        bf.data[..8].copy_from_slice(&r1.to_ne_bytes());
                    }
                    DataMode::Zero => bf.data = [0u8; 8],
                    DataMode::Ones => bf.data = [0xFFu8; 8],
                    DataMode::Counter => {
                        bf.data = [counter; 8];
                        counter = counter.wrapping_add(1);
                    }
                    DataMode::Sequence => {
                        bf.data[..8].copy_from_slice(&seqnum.to_be_bytes());
                        seqnum = seqnum.wrapping_add(1);
                    }
                    DataMode::QualityTest => {
                        let elapsed_ms = t_start.elapsed().as_millis() as u16;
                        bf.data[0] = 0xCA;
                        bf.data[1] = 0xFE;
                        bf.data[2..4].copy_from_slice(&qt_seqnum.to_be_bytes());
                        bf.data[4..6].copy_from_slice(&elapsed_ms.to_be_bytes());
                        bf.data[6] = cli.test_id;
                        bf.data[7] = bf.data[..7].iter().fold(0u8, |acc, &b| acc ^ b);
                        qt_seqnum = qt_seqnum.wrapping_add(1);
                    }
                }
            }

            // Send the batch
            match send_frames_batch(fd, &batch[..batch_count]) {
                Ok(n) => {
                    sent += n as u64;
                    live.sent.store(sent, Ordering::Relaxed);
                    if let Some(ref tx) = dump_tx {
                        for bf in &batch[..n] {
                            let _ = tx.try_send(*bf);
                        }
                    }
                }
                Err(e) => {
                    if is_iface_down_err(&e) {
                        errors += 1;
                        live.errors.store(errors, Ordering::Relaxed);
                        reconnect_socket(&mut fd, &cli.interface, cli.quiet);
                        continue;
                    }
                    // Fallback: send individually on batch failure
                    let mut iface_lost = false;
                    for f in &batch[..batch_count] {
                        match send_frame(fd, f) {
                            Ok(()) => {
                                sent += 1;
                                live.sent.store(sent, Ordering::Relaxed);
                                if let Some(ref tx) = dump_tx {
                                    let _ = tx.try_send(*f);
                                }
                            }
                            Err(e) => {
                                errors += 1;
                                live.errors.store(errors, Ordering::Relaxed);
                                if is_iface_down_err(&e) {
                                    reconnect_socket(&mut fd, &cli.interface, cli.quiet);
                                    iface_lost = true;
                                    break;
                                }
                                if e.raw_os_error() != Some(libc::ENOBUFS) && errors > 100 {
                                    eprintln!("error: too many write errors, aborting");
                                    break 'max_rate;
                                }
                            }
                        }
                    }
                    if iface_lost {
                        continue;
                    }
                    if errors <= 5 {
                        eprintln!("warning: sendmmsg failed, falling back: {}", e);
                    }
                }
            }

            if cli.progress > 0 && sent >= cli.progress && !cli.quiet {
                let prev = sent - batch_count as u64;
                if prev / cli.progress < sent / cli.progress {
                    let elapsed = t_start.elapsed().as_secs_f64();
                    eprintln!(
                        "  sent {} frames in {:.2}s ({:.0} fps)",
                        sent,
                        elapsed,
                        sent as f64 / elapsed,
                    );
                }
            }
        }
    } else {
        // ── Rate-limited / burst path: one frame at a time ───────────────
        loop {
            if !unlimited && sent >= cli.count {
                break;
            }

            if live.bus_off.load(Ordering::Relaxed) {
                errors += 1;
                live.errors.store(errors, Ordering::Relaxed);
                wait_while_bus_off(&live);
                next_send = Instant::now();
                if cli.burst {
                    burst_phase_start = next_send;
                }
                continue;
            }

            // ── ID ──
            let raw_id = match cli.id_mode {
                IdMode::Random => rng.range_u32(id_min, id_max),
                IdMode::Sequential => {
                    let id = seq_id;
                    seq_id = if seq_id >= id_max { id_min } else { seq_id + 1 };
                    id
                }
            };

            // Decide if this frame is extended
            let use_extended = match cli.id_kind {
                IdKind::Standard => false,
                IdKind::Extended => true,
                IdKind::Mixed => rng.next() & 1 == 0,
            };

            frame.can_id = if use_extended {
                let mut id = raw_id & CAN_EFF_MASK;
                if cli.ext_id_above_sff && id <= CAN_SFF_MASK {
                    id = CAN_SFF_MASK + 1 + (id % (CAN_EFF_MASK - CAN_SFF_MASK));
                }
                id | CAN_EFF_FLAG
            } else {
                raw_id & CAN_SFF_MASK
            };

            // ── DLC ──
            frame.can_dlc = rng.range_u8(dlc_min, dlc_max).min(CAN_MAX_DLC);

            // ── Data ──
            match cli.data_mode {
                DataMode::Random => {
                    let r1 = rng.next();
                    frame.data[..8].copy_from_slice(&r1.to_ne_bytes());
                }
                DataMode::Zero => {
                    frame.data = [0u8; 8];
                }
                DataMode::Ones => {
                    frame.data = [0xFFu8; 8];
                }
                DataMode::Counter => {
                    frame.data = [counter; 8];
                    counter = counter.wrapping_add(1);
                }
                DataMode::Sequence => {
                    frame.data[..8].copy_from_slice(&seqnum.to_be_bytes());
                    seqnum = seqnum.wrapping_add(1);
                }
                DataMode::QualityTest => {
                    let elapsed_ms = t_start.elapsed().as_millis() as u16;
                    frame.data[0] = 0xCA;
                    frame.data[1] = 0xFE;
                    frame.data[2..4].copy_from_slice(&qt_seqnum.to_be_bytes());
                    frame.data[4..6].copy_from_slice(&elapsed_ms.to_be_bytes());
                    frame.data[6] = cli.test_id;
                    frame.data[7] = frame.data[..7].iter().fold(0u8, |acc, &b| acc ^ b);
                    qt_seqnum = qt_seqnum.wrapping_add(1);
                }
            }

            // ── Rate limiting ──
            if cli.burst {
                // Check for phase transition
                let phase_dur = if burst_phase_high {
                    burst_high_dur
                } else {
                    burst_low_dur
                };
                if burst_phase_start.elapsed() >= phase_dur {
                    burst_phase_high = !burst_phase_high;
                    burst_phase_start = Instant::now();
                    next_send = burst_phase_start;
                }

                let current_interval = if burst_phase_high {
                    burst_high_interval
                } else {
                    burst_low_interval
                };
                wait_until(next_send);
                next_send += current_interval;
                let now = Instant::now();
                if next_send < now {
                    next_send = now + current_interval;
                }
            } else {
                wait_until(next_send);
                next_send += interval;
                let now = Instant::now();
                if next_send < now {
                    next_send = now + interval;
                }
            }

            // ── Send ──
            match send_frame(fd, &frame) {
                Ok(()) => {
                    sent += 1;
                    live.sent.store(sent, Ordering::Relaxed);
                    if let Some(ref tx) = dump_tx {
                        let _ = tx.try_send(frame);
                    }
                }
                Err(e) => {
                    errors += 1;
                    live.errors.store(errors, Ordering::Relaxed);
                    if is_iface_down_err(&e) {
                        reconnect_socket(&mut fd, &cli.interface, cli.quiet);
                        // Reset rate-limit baselines so we don't catch up missed frames
                        next_send = Instant::now();
                        if cli.burst {
                            burst_phase_start = next_send;
                        }
                    } else {
                        if errors <= 5 {
                            eprintln!("warning: write failed: {}", e);
                        }
                        if errors == 5 {
                            eprintln!("warning: suppressing further write errors");
                        }
                        // ENOBUFS is transient on busy interfaces — keep going
                        if e.raw_os_error() != Some(libc::ENOBUFS) && errors > 100 {
                            eprintln!("error: too many write errors, aborting");
                            break;
                        }
                    }
                }
            }

            // ── Progress ──
            if cli.progress > 0 && sent.is_multiple_of(cli.progress) && sent > 0 && !cli.quiet {
                let elapsed = t_start.elapsed().as_secs_f64();
                eprintln!(
                    "  sent {} frames in {:.2}s ({:.0} fps)",
                    sent,
                    elapsed,
                    sent as f64 / elapsed,
                );
            }
        }
    } // else (rate-limited)

    let elapsed = t_start.elapsed();

    live.running.store(false, Ordering::Relaxed);
    drop(dump_tx);
    if let Some(h) = stats_handle {
        let _ = h.join();
    }
    if let Some(h) = err_monitor_handle {
        let _ = h.join();
    }

    unsafe {
        libc::close(fd);
    }

    if !cli.quiet {
        let secs = elapsed.as_secs_f64();
        eprintln!(
            "mcangen: done — {} frames in {:.3}s ({:.0} fps, {} errors)",
            sent,
            secs,
            if secs > 0.0 { sent as f64 / secs } else { 0.0 },
            errors,
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::{Duration, Instant};

    /// Try to open a CAN socket on can0.  Returns None if unavailable
    /// (no interface, no permissions), so tests can skip gracefully.
    fn try_open_can0() -> Option<i32> {
        open_can_socket("can0").ok()
    }

    // ── Correctness: send_frame delivers a valid frame ───────────────

    #[test]
    fn test_quality_test_requires_exact_id() {
        let err = Cli::try_parse_from(["mcangen", "vcan0", "--data-mode", "quality-test"])
            .expect_err("quality-test mode must require --id");
        let msg = err.to_string();
        assert!(
            msg.contains("--id"),
            "expected clap error to mention --id, got: {msg}"
        );
    }

    #[test]
    fn test_quality_test_accepts_exact_id() {
        let cli = Cli::try_parse_from([
            "mcangen",
            "vcan0",
            "--data-mode",
            "quality-test",
            "--id",
            "0x7E0",
        ])
        .expect("quality-test mode should accept --id");
        assert_eq!(cli.id, Some(0x7E0));
    }

    #[test]
    fn test_exact_id_conflicts_with_id_range() {
        let err = Cli::try_parse_from(["mcangen", "vcan0", "--id", "0x123", "--id-min", "0x100"])
            .expect_err("--id should conflict with --id-min");
        let msg = err.to_string();
        assert!(
            msg.contains("--id") && msg.contains("--id-min"),
            "expected clap conflict error to mention --id and --id-min, got: {msg}"
        );
    }

    #[test]
    fn test_send_frame_correctness() {
        let fd = match try_open_can0() {
            Some(fd) => fd,
            None => {
                eprintln!("SKIP: can0 not available");
                return;
            }
        };

        let frame = CanFrame {
            can_id: 0x123,
            can_dlc: 8,
            __pad: 0,
            __res0: 0,
            __res1: 0,
            data: [0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0xBA, 0xBE],
        };

        // Should succeed without error
        let result = send_frame(fd, &frame);
        unsafe { libc::close(fd) };
        assert!(result.is_ok(), "send_frame failed: {:?}", result.err());
    }

    // ── Correctness: send_frames_batch delivers all frames ───────────

    #[test]
    fn test_send_frames_batch_correctness() {
        let fd = match try_open_can0() {
            Some(fd) => fd,
            None => {
                eprintln!("SKIP: can0 not available");
                return;
            }
        };

        let mut frames = [CanFrame {
            can_id: 0,
            can_dlc: 8,
            __pad: 0,
            __res0: 0,
            __res1: 0,
            data: [0; 8],
        }; BATCH_SIZE];

        for (i, f) in frames.iter_mut().enumerate() {
            f.can_id = (i as u32) & CAN_SFF_MASK;
            f.data[0] = i as u8;
        }

        let result = send_frames_batch(fd, &frames);
        unsafe { libc::close(fd) };
        match result {
            Ok(n) => assert_eq!(n, BATCH_SIZE, "expected {BATCH_SIZE} frames sent, got {n}"),
            Err(e) => panic!("send_frames_batch failed: {e}"),
        }
    }

    // ── Correctness: batch count matches requested ───────────────────

    #[test]
    fn test_send_frames_batch_partial() {
        let fd = match try_open_can0() {
            Some(fd) => fd,
            None => {
                eprintln!("SKIP: can0 not available");
                return;
            }
        };

        let frames: Vec<CanFrame> = (0..7)
            .map(|i| CanFrame {
                can_id: i as u32,
                can_dlc: 1,
                __pad: 0,
                __res0: 0,
                __res1: 0,
                data: [i as u8; 8],
            })
            .collect();

        let result = send_frames_batch(fd, &frames);
        unsafe { libc::close(fd) };
        match result {
            Ok(n) => assert_eq!(n, 7, "expected 7 frames sent, got {n}"),
            Err(e) => panic!("send_frames_batch failed: {e}"),
        }
    }

    // ── Performance: sendmmsg vs write throughput ─────────────────────

    #[test]
    fn bench_sendmmsg_vs_write() {
        let fd = match try_open_can0() {
            Some(fd) => fd,
            None => {
                eprintln!("SKIP: can0 not available");
                return;
            }
        };

        let total_frames: usize = 100_000;
        let frame = CanFrame {
            can_id: 0x7FF,
            can_dlc: 8,
            __pad: 0,
            __res0: 0,
            __res1: 0,
            data: [0xAA; 8],
        };

        // ── Benchmark: individual write() calls ──
        let start = Instant::now();
        let mut sent_write: usize = 0;
        for _ in 0..total_frames {
            if send_frame(fd, &frame).is_ok() {
                sent_write += 1;
            }
        }
        let write_elapsed = start.elapsed();

        // ── Benchmark: sendmmsg() batching ──
        let batch = [frame; BATCH_SIZE];
        let start = Instant::now();
        let mut sent_batch: usize = 0;
        while sent_batch < total_frames {
            let remaining = total_frames - sent_batch;
            let count = remaining.min(BATCH_SIZE);
            match send_frames_batch(fd, &batch[..count]) {
                Ok(n) => sent_batch += n,
                Err(_) => break,
            }
        }
        let batch_elapsed = start.elapsed();

        unsafe { libc::close(fd) };

        let write_fps = sent_write as f64 / write_elapsed.as_secs_f64();
        let batch_fps = sent_batch as f64 / batch_elapsed.as_secs_f64();
        let speedup = batch_fps / write_fps;

        eprintln!("\n=== sendmmsg vs write benchmark ({total_frames} frames) ===");
        eprintln!(
            "  write():    {sent_write} frames in {:.3}s = {write_fps:.0} fps",
            write_elapsed.as_secs_f64()
        );
        eprintln!(
            "  sendmmsg(): {sent_batch} frames in {:.3}s = {batch_fps:.0} fps",
            batch_elapsed.as_secs_f64()
        );
        eprintln!("  speedup:    {speedup:.2}x");

        // sendmmsg should be at least as fast as write (no regression)
        assert!(
            speedup >= 0.95,
            "sendmmsg was significantly slower than write: {speedup:.2}x"
        );
        // On vcan, expect at least some improvement
        if speedup > 1.05 {
            eprintln!("  PASS: sendmmsg is {speedup:.2}x faster");
        } else {
            eprintln!("  NOTE: similar performance (expected on virtual CAN)");
        }
    }

    // ── Precision: wait_until with clock_nanosleep ───────────────────

    #[test]
    fn test_wait_until_precision() {
        let intervals = [
            Duration::from_micros(200), // 5000 fps
            Duration::from_millis(1),   // 1000 fps
            Duration::from_millis(5),   // 200 fps
            Duration::from_millis(20),  // 50 fps
        ];

        eprintln!("\n=== wait_until precision test ===");

        for &interval in &intervals {
            let iterations = 500;
            let mut max_overshoot = Duration::ZERO;
            let mut total_overshoot = Duration::ZERO;

            let mut target = Instant::now() + interval;
            for _ in 0..iterations {
                wait_until(target);
                let now = Instant::now();
                let overshoot = now.duration_since(target);
                total_overshoot += overshoot;
                if overshoot > max_overshoot {
                    max_overshoot = overshoot;
                }
                target += interval;
                // Reset if we fell behind
                if target < now {
                    target = now + interval;
                }
            }

            let avg_us = total_overshoot.as_micros() as f64 / iterations as f64;
            let max_us = max_overshoot.as_micros();
            let interval_us = interval.as_micros();

            eprintln!(
                "  interval={interval_us:>6}µs  avg_overshoot={avg_us:>6.1}µs  max_overshoot={max_us:>6}µs"
            );

            // Average overshoot should be well under 1ms
            assert!(
                avg_us < 500.0,
                "avg overshoot {avg_us:.1}µs too high for {interval_us}µs interval"
            );
            // Max overshoot should be under 2ms (scheduling jitter)
            assert!(
                max_us < 2000,
                "max overshoot {max_us}µs too high for {interval_us}µs interval"
            );
        }
    }
}
