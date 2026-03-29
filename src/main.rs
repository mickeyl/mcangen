use clap::{Parser, ValueEnum};
use std::io;
use std::mem;
use std::time::{Duration, Instant};

// ── SocketCAN constants & structs ──────────────────────────────────────────

const AF_CAN: i32 = 29;
const PF_CAN: i32 = AF_CAN;
const CAN_RAW: i32 = 1;
// libc::Ioctl is c_ulong on glibc, c_int on musl — use the alias for portability.
const SIOCGIFINDEX: libc::Ioctl = 0x8933 as libc::Ioctl;

const CAN_EFF_FLAG: u32 = 0x8000_0000;
const CAN_SFF_MASK: u32 = 0x0000_07FF;
const CAN_EFF_MASK: u32 = 0x1FFF_FFFF;
const CAN_MAX_DLC: u8 = 8;

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
    #[arg(long, default_value = "0x000", value_parser = parse_hex_u32)]
    id_min: u32,

    /// Maximum CAN ID (hex or decimal). Defaults to 0x7FF (standard) or 0x1FFFFFFF (extended/mixed).
    #[arg(long, value_parser = parse_hex_u32)]
    id_max: Option<u32>,

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
}

// ── Precise rate-limited sleep ─────────────────────────────────────────────

/// Busy-spin until `target` time is reached. For intervals > 1ms, sleeps for
/// most of the duration then busy-spins the remainder for precision.
#[inline]
fn wait_until(target: Instant) {
    let now = Instant::now();
    if now >= target {
        return;
    }
    let remaining = target - now;
    // For long waits, sleep most of it to avoid wasting CPU
    if remaining > Duration::from_millis(2) {
        std::thread::sleep(remaining - Duration::from_millis(1));
    }
    // Busy-spin the final stretch for accuracy
    while Instant::now() < target {
        std::hint::spin_loop();
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
    let id_min = cli.id_min.min(id_ceiling);
    let id_max = cli.id_max.unwrap_or(id_ceiling).min(id_ceiling);
    if id_min > id_max {
        eprintln!(
            "error: --id-min (0x{:X}) must be <= --id-max (0x{:X})",
            id_min, id_max
        );
        std::process::exit(1);
    }

    let unlimited = cli.count == 0;
    let max_rate = cli.fps == 0;

    let fd = open_can_socket(&cli.interface).unwrap_or_else(|e| {
        eprintln!(
            "error: failed to open CAN socket on '{}': {}",
            cli.interface, e
        );
        eprintln!("hint: make sure the interface exists (ip link show) and you have CAP_NET_RAW");
        std::process::exit(1);
    });

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

    loop {
        if !unlimited && sent >= cli.count {
            break;
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
        } else if !max_rate {
            wait_until(next_send);
            next_send += interval;
            let now = Instant::now();
            if next_send < now {
                next_send = now + interval;
            }
        }

        // ── Send ──
        match send_frame(fd, &frame) {
            Ok(()) => sent += 1,
            Err(e) => {
                errors += 1;
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

        // ── Progress ──
        if cli.progress > 0 && sent % cli.progress == 0 && sent > 0 && !cli.quiet {
            let elapsed = t_start.elapsed().as_secs_f64();
            eprintln!(
                "  sent {} frames in {:.2}s ({:.0} fps)",
                sent,
                elapsed,
                sent as f64 / elapsed,
            );
        }
    }

    let elapsed = t_start.elapsed();

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
