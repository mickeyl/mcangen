<p align="center">
  <img src="assets/logo.svg" alt="mcangen logo" width="640">
</p>

# mcangen

[![CI](https://github.com/mickeyl/mcangen/actions/workflows/ci.yml/badge.svg)](https://github.com/mickeyl/mcangen/actions/workflows/ci.yml)
[![Release](https://github.com/mickeyl/mcangen/actions/workflows/release.yml/badge.svg)](https://github.com/mickeyl/mcangen/actions/workflows/release.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![crates.io](https://img.shields.io/crates/v/mcangen.svg)](https://crates.io/crates/mcangen)
![Platform: Linux](https://img.shields.io/badge/platform-Linux-blue)
![Rust](https://img.shields.io/badge/language-Rust-orange)

High-performance CAN bus frame generator for Linux, built in Rust.

## Why?

If you develop or test anything that touches a CAN bus — automotive ECUs,
J2534 passthru devices, OBD adapters, SocketCAN drivers, or CAN-to-IP
gateways — you need a way to throw traffic at it. The venerable `cangen`
from [can-utils](https://github.com/linux-can/can-utils) works, but it has
limitations:

- It can't hit high frame rates reliably — timing drifts at scale.
- Mixing standard and extended IDs in one run requires scripting.
- There's no built-in way to send an exact number of frames and stop, which
  makes automated test scripts awkward.
- There's no sequence-number mode to detect drops or reordering on the
  receiving end.

**mcangen** fixes all of that. It uses raw SocketCAN writes, a hybrid
sleep/busy-spin rate limiter, and a fast xorshift PRNG to generate diverse
CAN traffic at line rate or any precise target FPS — then stops at exactly
the count you asked for.

Typical use cases:

- **Benchmarking** a CAN interface, driver, or J2534 device at maximum
  throughput.
- **Stress testing** a receiver to find buffer overflows, dropped frames,
  or firmware crashes.
- **Verifying frame counts** — send exactly *N* frames, compare what
  arrived on the other side. (Hint: if your device counts a few more than
  you sent, that's CAN bus retransmissions from arbitration losses — not a
  bug.)
- **Regression testing** with reproducible traffic — same seed, same
  frames every time.

### End-to-end CAN diagnosis

The motivating diagnostic workflow uses more than a frame counter. Linux
generates deterministic quality traffic, a device under test validates and
optionally relays it on another CAN ID, and `mcandump` checks the complete
return path:

```bash
# Receiver/verifier: requests on 0x700, relay responses on 0x701
mcandump can1 --quiet --quality-test --quality-id 0x700 \
    --quality-response-id 0x701 --quality-test-id 1 --quality-strict

# Generator: 64-byte CAN-FD+BRS frames with sequence, timestamp, pattern, CRC32C
mcangen can1 --fd --brs --data-mode quality-test --id 0x700 \
    --test-id 1 -r 1000 -n 30000
```

With ESPenlaub hwtest, configure the MCU side with `twai quality relay 700
701 1`. The final report identifies missing, duplicate, reordered, corrupt,
and kernel-dropped frames and includes request/response round-trip latency.
Use an 8-byte Classic CAN quality stream when the controller does not support
CAN-FD.

## Features

- **Fast** — `sendmmsg()` batching in max-rate mode for reduced syscall
  overhead, `clock_nanosleep()` for precise rate control with minimal
  CPU usage, CAN-FD capable socket, release build with LTO
- **All frame types** — standard (11-bit) IDs, extended (29-bit) IDs, or a
  random mix of both
- **Configurable payload** — 0 to 8 bytes for Classic CAN, 0 to 64 bytes
  for CAN-FD
- **Data patterns** — random, zeros, ones (0xFF), incrementing counter,
  64-bit big-endian sequence number, or content-aware Classic/CAN-FD
  quality-test protocols on an exact CAN ID via `--id`
- **UDS flash simulation** — realistic ECU reprogramming session with
  proper ISO-TP framing, security access, memory erase, firmware transfer,
  DTC handling, and ECU reset — both tester and ECU sides on the bus
- **Burst mode** — alternating high/low rate phases to emulate ECU
  reprogramming traffic patterns
- **Precise rate control** — `clock_nanosleep` with a final busy-spin
  for sub-microsecond accuracy from 1 fps to line rate
- **Exact counts** — send a precise number of frames then stop
- **Reproducible** — seed the PRNG for deterministic, repeatable runs
- **Live stats** — automatic fps/count/error display on interactive
  terminals, updated every second
- **Frame dump** — `--dump` prints each sent frame to stdout in candump
  format for inspection or piping
- **Resilient** — survives the interface vanishing and coming back
  (admin-down, removal, USB unplug); detects controller BUS-OFF via CAN
  error frames and pauses transmission instead of silently counting
  phantom sends; shows ERROR-WARNING / ERROR-PASSIVE, un-ACKed frames,
  error counters and the on-wire frame count in the stats line, and can
  hold while ERROR-PASSIVE (`--on-error pause`); with `--auto-restart`
  cycles the interface via netlink
  to recover from BUS-OFF on drivers that lack kernel auto-restart
  (notably `gs_usb` / candleLight / Canable v1)
- **Minimal dependencies** — just `clap`, `libc`, `nix`, and `fastrand`

## Requirements

- Linux with SocketCAN support (kernel 2.6.25+)
- Rust toolchain 1.94+
- `CAP_NET_RAW` capability or root access

## Building

```bash
make build
```

Or directly with Cargo:

```bash
cargo build --release
```

The binary is at `target/release/mcangen`.

## Installation

From crates.io:

```bash
cargo install mcangen
```

From source:

```bash
make install                # installs to ~/.local/bin and man page
```

Or to a system-wide location:

```bash
make install PREFIX=/usr/local
```

## Usage

```
mcangen [OPTIONS] <INTERFACE>
```

### Examples

**Blast a million frames as fast as possible and report progress:**

```bash
mcangen can0 -r 0 -n 1000000 -p 100000
```

**Send at exactly 500 fps with mixed standard and extended IDs:**

```bash
mcangen can0 -r 500 --id-kind mixed -n 5000
```

**Walk through every standard ID sequentially, fixed DLC of 4:**

```bash
mcangen can0 --id-mode sequential --id-min 0x000 --id-max 0x7FF \
    --dlc-min 4 --dlc-max 4 -n 2048
```

**Use sequence numbers to detect drops on the receiver:**

```bash
mcangen can0 -n 50000 -r 10000 --data-mode sequence
```

On the receiving end, decode the 8 payload bytes as a big-endian u64.
Any gap in the sequence means a dropped frame.

**Reproducible test run (same seed = same frames):**

```bash
mcangen can0 -n 5000 -r 1000 --seed 42
```

**Extended IDs only, random data, no rate limit:**

```bash
mcangen can0 --id-kind extended -r 0 -n 100000
```

**UDS flash simulation — realistic ECU reprogramming session on the bus:**

```bash
mcangen vcan0 --uds-flash -n 1
```

Generates a full 16-phase UDS reprogramming session: diagnostic session
control, ECU identification reads, security access (seed/key), memory
erase with pending responses, multi-frame ISO-TP firmware transfer
(50–150 blocks), DTC read/clear/verify, and ECU reset. Both tester and
ECU frames appear on the bus with realistic timing.

```bash
# Double speed, fixed 100 blocks, no error injection
mcangen can0 --uds-flash -n 1 --speed 2.0 --transfer-blocks 100 --no-errors

# Loop forever with OBD-II polling between sessions
mcangen vcan0 --uds-flash

# Custom arbitration IDs, skip inter-session OBD traffic
mcangen can0 --uds-flash --tester-id 0x641 --ecu-id 0x642 --no-obd -n 3
```

In UDS flash mode, `-n` sets the number of sessions (0 = loop forever).
Between sessions, OBD-II polling traffic (PIDs on 0x7DF) is generated
with drifting vehicle state unless `--no-obd` is given.

**Burst mode — simulate ECU reprogramming traffic pattern:**

```bash
mcangen can0 --burst -n 100000
```

Default cycle: 2 s at 5000 fps, then 500 ms at 50 fps, repeating.
Customise with `--burst-high-rate`, `--burst-low-rate`, `--burst-high-ms`,
`--burst-low-ms`.

**CANcorder quality-test protocol on a fixed ID:**

```bash
mcangen can0 --data-mode quality-test --id 0x7E0 -r 1000 -n 10000
```

Generates frames with the 0xCAFE magic marker, 16-bit sequence number,
16-bit timestamp offset, test ID, and XOR checksum — ready for
CANcorder's quality test panel.
In this mode, `--id` is required and `--id-min`/`--id-max` are rejected.

**Versioned CAN-FD quality traffic with BRS:**

```bash
mcangen can1 --fd --brs --data-mode quality-test --id 0x700 \
    --test-id 1 -r 1000 -n 30000
```

The FD quality format uses a fixed 64-byte payload with a 64-bit sequence
number, microsecond sender clock, deterministic payload pattern, and CRC32C.
It is understood by `mcandump --quality-test` and ESPenlaub hwtest's
`twai quality` commands. Regular CAN-FD generation accepts payload lengths up
to 64 bytes; lengths are rounded up to a valid CAN-FD wire length.

**Quiet mode for scripting (exit code only):**

```bash
mcangen can0 -n 10000 -q && echo "done"
```

### All options

| Option | Description | Default |
|---|---|---|
| `-n, --count N` | Number of frames to send (0 = unlimited) | `0` |
| `-r, --rate FPS` | Target frames/sec (0 = max speed) | `5` |
| `--dlc-min N` | Minimum payload length (0–8 classic, 0–64 FD) | `0` |
| `--dlc-max N` | Maximum payload length (0–8 classic, 0–64 FD) | `8` |
| `--fd` | Emit Linux `canfd_frame`s; quality-test uses the versioned 64-byte format | off |
| `--brs` | Enable CAN-FD bitrate switching (requires `--fd`) | off |
| `--id-min ID` | Minimum CAN ID (hex or decimal) | `0x000` |
| `--id-max ID` | Maximum CAN ID (hex or decimal) | `0x7FF` / `0x1FFFFFFF` |
| `--id ID` | Exact CAN ID; required for `quality-test` mode | none |
| `--id-kind MODE` | `standard`, `extended`, or `mixed` | `standard` |
| `--ext-id-above-sff` | Keep extended IDs > 0x7FF to avoid misdetection | `true` |
| `--id-mode MODE` | `random` or `sequential` | `random` |
| `--data-mode MODE` | `random`, `zero`, `counter`, `sequence`, `ones`, or `quality-test` | `random` |
| `-s, --seed SEED` | RNG seed (0 = random) | `0` |
| `-p, --progress N` | Print stats every N frames | `0` |
| `-q, --quiet` | Suppress all output except errors | off |
| `--dump` | Dump sent frames to stdout in candump format | off |
| `--no-local-loopback` | Disable local TX echo on the generator socket (useful with controller loopback) | off |
| `--burst` | Enable burst mode (alternating high/low rate) | off |
| `--burst-high-rate FPS` | High-rate phase FPS | `5000` |
| `--burst-low-rate FPS` | Low-rate phase FPS | `50` |
| `--burst-high-ms MS` | High-rate phase duration (ms) | `2000` |
| `--burst-low-ms MS` | Low-rate phase duration (ms) | `500` |
| `--test-id ID` | Test ID byte for `quality-test` mode (0–255) | `0` |
| `--uds-flash` | UDS flash simulation mode (see below) | off |
| `--tester-id ID` | [UDS flash] Tester request CAN ID | `0x7E0` |
| `--ecu-id ID` | [UDS flash] ECU response CAN ID | `0x7E8` |
| `--speed FACTOR` | [UDS flash] Timing multiplier (2.0 = double speed) | `1.0` |
| `--transfer-blocks N` | [UDS flash] Blocks per session (0 = random 50–150) | `0` |
| `--no-obd` | [UDS flash] Skip OBD-II polling between sessions | off |
| `--no-errors` | [UDS flash] Disable error injection | off |
| `--auto-restart` | On BUS-OFF, cycle the interface via netlink to recover (needs `CAP_NET_ADMIN`) | off |
| `--on-error` | While the controller is ERROR-PASSIVE: `continue` (send, show it in the stats line) or `pause` (hold until ERROR-ACTIVE) | `continue` |

### Makefile targets

Run `make` to see all available targets:

```
build      Build release binary
run        Run with IFACE, COUNT, RATE, EXTRA
blast      1M frames as fast as possible
test       Quick smoke test (1000 frames @ 2000 fps)
vcan       Create vcan0 virtual interface (requires sudo)
man        View the man page
install    Install binary and man page to PREFIX
uninstall  Remove installed files
fmt        cargo fmt
check      cargo check
clippy     cargo clippy
clean      cargo clean
```

Override the interface: `make blast IFACE=vcan0`

### Testing with virtual CAN

No hardware needed — use the kernel's virtual CAN driver:

```bash
make vcan                   # set up vcan0 (requires sudo)
make test IFACE=vcan0       # quick smoke test
make blast IFACE=vcan0      # 1M frame throughput benchmark
```

## Permissions

Sending raw CAN frames requires `CAP_NET_RAW`. Either run as root or
grant the capability to the binary:

```bash
sudo setcap cap_net_raw+ep target/release/mcangen
```

If you also want `--auto-restart` (which calls `RTM_NEWLINK` over
netlink to cycle the interface), grant both capabilities:

```bash
sudo setcap cap_net_admin,cap_net_raw=eip target/release/mcangen
```

Without `CAP_NET_ADMIN`, `--auto-restart` prints a one-shot warning and
falls back to wait-mode for the rest of the run.

## Resilience

mcangen is designed to keep running across transient interface failures
rather than die on the first hiccup.

**Interface vanishes** (`ip link delete`, USB unplug, `ip link set …
down`): writes start failing with `ENODEV`/`ENETDOWN`. mcangen closes
the dead socket and reopens with exponential backoff (100 ms → 30 s
cap), only declaring success once the interface is administratively up
again. The stats line keeps ticking through the outage so you can see
the gap.

**Controller goes BUS-OFF**: `write()` doesn't reliably surface this —
on many drivers frames just queue silently while no actual bus traffic
flows. mcangen opens a second socket with `CAN_RAW_ERR_FILTER` set to
`BUSOFF | RESTARTED`, and a background thread reads error frames from
it. On `CAN_ERR_BUSOFF` the live stats line shows `BUS-OFF`, the send
loop pauses (with backoff), and frames that *would* have been written
are counted as errors instead of as phantom successes. On
`CAN_ERR_RESTARTED` (kernel auto-restart, manual `ip link set <iface>
type can restart`, or our own `--auto-restart`), transmission
transparently resumes.

**Nobody ACKs — ERROR-PASSIVE without BUS-OFF**: a node whose frames
no other node acknowledges never reaches BUS-OFF: per spec its TX error
counter stops at 128, so it sits in ERROR-PASSIVE retrying the same
frame forever while `write()` keeps succeeding into the qdisc (default
`qlen` 10, often raised to thousands). The error socket therefore also
subscribes to `CRTL | ACK`: the stats line shows `ERROR-PASSIVE`,
`no-ack <n>`, `tec/rec` and `wire <n>` (sysfs `tx_packets`, i.e. frames
the driver really completed — shown whenever it differs from `frames`),
and a one-shot message on stderr points at wiring / termination /
bitrate. `--on-error pause` holds transmission until the controller
reports ERROR-ACTIVE again; the default `continue` keeps sending, which
is what you want when the far side is expected to come up later.

**Recovery from BUS-OFF on uncooperative drivers**: not every CAN
driver implements the kernel hook (`do_set_mode`) needed to honor
`restart-ms` or the `IFLA_CAN_RESTART` netlink attribute. The
`gs_usb` driver — used by candleLight, Canable v1, gs_usb_leonardo,
and most cheap STM32-based USB-CAN adapters — is the notable mainline
example. For these, the kernel cannot self-recover and `ip link set
<iface> type can restart` returns `EOPNOTSUPP`. With `--auto-restart`
mcangen works around this by cycling the interface itself via raw
netlink (down → 150 ms settle → up). Repeated rapid restarts back off
exponentially up to 60 s so a permanently bad bus can't pin the CPU.

```bash
# fire and forget — recover from BUS-OFF without operator intervention
mcangen can0 -r 100 --auto-restart
```

## A note on frame counts

If you send 1,000,000 frames and your receiver reports slightly more
(e.g. 1,001,034), that's not a bug. CAN controllers automatically
retransmit frames that lose bus arbitration or encounter errors. Each
retransmission is a valid frame on the wire, so the receiver counts it.
mcangen counts successful `write()` calls, which will always match
`--count` exactly. The difference tells you how noisy or contested your
bus is.

## See Also

- [mcandump](https://github.com/mickeyl/mcandump) — CAN bus logger
  proxy (companion tool for capturing and forwarding traffic)
- [CANcorder](https://apps.apple.com/app/cancorder/id6743640770) — CAN
  bus logger and analyzer for macOS/iOS

## Man page

```bash
man ./man/mcangen.1
```

## License

[MIT](LICENSE)

## Author

Dr. Michael 'Mickey' Lauer <mickey@vanille-media.de>
