# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

mcangen is a high-performance CAN and CAN-FD bus frame generator for Linux, written in Rust. It sends frames to SocketCAN interfaces for testing, benchmarking, simulating automotive diagnostic traffic, and running content-aware end-to-end quality tests. It replaces `cangen` from can-utils with better timing precision, exact frame counts, CAN-FD/BRS support, and additional modes including full UDS flash simulation.

## Build and test

```bash
cargo build --release          # release build (LTO, stripped)
cargo build                    # debug build
make build                     # same as cargo build --release
cargo test                     # unit tests plus optional can0 socket tests
cargo clippy                   # lint
cargo fmt                      # format
```

The test suite covers CLI validation, Classic/CAN-FD quality payloads, batching,
and rate-limiter precision. Socket send tests use `can0` when it exists; if it
exists it must be up and able to transmit. A virtual interface can provide a
hardware-independent setup:

```bash
sudo ip link add dev can0 type vcan && sudo ip link set up can0
cargo test
```

The binary requires `CAP_NET_RAW` or root to open raw CAN sockets.

## Architecture

Single-file application: everything is in `src/main.rs` (~2500 lines). No modules, no library crate.

**Sections in order:**

1. **SocketCAN FFI** — `CanFrame`, `CanFdFrame`, `SockaddrCan`, and `Ifreq` structs matching kernel layout. Constants cover Classic CAN, CAN-FD, local loopback, and error frames. `open_can_socket()` does socket/ioctl/bind, enables CAN-FD, and applies the requested local-loopback policy; `open_err_socket()` opens a separate socket with empty RX filter + error filter + 1s `SO_RCVTIMEO` for the bus-state monitor. Classic and FD paths have individual and `sendmmsg()` batch send helpers.

2. **CLI** — clap derive macros. Three enums control behavior: `IdMode` (random/sequential), `DataMode` (random/zero/counter/sequence/ones/quality-test), `IdKind` (standard/extended/mixed). The `Cli` struct has standard/FD frame generation and UDS flash options. `--fd` selects CAN-FD, `--brs` enables bitrate switching, and `--no-local-loopback` suppresses the generator socket's local TX echo for controller-loopback diagnostics. Standard mode supports either an ID range (`--id-min`/`--id-max`) or an exact `--id`; `quality-test` requires the exact-ID form. `--auto-restart` enables the netlink BUS-OFF recovery path.

3. **Quality protocol** — Classic quality frames use the compact legacy `CA FE` layout with a 16-bit sequence and XOR checksum. CAN-FD quality frames use the versioned `CA FD` layout with a 64-bit sequence, microsecond sender clock, deterministic payload pattern, and CRC32C. The format is shared with `mcandump --quality-test` and ESPenlaub hwtest's `twai quality` commands.

4. **Interface vanishing / reconnect** — `is_iface_down_err()` classifies `ENETDOWN`/`ENODEV`/`ENXIO`/`ENETUNREACH` as iface-vanished. `iface_is_up()` queries `IFF_UP` via `nix::ifaddrs::getifaddrs`. `reconnect_socket()` closes the dead fd and reopens with 100ms→30s exponential backoff, refusing to declare success until `IFF_UP` returns true (avoids tight loops on admin-down interfaces) and preserving the local-loopback setting.

5. **Netlink for BUS-OFF recovery** — `if_nametoindex()`, `nl_set_iface_up()`, `cycle_iface()`. Hand-rolled `RTM_NEWLINK` over a `NETLINK_ROUTE` socket: send a single `nlmsghdr` + `ifinfomsg` (no nested attributes, just toggling `IFF_UP`), parse the `NLMSG_ERROR` ack and surface negative errno (so `EPERM` propagates cleanly). Used by `--auto-restart` to work around drivers without `do_set_mode` (notably `gs_usb`). Bittiming is *not* reapplied — `priv->bittiming` persists across `ndo_close`/`ndo_open` in the can-dev framework.

6. **RNG** — Custom xorshift64* PRNG (`Rng` struct). Methods include `uniform()` for float ranges, `delay_us()` for randomized timing in microseconds, `chance()` for probability checks, `fill_bytes()` for bulk random data.

7. **Rate limiter** — `wait_until()` uses hybrid sleep (>2ms) + busy-spin (final stretch) for sub-millisecond precision.

8. **UDS Flash simulation** — `TimedFrame` struct holds a CAN ID, 8-byte payload, and pre-delay in microseconds (can_id=0 means delay-only marker). ISO-TP helpers (`push_sf`, `push_nrc`, `push_multi`) build single-frame, negative response, and multi-frame (FF+FC+CFs) sequences. `gen_uds_session()` orchestrates 16 phases of a realistic ECU reprogramming session. `gen_obd_polling()` produces inter-session OBD-II traffic. `play_timed_frames()` sends the pre-generated frame list with scaled timing, takes `&mut fd` so it can reattach mid-playback. `run_uds_flash()` is the top-level loop.

9. **Live monitoring** — `LiveState` struct with atomic counters (`sent`, `errors`, `bus_off`, `running`) shared between the main send loop and background threads. `stats_thread()` prints a `\r`-overwritten stats line to stderr every second (showing `BUS-OFF` when `live.bus_off` is set); spawned automatically when stderr is a TTY (unless `--quiet` or `--dump` is set). `dump_thread()` reads frames from a bounded `mpsc::sync_channel` and prints candump-style output to stdout; frames are dropped silently if the channel is full. `err_monitor_thread()` reads CAN error frames from a dedicated socket, flips `bus_off` on `CAN_ERR_BUSOFF`/`CAN_ERR_RESTARTED`, self-heals across iface outages via `err_socket_reopen()`, and (when `auto_restart`) calls `cycle_iface()` with adaptive backoff (200ms doubling up to 60s on rapid retries). `wait_while_bus_off()` is the helper send loops use to pause during outages.

10. **main()** — Validates args, opens the socket, spawns the error-monitor thread and optional stats/dump threads, then dispatches to UDS, Classic CAN, or CAN-FD generation. FD quality mode always uses its canonical integrity payload. All send paths check `bus_off` first and pause via `wait_while_bus_off()`; on interface-loss errors they reconnect and reset rate-limit baselines.

**Key design choices:**
- UDS flash pre-generates an entire session as `Vec<TimedFrame>` then plays it back, separating generation from timing.
- The standard mode generates and sends one frame at a time in a tight loop.
- All CAN IDs use the raw SocketCAN u32 format (bit 31 = extended frame flag).
- ISO-TP padding byte is 0xCC throughout.
- Monitoring threads use relaxed atomics and try_send to avoid impacting the hot send loop.
- BUS-OFF is *only* detectable via CAN error frames on a dedicated monitor socket — `write()` doesn't reliably surface it (frames queue silently in the kernel). Hence the second socket with empty RX filter + `CAN_RAW_ERR_FILTER`.
- The send fd is owned by the main thread and threaded through as `&mut i32` so reconnect can replace it in place; the monitor fd is owned exclusively by the monitor thread, which has its own reopen logic.
- Netlink is hand-rolled (~140 lines) rather than pulled from `netlink-packet-route` (~5 MB compiled) because we only need one message type with no nested attributes.

## CAN diagnosis user story

The motivating workflow is an end-to-end diagnostic test rather than a raw
frame-count benchmark: Linux generates reproducible Classic CAN or CAN-FD
quality traffic, an MCU validates and optionally relays it on a second CAN ID,
and `mcandump` verifies every sequence and payload while correlating relay
responses. A passing run proves that the complete path preserved ordering and
content; failures distinguish missing, duplicate, reordered, corrupt, and
kernel-dropped frames. Keep the quality wire format compatible across all
three projects (`mcangen`, `mcandump`, and ESPenlaub hwtest).

## Dependencies

Only four crates: `clap` (CLI), `fastrand` (seed fallback), `libc` (syscalls), `nix` (higher-level Unix bindings — used for `getifaddrs`/`InterfaceFlags` only). The release profile enables LTO and single codegen unit.
