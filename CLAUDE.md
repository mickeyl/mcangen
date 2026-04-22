# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

mcangen is a high-performance CAN bus frame generator for Linux, written in Rust. It sends CAN frames to SocketCAN interfaces for testing, benchmarking, and simulating automotive diagnostic traffic. It replaces `cangen` from can-utils with better timing precision, exact frame counts, and additional modes including full UDS flash simulation.

## Build and test

```bash
cargo build --release          # release build (LTO, stripped)
cargo build                    # debug build
make build                     # same as cargo build --release
cargo clippy                   # lint
cargo fmt                      # format
```

There is no test suite — testing requires a SocketCAN interface. Set up virtual CAN for local testing:

```bash
sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
make test IFACE=vcan0          # 1000 frames @ 2000 fps smoke test
make blast IFACE=vcan0         # 1M frames at max rate
```

The binary requires `CAP_NET_RAW` or root to open raw CAN sockets.

## Architecture

Single-file application: everything is in `src/main.rs` (~2500 lines). No modules, no library crate.

**Sections in order:**

1. **SocketCAN FFI** — `CanFrame`, `SockaddrCan`, `Ifreq` structs matching kernel layout. Constants for both classic and error frames (`CAN_RAW_FILTER`, `CAN_RAW_ERR_FILTER`, `CAN_ERR_FLAG`, `CAN_ERR_BUSOFF`, `CAN_ERR_RESTARTED`). `open_can_socket()` does socket/ioctl/bind for the send socket; `open_err_socket()` opens a separate socket with empty RX filter + error filter + 1s `SO_RCVTIMEO` for the bus-state monitor. `send_frame()` does raw `libc::write`; `send_frames_batch()` uses `sendmmsg`.

2. **CLI** — clap derive macros. Three enums control behavior: `IdMode` (random/sequential), `DataMode` (random/zero/counter/sequence/ones/quality-test), `IdKind` (standard/extended/mixed). The `Cli` struct has two groups of options: standard frame generation and UDS flash mode (`--uds-flash` and its `--tester-id`, `--ecu-id`, `--speed`, `--transfer-blocks`, `--no-obd`, `--no-errors`). Standard mode supports either an ID range (`--id-min`/`--id-max`) or an exact `--id`; `quality-test` requires the exact-ID form. `--auto-restart` enables the netlink BUS-OFF recovery path.

3. **Interface vanishing / reconnect** — `is_iface_down_err()` classifies `ENETDOWN`/`ENODEV`/`ENXIO`/`ENETUNREACH` as iface-vanished. `iface_is_up()` queries `IFF_UP` via `nix::ifaddrs::getifaddrs`. `reconnect_socket()` closes the dead fd and reopens with 100ms→30s exponential backoff, refusing to declare success until `IFF_UP` returns true (avoids tight loops on admin-down interfaces).

4. **Netlink for BUS-OFF recovery** — `if_nametoindex()`, `nl_set_iface_up()`, `cycle_iface()`. Hand-rolled `RTM_NEWLINK` over a `NETLINK_ROUTE` socket: send a single `nlmsghdr` + `ifinfomsg` (no nested attributes, just toggling `IFF_UP`), parse the `NLMSG_ERROR` ack and surface negative errno (so `EPERM` propagates cleanly). Used by `--auto-restart` to work around drivers without `do_set_mode` (notably `gs_usb`). Bittiming is *not* reapplied — `priv->bittiming` persists across `ndo_close`/`ndo_open` in the can-dev framework.

5. **RNG** — Custom xorshift64* PRNG (`Rng` struct). Methods include `uniform()` for float ranges, `delay_us()` for randomized timing in microseconds, `chance()` for probability checks, `fill_bytes()` for bulk random data.

6. **Rate limiter** — `wait_until()` uses hybrid sleep (>2ms) + busy-spin (final stretch) for sub-millisecond precision.

7. **UDS Flash simulation** — `TimedFrame` struct holds a CAN ID, 8-byte payload, and pre-delay in microseconds (can_id=0 means delay-only marker). ISO-TP helpers (`push_sf`, `push_nrc`, `push_multi`) build single-frame, negative response, and multi-frame (FF+FC+CFs) sequences. `gen_uds_session()` orchestrates 16 phases of a realistic ECU reprogramming session. `gen_obd_polling()` produces inter-session OBD-II traffic. `play_timed_frames()` sends the pre-generated frame list with scaled timing, takes `&mut fd` so it can reattach mid-playback. `run_uds_flash()` is the top-level loop.

8. **Live monitoring** — `LiveState` struct with atomic counters (`sent`, `errors`, `bus_off`, `running`) shared between the main send loop and background threads. `stats_thread()` prints a `\r`-overwritten stats line to stderr every second (showing `BUS-OFF` when `live.bus_off` is set); spawned automatically when stderr is a TTY (unless `--quiet` or `--dump` is set). `dump_thread()` reads frames from a bounded `mpsc::sync_channel` and prints candump-style output to stdout; frames are dropped silently if the channel is full. `err_monitor_thread()` reads CAN error frames from a dedicated socket, flips `bus_off` on `CAN_ERR_BUSOFF`/`CAN_ERR_RESTARTED`, self-heals across iface outages via `err_socket_reopen()`, and (when `auto_restart`) calls `cycle_iface()` with adaptive backoff (200ms doubling up to 60s on rapid retries). `wait_while_bus_off()` is the helper send loops use to pause during outages.

9. **main()** — Validates args, opens socket, spawns the err-monitor thread (always) and optional stats/dump threads, dispatches to either `run_uds_flash()` (early return) or the standard frame-generation loop with burst/constant/max-rate timing, then joins monitoring threads. All three send paths (`play_timed_frames`, max-rate batch, rate-limited single-frame) check `bus_off` first and pause via `wait_while_bus_off()`; on send errors classified by `is_iface_down_err()` they call `reconnect_socket()` and reset rate-limit baselines.

**Key design choices:**
- UDS flash pre-generates an entire session as `Vec<TimedFrame>` then plays it back, separating generation from timing.
- The standard mode generates and sends one frame at a time in a tight loop.
- All CAN IDs use the raw SocketCAN u32 format (bit 31 = extended frame flag).
- ISO-TP padding byte is 0xCC throughout.
- Monitoring threads use relaxed atomics and try_send to avoid impacting the hot send loop.
- BUS-OFF is *only* detectable via CAN error frames on a dedicated monitor socket — `write()` doesn't reliably surface it (frames queue silently in the kernel). Hence the second socket with empty RX filter + `CAN_RAW_ERR_FILTER`.
- The send fd is owned by the main thread and threaded through as `&mut i32` so reconnect can replace it in place; the monitor fd is owned exclusively by the monitor thread, which has its own reopen logic.
- Netlink is hand-rolled (~140 lines) rather than pulled from `netlink-packet-route` (~5 MB compiled) because we only need one message type with no nested attributes.

## Dependencies

Only four crates: `clap` (CLI), `fastrand` (seed fallback), `libc` (syscalls), `nix` (higher-level Unix bindings — used for `getifaddrs`/`InterfaceFlags` only). The release profile enables LTO and single codegen unit.
