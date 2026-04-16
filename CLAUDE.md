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

Single-file application: everything is in `src/main.rs` (~1900 lines). No modules, no library crate.

**Sections in order:**

1. **SocketCAN FFI** — `CanFrame`, `SockaddrCan`, `Ifreq` structs matching kernel layout. `open_can_socket()` does socket/ioctl/bind. `send_frame()` does raw `libc::write`.

2. **CLI** — clap derive macros. Three enums control behavior: `IdMode` (random/sequential), `DataMode` (random/zero/counter/sequence/ones/quality-test), `IdKind` (standard/extended/mixed). The `Cli` struct has two groups of options: standard frame generation and UDS flash mode (`--uds-flash` and its `--tester-id`, `--ecu-id`, `--speed`, `--transfer-blocks`, `--no-obd`, `--no-errors`). Standard mode supports either an ID range (`--id-min`/`--id-max`) or an exact `--id`; `quality-test` requires the exact-ID form.

3. **RNG** — Custom xorshift64* PRNG (`Rng` struct). Methods include `uniform()` for float ranges, `delay_us()` for randomized timing in microseconds, `chance()` for probability checks, `fill_bytes()` for bulk random data.

4. **Rate limiter** — `wait_until()` uses hybrid sleep (>2ms) + busy-spin (final stretch) for sub-millisecond precision.

5. **UDS Flash simulation** — `TimedFrame` struct holds a CAN ID, 8-byte payload, and pre-delay in microseconds (can_id=0 means delay-only marker). ISO-TP helpers (`push_sf`, `push_nrc`, `push_multi`) build single-frame, negative response, and multi-frame (FF+FC+CFs) sequences. `gen_uds_session()` orchestrates 16 phases of a realistic ECU reprogramming session. `gen_obd_polling()` produces inter-session OBD-II traffic. `play_timed_frames()` sends the pre-generated frame list with scaled timing. `run_uds_flash()` is the top-level loop.

6. **Live monitoring** — `LiveState` struct with atomic counters (`sent`, `errors`, `running`) shared between the main send loop and optional background threads. `stats_thread()` prints a `\r`-overwritten stats line to stderr every second. `dump_thread()` reads frames from a bounded `mpsc::sync_channel` and prints candump-style output to stdout; frames are dropped silently if the channel is full.

7. **main()** — Validates args, opens socket, spawns optional stats/dump threads, dispatches to either `run_uds_flash()` (early return) or the standard frame-generation loop with burst/constant/max-rate timing, then joins monitoring threads.

**Key design choices:**
- UDS flash pre-generates an entire session as `Vec<TimedFrame>` then plays it back, separating generation from timing.
- The standard mode generates and sends one frame at a time in a tight loop.
- All CAN IDs use the raw SocketCAN u32 format (bit 31 = extended frame flag).
- ISO-TP padding byte is 0xCC throughout.
- Monitoring threads use relaxed atomics and try_send to avoid impacting the hot send loop.

## Dependencies

Only four crates: `clap` (CLI), `fastrand` (seed fallback), `libc` (syscalls), `nix` (higher-level Unix bindings). The release profile enables LTO and single codegen unit.
