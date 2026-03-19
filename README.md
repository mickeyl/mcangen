# mcangen

High-performance CAN bus frame generator for Linux. A fast, flexible
replacement for `cangen` from can-utils, built in Rust for maximum throughput.

## Features

- **Fast** — raw SocketCAN writes with zero-copy framing, optimized release build with LTO
- **All frame types** — standard (11-bit) IDs, extended (29-bit) IDs, or a random mix
- **Configurable DLC** — any range from 0 to 8 bytes
- **Data patterns** — random, zeros, ones (0xFF), incrementing counter, or 64-bit sequence number
- **Precise rate control** — hybrid sleep/busy-spin for accurate FPS targeting
- **Exact counts** — send a precise number of frames then stop
- **Reproducible** — seed the RNG for deterministic output
- **Minimal dependencies** — just `clap`, `libc`, `nix`, and `fastrand`

## Requirements

- Linux with SocketCAN support (kernel 2.6.25+)
- Rust stable toolchain (1.70+)
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

```bash
make install        # installs to ~/.local/bin/mcangen
```

Or copy the binary wherever you like:

```bash
sudo install -Dm755 target/release/mcangen /usr/local/bin/mcangen
```

## Usage

```
mcangen [OPTIONS] <INTERFACE>
```

### Quick examples

```bash
# Send 10000 random frames as fast as possible
mcangen can0 -n 10000

# 500 fps, mixed standard + extended IDs
mcangen vcan0 -r 500 --id-kind mixed -n 5000

# Sequential IDs, fixed DLC of 4, counter payload
mcangen can0 --id-mode sequential --dlc-min 4 --dlc-max 4 --data-mode counter -p 1000

# Reproducible run
mcangen vcan0 -n 5000 -r 1000 --seed 42

# Full-speed blast with progress reporting
mcangen can0 -n 1000000 -p 100000
```

### Options

| Option | Description | Default |
|---|---|---|
| `-n, --count N` | Number of frames to send (0 = unlimited) | `0` |
| `-r, --rate FPS` | Target frames/sec (0 = max speed) | `0` |
| `--dlc-min N` | Minimum DLC (0–8) | `0` |
| `--dlc-max N` | Maximum DLC (0–8) | `8` |
| `--id-min ID` | Minimum CAN ID (hex or decimal) | `0x000` |
| `--id-max ID` | Maximum CAN ID (hex or decimal) | `0x7FF` |
| `--id-kind MODE` | `standard`, `extended`, or `mixed` | `standard` |
| `--id-mode MODE` | `random` or `sequential` | `random` |
| `--data-mode MODE` | `random`, `zero`, `counter`, `sequence`, or `ones` | `random` |
| `-s, --seed SEED` | RNG seed (0 = random) | `0` |
| `-p, --progress N` | Print stats every N frames | `0` |
| `-q, --quiet` | Suppress all output except errors | off |

### Testing with virtual CAN

```bash
make vcan                   # set up vcan0 (requires sudo)
make test IFACE=vcan0       # quick smoke test
make blast IFACE=vcan0      # 1M frame throughput benchmark
```

## Permissions

Sending raw CAN frames requires `CAP_NET_RAW`. Either run as root or:

```bash
sudo setcap cap_net_raw+ep target/release/mcangen
```

## Man page

```bash
man ./man/mcangen.1
```

To install system-wide:

```bash
sudo install -Dm644 man/mcangen.1 /usr/local/share/man/man1/mcangen.1
```

## License

[MIT](LICENSE)
