# Contributing to mcangen

Thanks for your interest in contributing!

## Building

You need a stable Rust toolchain (1.70+) and a Linux system with SocketCAN headers.

```bash
make build
```

## Testing

Set up a virtual CAN interface and run the smoke test:

```bash
make vcan
make test IFACE=vcan0
```

## Code quality

Before submitting a PR, please run:

```bash
make fmt
make clippy
```

## Submitting changes

1. Fork the repository
2. Create a feature branch (`git checkout -b my-feature`)
3. Make your changes and ensure `make clippy` passes
4. Commit with a clear message
5. Open a pull request

## Reporting bugs

Open an issue with:
- Your kernel version (`uname -r`)
- CAN interface type (hardware or vcan)
- The exact command you ran
- The output / error you got
