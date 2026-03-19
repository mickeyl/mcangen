IFACE     ?= can0
COUNT     ?= 0
RATE      ?= 0
EXTRA     ?=

PREFIX    ?= $(HOME)/.local
BINDIR    ?= $(PREFIX)/bin
MANDIR    ?= $(PREFIX)/share/man

BIN       := target/release/mcangen
SRC       := $(shell find src -name '*.rs') Cargo.toml Cargo.lock

.PHONY: all build run blast test clean install uninstall fmt check clippy vcan man help

all: help

build: $(BIN)

$(BIN): $(SRC)
	cargo build --release

run: $(BIN)
	./$(BIN) $(IFACE) -n $(COUNT) -r $(RATE) $(EXTRA)

# Send as fast as possible — good for throughput benchmarks
blast: $(BIN)
	./$(BIN) $(IFACE) -n 1000000 -p 100000 $(EXTRA)

# Quick smoke test: 1000 frames at 2000 fps with progress
test: $(BIN)
	./$(BIN) $(IFACE) -n 1000 -r 2000 -p 500 $(EXTRA)

# Set up a virtual CAN interface for testing
vcan:
	sudo modprobe vcan
	sudo ip link add dev vcan0 type vcan 2>/dev/null || true
	sudo ip link set up vcan0

fmt:
	cargo fmt

check:
	cargo check

clippy:
	cargo clippy -- -D warnings

man:
	@man ./man/mcangen.1

install: $(BIN)
	install -Dm755 $(BIN) $(BINDIR)/mcangen
	install -Dm644 man/mcangen.1 $(MANDIR)/man1/mcangen.1

uninstall:
	rm -f $(BINDIR)/mcangen
	rm -f $(MANDIR)/man1/mcangen.1

clean:
	cargo clean

help:
	@echo "mcangen Makefile"
	@echo ""
	@echo "Targets:"
	@echo "  build      Build release binary (default)"
	@echo "  run        Run with IFACE, COUNT, RATE, EXTRA"
	@echo "  blast      1M frames as fast as possible"
	@echo "  test       Quick smoke test (1000 frames @ 2000 fps)"
	@echo "  vcan       Create vcan0 virtual interface (requires sudo)"
	@echo "  man        View the man page"
	@echo "  install    Install binary and man page to PREFIX (default: ~/.local)"
	@echo "  uninstall  Remove installed files"
	@echo "  fmt        cargo fmt"
	@echo "  check      cargo check"
	@echo "  clippy     cargo clippy"
	@echo "  clean      cargo clean"
	@echo ""
	@echo "Variables:"
	@echo "  IFACE=$(IFACE)  COUNT=$(COUNT)  RATE=$(RATE)  PREFIX=$(PREFIX)"
	@echo ""
	@echo "Examples:"
	@echo "  make run RATE=500 COUNT=5000"
	@echo "  make blast IFACE=vcan0"
	@echo "  make install PREFIX=/usr/local"
	@echo "  make run EXTRA='--id-kind mixed --data-mode counter'"
