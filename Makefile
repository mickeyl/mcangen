IFACE     ?= can0
COUNT     ?= 0
RATE      ?= 0
EXTRA     ?=

PREFIX    ?= $(HOME)/.local
BINDIR    ?= $(PREFIX)/bin
MANDIR    ?= $(PREFIX)/share/man

BIN       := target/release/mcangen
SRC       := $(shell find src -name '*.rs') Cargo.toml Cargo.lock

CAPS      := cap_net_admin,cap_net_raw=eip

VERSION   := $(shell grep '^version' Cargo.toml | head -1 | sed 's/.*"\(.*\)"/\1/')

.PHONY: all build run blast uds-flash test clean install uninstall fmt check clippy vcan vcanfd man release publish help

all: help

build: $(BIN)

$(BIN): $(SRC)
	cargo build --release
	@echo "Applying capabilities ($(CAPS)) — may prompt for sudo password"
	sudo setcap $(CAPS) $(BIN)

run: $(BIN)
	./$(BIN) $(IFACE) -n $(COUNT) -r $(RATE) $(EXTRA)

# Send as fast as possible — good for throughput benchmarks
blast: $(BIN)
	./$(BIN) $(IFACE) -r 0 -n 1000000 -p 100000 $(EXTRA)

# UDS flash simulation: single session by default
uds-flash: $(BIN)
	./$(BIN) $(IFACE) --uds-flash -n 1 $(EXTRA)

# Quick smoke test: 1000 frames at 2000 fps with progress
test: $(BIN)
	./$(BIN) $(IFACE) -n 1000 -r 2000 -p 500 $(EXTRA)

# Set up a virtual CAN interface for testing
vcan:
	sudo modprobe vcan
	sudo ip link add dev vcan0 type vcan 2>/dev/null || true
	sudo ip link set up vcan0

# Set up a virtual CAN-FD interface for testing
vcanfd:
	sudo modprobe vcan
	sudo ip link add dev vcan0 type vcan mtu 72 2>/dev/null || true
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
	@echo "Applying capabilities ($(CAPS)) to $(BINDIR)/mcangen — may prompt for sudo password"
	sudo setcap $(CAPS) $(BINDIR)/mcangen

uninstall:
	rm -f $(BINDIR)/mcangen
	rm -f $(MANDIR)/man1/mcangen.1

release: build
	@if git diff --quiet && git diff --cached --quiet; then \
		echo "Working tree clean — tagging v$(VERSION)"; \
	else \
		echo "error: uncommitted changes — commit first"; exit 1; \
	fi
	@if git tag | grep -q "^v$(VERSION)$$"; then \
		echo "error: tag v$(VERSION) already exists — bump version in Cargo.toml and man/mcangen.1"; exit 1; \
	fi
	git tag -a v$(VERSION) -m "v$(VERSION)"
	git push --tags
	@echo "Tagged and pushed v$(VERSION) — GitHub release workflow will build binaries."
	@echo "Run 'make publish' to push to crates.io."

publish:
	cargo publish --dry-run
	@echo ""
	@echo "Dry run passed. Publishing v$(VERSION) to crates.io in 5s... (Ctrl-C to abort)"
	@sleep 5
	cargo publish

clean:
	cargo clean

help:
	@echo "mcangen Makefile"
	@echo ""
	@echo "Targets:"
	@echo "  build      Build release binary (default)"
	@echo "  run        Run with IFACE, COUNT, RATE, EXTRA"
	@echo "  blast      1M frames as fast as possible"
	@echo "  uds-flash  Run single UDS flash session"
	@echo "  test       Quick smoke test (1000 frames @ 2000 fps)"
	@echo "  vcan       Create vcan0 virtual interface (requires sudo)"
	@echo "  vcanfd     Create vcan0 with CAN-FD MTU (requires sudo)"
	@echo "  man        View the man page"
	@echo "  install    Install binary and man page to PREFIX (default: ~/.local)"
	@echo "  uninstall  Remove installed files"
	@echo "  fmt        cargo fmt"
	@echo "  check      cargo check"
	@echo "  clippy     cargo clippy"
	@echo "  clean      cargo clean"
	@echo "  release    Tag v\$$VERSION, push tag, trigger GitHub release build"
	@echo "  publish    Publish to crates.io (dry-run first, 5s to abort)"
	@echo ""
	@echo "Variables:"
	@echo "  IFACE=$(IFACE)  COUNT=$(COUNT)  RATE=$(RATE)  PREFIX=$(PREFIX)  VERSION=$(VERSION)"
	@echo ""
	@echo "Examples:"
	@echo "  make run RATE=500 COUNT=5000"
	@echo "  make blast IFACE=vcan0"
	@echo "  make install PREFIX=/usr/local"
	@echo "  make run EXTRA='--id-kind mixed --data-mode counter'"
