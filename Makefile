# Makefile for RadarPub
# Part of the EdgeFirst Perception Middleware
# Copyright (c) 2025 Au-Zone Technologies

.PHONY: all build test clean format lint clippy audit doc coverage pre-release help

# Default target
all: build

# Build release binary
build:
	@echo "Building RadarPub..."
	cargo build --release --features "can,zenoh"

# Build all binaries
build-all:
	@echo "Building all binaries..."
	cargo build --release --features "can,zenoh"
	cargo build --release --bin drvegrdctl --features "can"

# Build examples
build-examples:
	@echo "Building examples..."
	cargo build --release --example radar_viewer --features "rerun"
	cargo build --release --example zenoh_viewer --features "rerun,zenoh"

# Cross-compile for ARM64 (Maivin/Raivin)
build-arm64:
	@echo "Cross-compiling for ARM64..."
	cross build --target aarch64-unknown-linux-gnu --release --features "can,zenoh"

# Run all tests
test:
	@echo "Running tests..."
	cargo test --workspace --all-features

# Run tests with coverage
coverage:
	@echo "Running tests with coverage..."
	@command -v cargo-llvm-cov >/dev/null 2>&1 || { \
		echo "cargo-llvm-cov not found. Installing..."; \
		cargo install cargo-llvm-cov; \
	}
	cargo llvm-cov --all-features --workspace --html
	@echo "Coverage report generated in target/llvm-cov/html/index.html"

# Format code
format:
	@echo "Formatting code..."
	cargo fmt --all

# Check formatting
format-check:
	@echo "Checking code formatting..."
	cargo fmt --all -- --check

# Run clippy linter
clippy:
	@echo "Running clippy..."
	cargo clippy --all-targets --all-features -- -D warnings

# Run all linters
lint: format-check clippy
	@echo "✅ All linting checks passed"

# Security audit
audit:
	@echo "Running security audit..."
	@command -v cargo-audit >/dev/null 2>&1 || { \
		echo "cargo-audit not found. Installing..."; \
		cargo install cargo-audit; \
	}
	cargo audit

# Generate documentation
doc:
	@echo "Generating documentation..."
	cargo doc --no-deps --open --all-features

# Generate SBOM (Software Bill of Materials)
sbom:
	@echo "Generating SBOM..."
	@if [ ! -f "venv/bin/scancode" ]; then \
		echo "Installing scancode-toolkit..."; \
		python3 -m venv venv; \
		venv/bin/pip install --upgrade pip; \
		venv/bin/pip install scancode-toolkit; \
	fi
	@if ! command -v cargo-cyclonedx >/dev/null 2>&1; then \
		echo "Installing cargo-cyclonedx..."; \
		cargo install cargo-cyclonedx; \
	fi
	@if ! command -v cyclonedx >/dev/null 2>&1 && ! [ -f ~/.local/bin/cyclonedx ]; then \
		echo "Installing CycloneDX CLI..."; \
		mkdir -p ~/.local/bin; \
		wget -q -O ~/.local/bin/cyclonedx \
			https://github.com/CycloneDX/cyclonedx-cli/releases/download/v0.27.1/cyclonedx-linux-x64; \
		chmod +x ~/.local/bin/cyclonedx; \
	fi
	@.github/scripts/generate_sbom.sh
	@echo "✅ SBOM generated: sbom.json"

# Verify version synchronization (Rust single-language project)
verify-version:
	@echo "Verifying version consistency..."
	@VERSION_CARGO=$$(grep '^version = ' Cargo.toml | cut -d'"' -f2); \
	VERSION_TAG=$$(git describe --tags --abbrev=0 2>/dev/null | sed 's/^v//'); \
	if [ "$$VERSION_CARGO" != "$$VERSION_TAG" ]; then \
	  echo "⚠️  Version mismatch: Cargo.toml=$$VERSION_CARGO, git tag=v$$VERSION_TAG"; \
	  echo "Update Cargo.toml or create new tag with: git tag -a v$$VERSION_CARGO -m 'Release v$$VERSION_CARGO'"; \
	else \
	  echo "✅ Version synchronized: $$VERSION_CARGO"; \
	fi

# Pre-release checklist (run before creating git tag)
pre-release: format lint test audit sbom
	@echo ""
	@echo "✅ All pre-release checks passed!"
	@echo ""
	@echo "Next steps:"
	@echo "  1. Update CHANGELOG.md with release notes"
	@echo "  2. Verify version: make verify-version"
	@echo "  3. git add . && git commit -m 'RADARPUB-XXX: Prepare version X.Y.Z'"
	@echo "  4. git push origin <branch>"
	@echo "  5. Wait for CI/CD to pass (all green checkmarks)"
	@echo "  6. git tag -a vX.Y.Z -m 'Release vX.Y.Z'"
	@echo "  7. git push origin vX.Y.Z"

# Clean build artifacts
clean:
	@echo "Cleaning build artifacts..."
	cargo clean

# Development setup
dev-setup:
	@echo "Setting up development environment..."
	@echo "Installing required tools..."
	rustup component add rustfmt clippy
	cargo install cargo-audit cargo-llvm-cov cross
	@echo "Development tools installed!"
	@echo ""
	@echo "For CAN testing without hardware, set up vcan:"
	@echo "  sudo modprobe vcan"
	@echo "  sudo ip link add dev vcan0 type vcan"
	@echo "  sudo ip link set up vcan0"

# Help
help:
	@echo "RadarPub - EdgeFirst Perception Radar Node"
	@echo ""
	@echo "Available targets:"
	@echo "  make build         - Build release binary"
	@echo "  make build-all     - Build all binaries (radarpub, drvegrdctl)"
	@echo "  make build-examples- Build example applications (radar_viewer, zenoh_viewer)"
	@echo "  make build-arm64   - Cross-compile for ARM64 (Maivin/Raivin)"
	@echo "  make test          - Run all tests"
	@echo "  make coverage      - Run tests with coverage report"
	@echo "  make format        - Format code with rustfmt"
	@echo "  make format-check  - Check code formatting"
	@echo "  make clippy        - Run clippy linter"
	@echo "  make lint          - Run all linters (format-check + clippy)"
	@echo "  make audit         - Run security audit"
	@echo "  make doc           - Generate and open documentation"
	@echo "  make sbom          - Generate Software Bill of Materials (CI/CD only)"
	@echo "  make verify-version- Check Cargo.toml vs git tag version"
	@echo "  make pre-release   - Run all pre-release checks"
	@echo "  make clean         - Clean build artifacts"
	@echo "  make dev-setup     - Install development tools"
	@echo "  make help          - Show this help message"
	@echo ""
	@echo "See CONTRIBUTING.md for detailed development guide"
