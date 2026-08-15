#!/bin/bash
set -e

# Format the esp-hub75 library.
echo "=========================================="
echo "Formatting (library): cargo fmt"
echo "=========================================="
cargo fmt

# Format each example crate. Examples are independent crates (not part of a
# Cargo workspace), so each must be formatted separately. rustfmt discovers the
# repo-root `rustfmt.toml` by walking up the directory tree, so every example
# inherits the shared style.
for dir in examples/*/; do
    echo "=========================================="
    echo "Formatting: ${dir}cargo fmt"
    echo "=========================================="
    (cd "$dir" && cargo fmt)
done

echo "=========================================="
echo "All formatting succeeded!"
echo "=========================================="
