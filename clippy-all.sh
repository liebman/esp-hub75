#!/bin/bash
set -e

# Clippy the esp-hub75 library.
lib_aliases=(
    clippy-esp32
    clippy-esp32-c
    clippy-esp32c6
    clippy-esp32c5
    clippy-esp32s3
    clippy-esp32s3-c
)

for alias in "${lib_aliases[@]}"; do
    echo "=========================================="
    echo "Clippy (library): cargo $alias"
    echo "=========================================="
    cargo "$alias"
done

# Clippy each example crate using its own `clippy-*` aliases. Every example
# defines a common set of board aliases; a few add extras (e.g. `clippy-esp32c5`
# for the latched example), so only run the aliases a crate actually defines.
example_aliases=(
    clippy-esp32
    clippy-esp32s3
    clippy-esp32c6
    clippy-trinity
    clippy-esp32c5
)

for dir in examples/*/; do
    for alias in "${example_aliases[@]}"; do
        if grep -qE "^[[:space:]]*${alias}[[:space:]]*=" "$dir/.cargo/config.toml" 2>/dev/null; then
            echo "=========================================="
            echo "Clippy: ${dir}cargo $alias"
            echo "=========================================="
            (cd "$dir" && cargo "$alias")
        fi
    done
done

echo "=========================================="
echo "All clippy runs succeeded!"
echo "=========================================="
