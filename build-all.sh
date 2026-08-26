#!/bin/bash
set -e

# Build the esp-hub75 library.
lib_aliases=(
    build-esp32
    build-esp32-c
    build-esp32c6
    build-esp32c5
    build-esp32s3
    build-esp32s3-c
)

for alias in "${lib_aliases[@]}"; do
    echo "=========================================="
    echo "Building (library): cargo $alias"
    echo "=========================================="
    cargo "$alias"
done

# Build each example crate using its own `build-*` aliases. Every example
# defines a common set of board aliases; a few add extras (e.g. `build-esp32c5`
# for the latched example), so only run the aliases a crate actually defines.
example_aliases=(
    build-esp32
    build-esp32s3
    build-esp32c6
    build-trinity
    build-esp32c5
)

for dir in examples/*/; do
    for alias in "${example_aliases[@]}"; do
        if grep -qE "^[[:space:]]*${alias}[[:space:]]*=" "$dir/.cargo/config.toml" 2>/dev/null; then
            echo "=========================================="
            echo "Building: ${dir}cargo $alias"
            echo "=========================================="
            (cd "$dir" && cargo "$alias")
        fi
    done
done

echo "=========================================="
echo "All builds succeeded!"
echo "=========================================="
