#!/bin/bash
set -e

aliases=(
    build-32
    build-32-bp
    build-32-l
    build-32-bl
    build-32-d
    build-32h
    build-32r
    # build-c5-li
    build-c5-bli
    build-c6r
    build-c6-bp
    build-c6-bl
    build-s3-bp
    build-s3-bl
    build-s3h
    build-s3r
)

for alias in "${aliases[@]}"; do
    echo "=========================================="
    echo "Building: cargo $alias"
    echo "=========================================="
    cargo "$alias"
done

echo "=========================================="
echo "All builds succeeded!"
echo "=========================================="
