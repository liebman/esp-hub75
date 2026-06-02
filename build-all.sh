#!/bin/bash
set -e

aliases=(
    build-32
    build-32-l
    build-32h
    build-32r
    build-c5-bl
    build-c6-bp
    build-c6-bl
    build-c6r
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
