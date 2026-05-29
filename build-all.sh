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
    build-c5-l
    build-c5-bl
    build-c5-li
    build-c5-bli
    build-c6
    build-c6-bp
    build-c6-l
    build-c6-bl
    build-c6h
    build-c6r
    build-c6ri
    build-c6-bpi
    build-c6-li
    build-c6-bli
    build-s3
    build-s3-bp
    build-s3-l
    build-s3-bl
    build-s3-t
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
