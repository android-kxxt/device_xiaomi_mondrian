#!/bin/bash

set -e

# Patches the Android source tree recursively with the patches folder.

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
TOP="$(realpath "$SCRIPT_DIR/../../..")"

# Apply patches
echo "AOSP source tree: $TOP"
echo "Applying patches..."

find "$SCRIPT_DIR/patches" -type f -name "*.patch" | while read patch; do
    relative="$(realpath --relative-to="$SCRIPT_DIR/patches" "$patch")"
    dir="$(dirname "$relative")"
    echo "Applying $relative to $dir"
    patch -Np1 -d "$TOP/$dir" < "$patch"
done