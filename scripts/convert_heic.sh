#!/bin/bash
# Usage: ./scripts/convert_heic.sh <input.HEIC> <output.jpg> [max_dimension]
# Example: ./scripts/convert_heic.sh data/IMG_3540.HEIC data/left.jpg 1920

INPUT="${1}"
OUTPUT="${2}"
MAX_DIM="${3:-1920}"

if [[ -z "$INPUT" || -z "$OUTPUT" ]]; then
    echo "Usage: $0 <input.HEIC> <output.jpg> [max_dimension]"
    exit 1
fi

if [[ ! -f "$INPUT" ]]; then
    echo "Error: file not found: $INPUT"
    exit 1
fi

sips -s format jpeg -Z "$MAX_DIM" "$INPUT" --out "$OUTPUT"
echo "Converted: $INPUT -> $OUTPUT (max ${MAX_DIM}px)"
rm "$INPUT"
echo "Deleted: $INPUT"
