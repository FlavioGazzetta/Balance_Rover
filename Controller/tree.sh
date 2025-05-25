#!/bin/bash
# This script generates a tree-like directory listing for your project,
# including both directories and files, while excluding any directories
# under ".git/objects" and "node_modules", using find and sed.

# Set the base directory to the current directory
BASE_DIR="."

# Set the output file where the tree will be saved
OUTPUT_FILE="directory_tree.txt"

# Remove any existing output file
rm -f "$OUTPUT_FILE"

# Generate the tree structure, excluding .git/objects and node_modules
find "$BASE_DIR" \( -path "*/.venv/*" \) -prune -o -print | sed -e 's|[^/]*/|├── |g' > "$OUTPUT_FILE"

echo "Directory tree saved to $OUTPUT_FILE"