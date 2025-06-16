#!/usr/bin/env bash

repo_root="$PWD"
output_file="$repo_root/merged_all_files.txt"

# start fresh
> "$output_file"

find "$repo_root" \
  -path "$repo_root/services/staticRankedQuestions.ts" -prune -o \
  -path "$repo_root/.venv" -prune -o \
  -path "$repo_root/**/.venv" -prune -o \
  -type f \( -iname '*.xml' -o -iname '*.py' -o -iname '*.cpp' \) \
  -print | while IFS= read -r file; do

  printf "\n=== File: %s ===\n" "$file" >> "$output_file"
  cat "$file"                                           >> "$output_file"
  printf "\n--------------------------------------\n"    >> "$output_file"
done
