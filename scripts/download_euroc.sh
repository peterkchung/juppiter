#!/usr/bin/env bash
# About: Downloads EuRoC MAV dataset sequences to data/.
# Usage: bash scripts/download_euroc.sh [SEQUENCE...]
# Example: bash scripts/download_euroc.sh MH_01_easy MH_02_easy V1_01_easy

set -euo pipefail

DATA_DIR="$(cd "$(dirname "$0")/.." && pwd)/data"

# ETH Research Collection bundle URLs (individual sequence downloads are no longer available)
declare -A BUNDLE_URLS=(
  ["MH"]="https://www.research-collection.ethz.ch/bitstreams/7b2419c1-62b5-4714-b7f8-485e5fe3e5fe/download"
  ["V1"]="https://www.research-collection.ethz.ch/bitstreams/02ecda9a-298f-498b-970c-b7c44334d880/download"
  ["V2"]="https://www.research-collection.ethz.ch/bitstreams/ea12bc01-3677-4b4c-853d-87c7870b8c44/download"
)

declare -A BUNDLE_NAMES=(
  ["MH"]="machine_hall"
  ["V1"]="vicon_room1"
  ["V2"]="vicon_room2"
)

SEQUENCES=(
  MH_01_easy MH_02_easy MH_03_medium MH_04_difficult MH_05_difficult
  V1_01_easy V1_02_medium V1_03_difficult
  V2_01_easy V2_02_medium V2_03_difficult
)

usage() {
  echo "Usage: $0 [SEQUENCE...]"
  echo ""
  echo "Available sequences:"
  printf "  %s\n" "${SEQUENCES[@]}"
  echo ""
  echo "If no sequence is specified, downloads MH_01_easy."
  echo ""
  echo "Note: ETH bundles all sequences per environment into one zip."
  echo "Downloading any MH sequence downloads all MH sequences (~12 GB)."
}

validate_sequence() {
  local seq="$1"
  for valid in "${SEQUENCES[@]}"; do
    [[ "$seq" == "$valid" ]] && return 0
  done
  echo "Error: unknown sequence '$seq'"
  usage
  exit 1
}

get_bundle_key() {
  echo "${1%%_*}"
}

download_bundle() {
  local bundle_key="$1"
  local url="${BUNDLE_URLS[$bundle_key]}"
  local name="${BUNDLE_NAMES[$bundle_key]}"
  local zipfile="$DATA_DIR/${name}.zip"
  local marker="$DATA_DIR/.${bundle_key}_downloaded"

  if [[ -f "$marker" ]]; then
    return 0
  fi

  echo "Downloading $name bundle (this may take a while)..."
  mkdir -p "$DATA_DIR"
  wget --progress=bar:force -O "$zipfile" "$url"

  echo "Extracting $name bundle..."
  unzip -q -o "$zipfile" -d "$DATA_DIR"
  rm "$zipfile"

  # Extract inner sequence zips (bundle contains per-sequence zips)
  local bundle_dir="$DATA_DIR/$name"
  if [[ -d "$bundle_dir" ]]; then
    for seq_dir in "$bundle_dir"/*/; do
      local seq_name
      seq_name="$(basename "$seq_dir")"
      local inner_zip="$seq_dir/${seq_name}.zip"
      if [[ -f "$inner_zip" ]]; then
        echo "Extracting $seq_name..."
        unzip -q -o "$inner_zip" -d "$seq_dir"
        rm "$inner_zip"
      fi
      # Symlink to flat path: data/MH_01_easy -> data/machine_hall/MH_01_easy
      local link="$DATA_DIR/$seq_name"
      if [[ ! -e "$link" ]]; then
        ln -s "$name/$seq_name" "$link"
      fi
    done
  fi

  touch "$marker"
  echo "$name bundle extracted."
}

# Default to MH_01_easy if no args
targets=("${@:-MH_01_easy}")
if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

# Determine which bundles we need
declare -A needed_bundles
for seq in "${targets[@]}"; do
  validate_sequence "$seq"
  key=$(get_bundle_key "$seq")
  needed_bundles[$key]=1
done

# Download required bundles
for key in "${!needed_bundles[@]}"; do
  download_bundle "$key"
done

# Verify requested sequences exist
for seq in "${targets[@]}"; do
  if [[ -d "$DATA_DIR/$seq/mav0" ]]; then
    echo "$seq ready at $DATA_DIR/$seq/mav0"
  else
    echo "Warning: $seq not found after extraction. Check $DATA_DIR for contents."
  fi
done

echo "Done."
