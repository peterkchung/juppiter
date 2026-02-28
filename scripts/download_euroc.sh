#!/usr/bin/env bash
# About: Downloads EuRoC MAV dataset sequences to data/.
# Usage: bash scripts/download_euroc.sh [SEQUENCE...]
# Example: bash scripts/download_euroc.sh MH_01_easy MH_02_easy V1_01_easy

set -euo pipefail

BASE_URL="http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset"
DATA_DIR="$(cd "$(dirname "$0")/.." && pwd)/data"

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

download_sequence() {
  local seq="$1"
  local dest="$DATA_DIR/$seq"

  if [[ -d "$dest/mav0" ]]; then
    echo "Skipping $seq (already exists at $dest/mav0)"
    return 0
  fi

  local url="$BASE_URL/${seq}/${seq}.zip"
  local zipfile="$DATA_DIR/${seq}.zip"

  echo "Downloading $seq..."
  mkdir -p "$DATA_DIR"
  wget -q --show-progress -O "$zipfile" "$url"

  echo "Extracting $seq..."
  unzip -q -o "$zipfile" -d "$dest"
  rm "$zipfile"

  echo "$seq ready at $dest/mav0"
}

# Default to MH_01_easy if no args
targets=("${@:-MH_01_easy}")
if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

for seq in "${targets[@]}"; do
  validate_sequence "$seq"
  download_sequence "$seq"
done

echo "Done."
