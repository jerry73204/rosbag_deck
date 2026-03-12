#!/usr/bin/env bash
#
# Download test bag files for rosbag-deck development.
#
# Features:
#   - aria2c for multi-threaded downloads with continuation
#   - Local SHA-256 checksum cache (computed after first download, verified on re-runs)
#   - Skips files that already exist and match cached checksum
#
# Usage:
#   ./scripts/download-test-bags.sh          # download all
#   ./scripts/download-test-bags.sh small    # small bags only (~500 KB)
#   ./scripts/download-test-bags.sh large    # large bags only (~50 MB+)
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
DEST="$BASE_DIR/test_bags"
CHECKSUM_FILE="$DEST/.checksums.sha256"

# aria2c common flags
ARIA2_OPTS=(--continue=true --max-connection-per-server=4 --min-split-size=1M --auto-file-renaming=false --allow-overwrite=false --console-log-level=warn --summary-interval=0)

# ── Helpers ──────────────────────────────────────────────────────────────────

die() { echo "ERROR: $*" >&2; exit 1; }

require_cmd() {
    command -v "$1" >/dev/null 2>&1 || die "'$1' is required but not found. Install it first."
}

# Look up cached checksum for a file path (relative to DEST).
cached_checksum() {
    local rel_path="$1"
    [[ -f "$CHECKSUM_FILE" ]] || return 1
    grep -F "  $rel_path" "$CHECKSUM_FILE" 2>/dev/null | head -1 | cut -d' ' -f1
}

# Save checksum to cache.
save_checksum() {
    local file="$1" rel_path="$2"
    local sha; sha="$(sha256sum "$file" | cut -d' ' -f1)"
    mkdir -p "$(dirname "$CHECKSUM_FILE")"
    # Remove old entry if exists, then append.
    if [[ -f "$CHECKSUM_FILE" ]]; then
        grep -vF "  $rel_path" "$CHECKSUM_FILE" > "$CHECKSUM_FILE.tmp" 2>/dev/null || true
        mv "$CHECKSUM_FILE.tmp" "$CHECKSUM_FILE"
    fi
    echo "$sha  $rel_path" >> "$CHECKSUM_FILE"
}

# Verify file against cached checksum.
verify_cached() {
    local file="$1" rel_path="$2"
    [[ -f "$file" ]] || return 1
    local expected; expected="$(cached_checksum "$rel_path")" || return 1
    local actual; actual="$(sha256sum "$file" | cut -d' ' -f1)"
    [[ "$actual" == "$expected" ]]
}

# Download a file with aria2c. Skips if file exists and cached checksum matches.
#   download <url> <dest_path>
download() {
    local url="$1" dest="$2"
    local dir; dir="$(dirname "$dest")"
    local filename; filename="$(basename "$dest")"
    local rel_path="${dest#$DEST/}"

    # Skip if already downloaded and checksum matches cache.
    if [[ -f "$dest" ]] && verify_cached "$dest" "$rel_path"; then
        echo "  SKIP  $rel_path (checksum OK)"
        return 0
    fi

    mkdir -p "$dir"
    echo "  GET   $rel_path"
    if ! aria2c "${ARIA2_OPTS[@]}" --dir="$dir" --out="$filename" "$url"; then
        echo "  WARN  Failed to download: $url" >&2
        return 1
    fi

    # Save checksum after successful download.
    save_checksum "$dest" "$rel_path"
}

# Download and extract a tar.gz or zip archive.
#   download_archive <url> <dest_dir> <archive_name>
download_archive() {
    local url="$1" dest_dir="$2" archive_name="$3"
    local archive_path="$dest_dir/$archive_name"

    # Skip if extraction marker exists.
    if [[ -f "$dest_dir/.extracted" ]]; then
        echo "  SKIP  $dest_dir (already extracted)"
        return 0
    fi

    mkdir -p "$dest_dir"
    download "$url" "$archive_path" || return 1

    echo "  UNPACK $archive_path"
    case "$archive_name" in
        *.tar.gz|*.tgz) tar xzf "$archive_path" -C "$dest_dir" ;;
        *.zip)          unzip -qo "$archive_path" -d "$dest_dir" ;;
        *)              die "Unknown archive format: $archive_name" ;;
    esac

    touch "$dest_dir/.extracted"
    rm -f "$archive_path"
}

# ── Bag definitions ──────────────────────────────────────────────────────────

ROS2_RAW="https://raw.githubusercontent.com/ros2/rosbag2/humble"
MCAP_LFS="https://media.githubusercontent.com/media/foxglove/mcap/main"

download_small() {
    echo ""
    echo "=== Small test bags ==="

    # -- ros2/rosbag2: talker (single topic, sqlite3, ~18 KB) --
    echo ""
    echo "--- talker (sqlite3, single topic, 20 msgs) ---"
    download "$ROS2_RAW/rosbag2_py/test/resources/talker/talker.db3" \
             "$DEST/rosbag2/talker/talker.db3"
    download "$ROS2_RAW/rosbag2_py/test/resources/talker/metadata.yaml" \
             "$DEST/rosbag2/talker/metadata.yaml"

    # -- ros2/rosbag2: cdr_test (multi-type, sqlite3, ~17 KB) --
    echo ""
    echo "--- cdr_test (sqlite3, multi-type, 7 msgs) ---"
    download "$ROS2_RAW/rosbag2_tests/resources/cdr_test/cdr_test_0.db3" \
             "$DEST/rosbag2/cdr_test/cdr_test_0.db3"
    download "$ROS2_RAW/rosbag2_tests/resources/cdr_test/metadata.yaml" \
             "$DEST/rosbag2/cdr_test/metadata.yaml"

    # -- ros2/rosbag2: test_bag_for_seek sqlite3 --
    echo ""
    echo "--- test_bag_for_seek (sqlite3, 5 msgs) ---"
    download "$ROS2_RAW/rosbag2_transport/test/resources/sqlite3/test_bag_for_seek/test_bag_for_seek_0.db3" \
             "$DEST/rosbag2/test_bag_for_seek_sqlite3/test_bag_for_seek_0.db3"
    download "$ROS2_RAW/rosbag2_transport/test/resources/sqlite3/test_bag_for_seek/metadata.yaml" \
             "$DEST/rosbag2/test_bag_for_seek_sqlite3/metadata.yaml"

    # -- ros2/rosbag2: test_bag_for_seek mcap --
    echo ""
    echo "--- test_bag_for_seek (mcap, 5 msgs) ---"
    download "$ROS2_RAW/rosbag2_transport/test/resources/mcap/test_bag_for_seek/test_bag_for_seek_0.mcap" \
             "$DEST/rosbag2/test_bag_for_seek_mcap/test_bag_for_seek_0.mcap"
    download "$ROS2_RAW/rosbag2_transport/test/resources/mcap/test_bag_for_seek/metadata.yaml" \
             "$DEST/rosbag2/test_bag_for_seek_mcap/metadata.yaml"

    # -- ros2/rosbag2: empty_bag --
    echo ""
    echo "--- empty_bag (sqlite3, 0 msgs) ---"
    download "$ROS2_RAW/ros2bag/test/resources/empty_bag/empty_bag_0.db3" \
             "$DEST/rosbag2/empty_bag/empty_bag_0.db3"
    download "$ROS2_RAW/ros2bag/test/resources/empty_bag/metadata.yaml" \
             "$DEST/rosbag2/empty_bag/metadata.yaml"

    # -- ros2/rosbag2: wbag (multi-file split, 8 topics, ~440 KB) --
    echo ""
    echo "--- wbag (sqlite3, multi-file split, 8 topics, 6074 msgs) ---"
    for i in 0 1 2 3 4; do
        download "$ROS2_RAW/rosbag2_py/test/resources/wbag/wbag_${i}.db3" \
                 "$DEST/rosbag2/wbag/wbag_${i}.db3"
    done
    download "$ROS2_RAW/rosbag2_py/test/resources/wbag/metadata.yaml" \
             "$DEST/rosbag2/wbag/metadata.yaml"

    # -- ros2/rosbag2: convert_a (sqlite3, 150 msgs) --
    echo ""
    echo "--- convert_a (sqlite3, 2 topics, 150 msgs) ---"
    download "$ROS2_RAW/rosbag2_py/test/resources/convert_a/rewriter_a_0.db3" \
             "$DEST/rosbag2/convert_a/rewriter_a_0.db3"
    download "$ROS2_RAW/rosbag2_py/test/resources/convert_a/metadata.yaml" \
             "$DEST/rosbag2/convert_a/metadata.yaml"

    # -- foxglove/mcap test data --
    echo ""
    echo "--- foxglove mcap test files ---"
    download "$MCAP_LFS/testdata/mcap/demo.mcap" \
             "$DEST/mcap/demo.mcap"
    download "$MCAP_LFS/rust/tests/data/compressed.mcap" \
             "$DEST/mcap/compressed.mcap"
    download "$MCAP_LFS/rust/tests/data/uncompressed.mcap" \
             "$DEST/mcap/uncompressed.mcap"

    # -- foxglove/mcap conformance: minimal set --
    echo ""
    echo "--- foxglove mcap conformance (selected) ---"
    local CONF="$MCAP_LFS/tests/conformance/data"
    download "$CONF/NoData/NoData.mcap" \
             "$DEST/mcap/conformance/NoData.mcap"
    download "$CONF/OneMessage/OneMessage.mcap" \
             "$DEST/mcap/conformance/OneMessage.mcap"
    download "$CONF/OneMessage/OneMessage-ch-chx-mx-pad-rch-rsh-st-sum.mcap" \
             "$DEST/mcap/conformance/OneMessage-all-features.mcap"
    download "$CONF/TenMessages/TenMessages-ch-chx-mx-st-sum.mcap" \
             "$DEST/mcap/conformance/TenMessages-chunked.mcap"
}

download_large() {
    echo ""
    echo "=== Large test bags ==="

    # -- Zenodo #7780490: LiDAR-Camera calibration, faro subset (46 MB) --
    echo ""
    echo "--- Zenodo: faro LiDAR-Camera calibration (~46 MB) ---"
    download_archive \
        "https://zenodo.org/records/7780490/files/faro.tar.gz?download=1" \
        "$DEST/zenodo_7780490" \
        "faro.tar.gz"

    # -- Zenodo #14209720: Autoware fault scenarios (413 MB) --
    echo ""
    echo "--- Zenodo: Autoware fault scenario bag (~413 MB) ---"
    download_archive \
        "https://zenodo.org/records/14209720/files/rosbag2_2024_06_18-17_11_08.zip?download=1" \
        "$DEST/zenodo_14209720" \
        "rosbag2_2024_06_18-17_11_08.zip"
}

# ── Main ─────────────────────────────────────────────────────────────────────

require_cmd aria2c
require_cmd sha256sum

FILTER="${1:-all}"

echo "Downloading test bags to: $DEST"

case "$FILTER" in
    small) download_small ;;
    large) download_large ;;
    all)   download_small; download_large ;;
    *)     die "Unknown filter: $FILTER (use 'small', 'large', or 'all')" ;;
esac

echo ""
echo "Done. Bags are in: $DEST"
du -sh "$DEST" 2>/dev/null || true
