#!/bin/bash

BASE_DIR="$(dirname "$0")"

SUBDIRS=(
	"rosbag2_2025_09_30_11_43_28"
	"rosbag2_2025_09_30_12_25_37"
	"rosbag2_2025_09_30_12_28_46"
	"rosbag2_2025_09_30_12_34_10"
	"rosbag2_2025_09_30_12_40_34"
	"rosbag2_2025_09_30_12_43_58"
	"rosbag2_2025_09_30_12_45_47"
	"rosbag2_2025_09_30_12_52_32"
	"rosbag2_2025_09_30_16_52_17"
	"rosbag2_2025_09_30_17_17_49"
)

sudo apt-get update
sudo apt-get install -y zstd

pip3 install mcap
#pip install "numpy<1.24"


for subdir in "${SUBDIRS[@]}"; do
	echo "current directory $(pwd)"
	echo "Entering $subdir"
	cd "$BASE_DIR/$subdir" || { echo "Failed to cd into $subdir"; exit 1; }

	TARGET_DIR="${1:-.}"
	PYTHON_SCRIPT="${2:-open_mcap.py}"

	if [ ! -d "$BASE_DIR/outputs" ]; then
		mkdir -p "$BASE_DIR/outputs"
		echo "Created directory: $BASE_DIR/outputs"
	fi
	LOG_FILE="../outputs/log_${subdir}.txt"

	if [[ -f "$LOG_FILE" ]]; then
		echo "Output for $subdir already exists, skipping."
		echo ""
		cd ..
		continue
	fi

	find "$TARGET_DIR" -type f -name "*.zstd" | while read -r file; do
		zstd -d -f --rm "$file"
	done

	echo "All zstd files have been unfeeezen."

	find "$TARGET_DIR" -type f -name "*.mcap" | sort | while read -r mcap; do
		echo ""
		echo "$mcap"
		python3 "../$PYTHON_SCRIPT" "$mcap" --overwrite || { echo "Python script failed on $mcap"; exit 1; }
	done 2>&1 | tee  "../outputs/log_${subdir}.txt"

	echo "Finished $subdir"
	echo ""

	echo "1: $(pwd)"
	cd ".."
	echo "2: $(pwd)"
done

echo "All exec_all.bash in subdirectories processed."
