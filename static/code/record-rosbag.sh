#!/bin/bash
# Record ROS 2 bag with compression and metadata
# Usage: ./record-rosbag.sh <output_name> <duration_seconds>

set -e

OUTPUT_NAME=${1:-"humanoid_data"}
DURATION=${2:-60}
TOPICS="/camera/depth/image_raw /camera/color/image_raw /imu/data /joint_states /tf /tf_static"

echo "===== Recording ROS 2 Bag ====="
echo "Output: ${OUTPUT_NAME}"
echo "Duration: ${DURATION} seconds"
echo "Topics: ${TOPICS}"
echo ""

# Create output directory
mkdir -p ~/rosbags
cd ~/rosbags

# Record with compression
ros2 bag record \
  --output ${OUTPUT_NAME} \
  --duration ${DURATION} \
  --compression-mode file \
  --compression-format zstd \
  ${TOPICS}

echo ""
echo "===== Recording Complete ====="
echo "Bag saved to: ~/rosbags/${OUTPUT_NAME}"

# Show bag info
ros2 bag info ${OUTPUT_NAME}
