#!/bin/bash
# Script to view all seeker camera images in separate windows

echo "Opening all camera image viewers..."
echo "Press Ctrl+C to close all windows"

# Function to cleanup on exit
cleanup() {
    echo "Closing all image viewers..."
    pkill -f "rqt_image_view"
    exit 0
}

trap cleanup SIGINT SIGTERM

# Launch image viewers for each camera
ros2 run rqt_image_view rqt_image_view /fisheye/left/image_raw &
sleep 1
ros2 run rqt_image_view rqt_image_view /fisheye/right/image_raw &
sleep 1
ros2 run rqt_image_view rqt_image_view /fisheye/bright/image_raw &
sleep 1
ros2 run rqt_image_view rqt_image_view /fisheye/bleft/image_raw &
sleep 1

# Launch disparity image viewers
ros2 run rqt_image_view rqt_image_view /front/disparity/image_raw &
sleep 1
ros2 run rqt_image_view rqt_image_view /right/disparity/image_raw &
sleep 1

echo "All viewers launched. Press Ctrl+C to close all."
wait
