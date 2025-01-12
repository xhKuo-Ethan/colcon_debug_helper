#!/bin/bash

# Get the absolute path of the script's directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Navigate to two levels up from the script's directory
cd "$SCRIPT_DIR/../.."

# Remove the build directory if it exists
if [ -d "build" ]; then
    echo "Removing build directory..."
    rm -rf build
else
    echo "No build directory found"
fi

if [ -d "install" ]; then
    echo "Removing build directory..."
    rm -rf install
else
    echo "No install directory found"
fi

if [ -d "log" ]; then
    echo "Removing build directory..."
    rm -rf log
else
    echo "No log directory found"
fi

# Execute the colcon build command
echo "Running colcon build..."

colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
