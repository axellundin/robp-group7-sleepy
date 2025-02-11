#!/bin/bash

# Find RealSense camera line and extract bus and device numbers
device_info=$(lsusb | grep "RealSense.*Depth Camera" | grep -o "Bus \([0-9]\+\) Device \([0-9]\+\)" | sed -E 's/Bus ([0-9]+) Device ([0-9]+)/\1 \2/')

# Check if device was found
if [ -z "$device_info" ]; then
    echo "RealSense camera not found"
    exit 1
fi

# Split the output into bus and device numbers
read bus device <<< "$device_info"

# Format numbers with leading zeros (3 digits)
bus_padded=$(printf "%03d" $bus)
device_padded=$(printf "%03d" $device)

# Construct the device path
device_path="/dev/bus/usb/${bus_padded}/${device_padded}"

# Check if the device path exists
if [ ! -e "$device_path" ]; then
    echo "Device path ${device_path} does not exist"
    exit 1
fi
echo "$device_path"
# Call the c-script with the device path
/home/sleepy/robp-group7-sleepy/util/usbreset "$device_path"
