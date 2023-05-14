#!/bin/bash

# Get current time in nanoseconds
timestamp=$(date +%s%N)

# Define output directory
output_dir="/home/joey/udp_logs/"

# Run tcpdump with timestamp in filename and output to directory
sudo tcpdump -i enx00e04c68054a udp port 54003 and src 169.254.212.117 -w "${output_dir}capture_$timestamp.pcap"