#!/bin/bash

# Check if FFmpeg is installed
if ! command -v ffmpeg &> /dev/null; then
    echo "FFmpeg is not installed. Please install FFmpeg and try again."
    exit 1
fi

# Check if two input files are provided
if [ $# -ne 2 ]; then
    echo "Usage: $0 <video1.mp4> <video2.mp4>"
    exit 1
fi

# Extract filenames and extension
input1=$1
input2=$2
output="output.mp4"

# Check if input files exist
if [ ! -f "$input1" ]; then
    echo "File $input1 does not exist."
    exit 1
fi

if [ ! -f "$input2" ]; then
    echo "File $input2 does not exist."
    exit 1
fi

# Get video dimensions
video1_dimensions=$(ffprobe -v error -select_streams v:0 -show_entries stream=width,height -of csv=s=x:p=0 "$input1")
video2_dimensions=$(ffprobe -v error -select_streams v:0 -show_entries stream=width,height -of csv=s=x:p=0 "$input2")

# Calculate output video dimensions
width1=$(cut -d 'x' -f 1 <<< "$video1_dimensions")
height1=$(cut -d 'x' -f 2 <<< "$video1_dimensions")

width2=$(cut -d 'x' -f 1 <<< "$video2_dimensions")
height2=$(cut -d 'x' -f 2 <<< "$video2_dimensions")

max_width=$((width1 > width2 ? width1 : width2))
max_height=$((height1 > height2 ? height1 : height2))

# Resize videos to match the maximum height
resized1="${input1%.*}_resized.mp4"
resized2="${input2%.*}_resized.mp4"

ffmpeg -i "$input1" -vf "scale=-1:$max_height" "$resized1"
ffmpeg -i "$input2" -vf "scale=-1:$max_height" "$resized2"

# Concatenate resized videos side by side
ffmpeg -i "$resized1" -i "$resized2" -filter_complex "\
    [0:v]pad=$max_width:$max_height:(max($max_width\,iw)):(max($max_height\,ih)) [left]; \
    [1:v]pad=$max_width:$max_height:(max($max_width\,iw)):(max($max_height\,ih)) [right]; \
    [left][right]hstack" "$output"

echo "Output video generated: $output"

# Clean up resized videos
rm "$resized1" "$resized2"
