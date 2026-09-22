#!/usr/bin/env bash

set -euo pipefail

if [[ $# -lt 3 || $# -gt 5 ]]; then
    echo "Usage: $0 <input.mp4> <stabilized.mp4> <comparison.mp4> [left-label] [right-label]" >&2
    exit 1
fi

input=$1
stabilized=$2
output=$3
left_label=${4:-Input}
right_label=${5:-Stabilized}

for command in ffmpeg ffprobe; do
    if ! command -v "$command" >/dev/null 2>&1; then
        echo "Required command not found: $command" >&2
        exit 1
    fi
done

for video in "$input" "$stabilized"; do
    if [[ ! -f "$video" ]]; then
        echo "Video not found: $video" >&2
        exit 1
    fi
done

video_size() {
    ffprobe -v error -select_streams v:0 \
        -show_entries stream=width,height \
        -of csv=p=0:s=x "$1"
}

input_size=$(video_size "$input")
stabilized_size=$(video_size "$stabilized")
input_width=${input_size%x*}
input_height=${input_size#*x}
stabilized_width=${stabilized_size%x*}
stabilized_height=${stabilized_size#*x}

canvas_width=$((input_width > stabilized_width ? input_width : stabilized_width))
canvas_height=$((input_height > stabilized_height ? input_height : stabilized_height))

# H.264/yuv420p requires even dimensions.
canvas_width=$((canvas_width + canvas_width % 2))
canvas_height=$((canvas_height + canvas_height % 2))

input_x=$(((canvas_width - input_width) / 2))
input_y=$(((canvas_height - input_height) / 2))
stabilized_x=$(((canvas_width - stabilized_width) / 2))
stabilized_y=$(((canvas_height - stabilized_height) / 2))

# Labels are passed as filter arguments rather than interpolated into the filter text.
# Restrict the two characters that have structural meaning in drawtext.
left_label=${left_label//:/\\:}
left_label=${left_label//\'/\\\'}
right_label=${right_label//:/\\:}
right_label=${right_label//\'/\\\'}

filter="[0:v]pad=${canvas_width}:${canvas_height}:${input_x}:${input_y}:black,"
filter+="drawtext=text='${left_label}':x=16:y=16:fontsize=30:fontcolor=white:"
filter+="box=1:boxcolor=black@0.55[left];"
filter+="[1:v]pad=${canvas_width}:${canvas_height}:${stabilized_x}:${stabilized_y}:black,"
filter+="drawtext=text='${right_label}':x=16:y=16:fontsize=30:fontcolor=white:"
filter+="box=1:boxcolor=black@0.55[right];"
filter+="[left][right]hstack=inputs=2[video]"

ffmpeg -y \
    -i "$input" \
    -i "$stabilized" \
    -filter_complex "$filter" \
    -map '[video]' \
    -map '0:a?' \
    -c:v libx264 \
    -preset medium \
    -crf 18 \
    -pix_fmt yuv420p \
    -c:a copy \
    -shortest \
    -movflags +faststart \
    "$output"

echo "Created $output (${canvas_width}x${canvas_height} per side)"
