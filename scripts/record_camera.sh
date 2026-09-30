#!/usr/bin/env bash
# Record from the external USB camera. Stop with Ctrl+C.
set -u

OUTDIR="/home/hann/vins_fusion_d435i_local/bags/recordings"
CAM="/dev/v4l/by-id/usb-icSpring_icspring_camera_202412091011-video-index0"
WIDTH=1280
HEIGHT=720
FPS=30

mkdir -p "$OUTDIR"

if [ ! -e "$CAM" ]; then
    echo "error: camera not found at $CAM" >&2
    echo "is it plugged in? check with: lsusb | grep 5986:0299" >&2
    exit 1
fi

if fuser "$CAM" >/dev/null 2>&1; then
    echo "error: camera is busy (another program has it open)" >&2
    exit 1
fi

OUTFILE="$OUTDIR/rec_$(date +%Y%m%d_%H%M%S).mp4"

echo "recording to $OUTFILE"
echo "press Ctrl+C to stop"

# ffmpeg runs in the foreground, so Ctrl+C delivers SIGINT straight to it.
# ffmpeg then finalizes the mp4 (writes the moov atom) before exiting --
# a hard kill would leave the file unplayable.
ffmpeg -hide_banner -loglevel warning -stats \
    -f v4l2 -input_format mjpeg -video_size "${WIDTH}x${HEIGHT}" \
    -framerate "$FPS" -i "$CAM" \
    -vsync cfr -r "$FPS" \
    -c:v libx264 -preset veryfast -crf 23 -pix_fmt yuv420p \
    "$OUTFILE"

echo
if [ -s "$OUTFILE" ]; then
    DUR=$(ffprobe -v error -show_entries format=duration \
          -of default=noprint_wrappers=1:nokey=1 "$OUTFILE" 2>/dev/null)
    SIZE=$(du -h "$OUTFILE" | cut -f1)
    # LC_NUMERIC=C: printf %f rejects "8.034000" under a comma-decimal locale.
    LC_NUMERIC=C printf 'saved %s  (%.1fs, %s)\n' "$OUTFILE" "${DUR:-0}" "$SIZE"
else
    echo "nothing recorded" >&2
    rm -f "$OUTFILE"
    exit 1
fi
