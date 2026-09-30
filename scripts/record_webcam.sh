#!/usr/bin/env bash
# Record the external USB webcams to MP4 with ffmpeg (run on the host). Stop with Ctrl+C.
# Usage: ./scripts/record_webcam.sh [cam1|cam2|all]   (default: cam1)
set -u

OUTDIR="$(realpath "$(dirname "$0")/..")/bags/recordings"
WIDTH=1280 HEIGHT=720 FPS=30
CAM1="/dev/v4l/by-id/usb-icSpring_icspring_camera_202412091011-video-index0"
CAM2="/dev/v4l/by-id/usb-Vimicro_corp._Integrated_Camera-video-index0"
# Both cameras are mounted upside down: hflip,vflip = 180 degrees ("null" = no flip).
FLIP="hflip,vflip"

case "${1:-cam1}" in
    cam1) CAMS=("$CAM1") ;;
    cam2) CAMS=("$CAM2") ;;
    all) CAMS=("$CAM1" "$CAM2") ;;
    *) echo "Usage: $0 [cam1|cam2|all]" >&2; exit 2 ;;
esac

for c in "${CAMS[@]}"; do
    [[ -e "$c" ]] || { echo "Camera not found: $c (check: ls -l /dev/v4l/by-id/)" >&2; exit 1; }
    fuser "$c" >/dev/null 2>&1 && { echo "Camera busy: $c" >&2; exit 1; }
done

mkdir -p "$OUTDIR"
STAMP=$(date +%Y%m%d_%H%M%S)
OUTS=() PIDS=()
for i in "${!CAMS[@]}"; do
    OUT="$OUTDIR/rec_${STAMP}_cam$((i + 1)).mp4"
    OUTS+=("$OUT")
    # Fragmented MP4 stays playable even if ffmpeg is killed before writing the trailer.
    ffmpeg -hide_banner -loglevel error -nostdin \
        -f v4l2 -input_format mjpeg -video_size "${WIDTH}x${HEIGHT}" -framerate "$FPS" -i "${CAMS[i]}" \
        -vf "$FLIP" -vsync cfr -r "$FPS" \
        -c:v libx264 -preset veryfast -crf 23 -pix_fmt yuv420p \
        -movflags +frag_keyframe+empty_moov+default_base_moof "$OUT" &
    PIDS+=($!)
done

echo "Recording ${WIDTH}x${HEIGHT}@${FPS}fps -- Ctrl+C to stop"
# Ctrl+C already reaches every ffmpeg via the process group. Forwarding a second
# signal would make ffmpeg exit immediately without finalizing the file.
trap '' INT
trap 'kill -TERM "${PIDS[@]}" 2>/dev/null' TERM
wait "${PIDS[@]}"

echo
for f in "${OUTS[@]}"; do
    [[ -s "$f" ]] && echo "saved $f ($(du -h "$f" | cut -f1))" || { echo "nothing recorded to $f"; rm -f "$f"; }
done
