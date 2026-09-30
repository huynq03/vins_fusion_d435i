#!/usr/bin/env bash
# Record both external cameras at once. Stop with Ctrl+C.
set -u

OUTDIR="/home/hann/vins_fusion_d435i_local/bags/recordings"
WIDTH=1280
HEIGHT=720
FPS=30

CAM1="/dev/v4l/by-id/usb-icSpring_icspring_camera_202412091011-video-index0"
CAM2="/dev/v4l/by-id/usb-Vimicro_corp._Integrated_Camera-video-index0"

# Both cameras are mounted upside down.
# hflip,vflip = 180 degrees. Use "null" (no-op filter) for no flip.
FLIP1="hflip,vflip"
FLIP2="hflip,vflip"

mkdir -p "$OUTDIR"
STAMP=$(date +%Y%m%d_%H%M%S)
OUT1="$OUTDIR/rec_${STAMP}_cam1.mp4"
OUT2="$OUTDIR/rec_${STAMP}_cam2.mp4"

for c in "$CAM1" "$CAM2"; do
    if [ ! -e "$c" ]; then
        echo "error: camera not found at $c" >&2
        echo "plugged in? check with: ls -l /dev/v4l/by-id/" >&2
        exit 1
    fi
done

grab() {
    exec ffmpeg -hide_banner -loglevel error -nostdin \
        -f v4l2 -input_format mjpeg -video_size "${WIDTH}x${HEIGHT}" \
        -framerate "$FPS" -i "$1" \
        -vf "$3" -vsync cfr -r "$FPS" \
        -c:v libx264 -preset veryfast -crf 23 -pix_fmt yuv420p \
        -movflags +frag_keyframe+empty_moov+default_base_moof "$2"
}

echo "recording ${WIDTH}x${HEIGHT} @ ${FPS}fps -- Ctrl+C to stop"
grab "$CAM1" "$OUT1" "$FLIP1" & P1=$!
grab "$CAM2" "$OUT2" "$FLIP2" & P2=$!

# Ctrl+C in a terminal already delivers SIGINT to the whole process group,
# so both ffmpegs get it directly. Do NOT forward another signal here: a
# second signal makes ffmpeg take its "immediate exit" path, which skips
# writing the trailer and leaves the mp4 without a moov atom. Only forward
# on TERM, for a kill from outside the terminal.
trap '' INT
trap 'kill -TERM $P1 $P2 2>/dev/null' TERM
wait $P1; wait $P2

echo
for f in "$OUT1" "$OUT2"; do
    [ -s "$f" ] && echo "saved $f ($(du -h "$f" | cut -f1))" || echo "nothing recorded to $f"
done
