#!/usr/bin/env python3
"""Track visual features in a normal video and save an annotated MP4."""

import argparse
from pathlib import Path

import cv2
import numpy as np


def parse_args():
    parser = argparse.ArgumentParser(
        description="Draw Lucas-Kanade feature tracks on any OpenCV-readable video."
    )
    parser.add_argument("input", help="Input video path")
    parser.add_argument("output", help="Output MP4 path")
    parser.add_argument("--max-features", type=int, default=300)
    parser.add_argument("--quality", type=float, default=0.01)
    parser.add_argument("--min-distance", type=float, default=12.0)
    parser.add_argument("--trail-length", type=int, default=20)
    parser.add_argument(
        "--draw-trails",
        action="store_true",
        help="Draw green motion-history lines behind tracked points",
    )
    parser.add_argument("--redetect-every", type=int, default=10)
    parser.add_argument("--display", action="store_true", help="Show a preview window")
    return parser.parse_args()


def detect_features(gray, points, maximum, quality, minimum_distance):
    remaining = maximum - len(points)
    if remaining <= 0:
        return points

    mask = np.full(gray.shape, 255, dtype=np.uint8)
    for point in points:
        x, y = point[-1]
        cv2.circle(
            mask,
            (int(round(float(x))), int(round(float(y)))),
            int(round(minimum_distance)),
            0,
            -1,
        )

    detected = cv2.goodFeaturesToTrack(
        gray,
        mask=mask,
        maxCorners=remaining,
        qualityLevel=quality,
        minDistance=minimum_distance,
        blockSize=7,
    )
    if detected is not None:
        points.extend([feature.reshape(1, 2) for feature in detected])
    return points


def main():
    args = parse_args()
    if args.max_features < 1 or args.trail_length < 1 or args.redetect_every < 1:
        raise SystemExit("Feature counts, trail length, and redetection interval must be positive")

    input_path = Path(args.input).expanduser()
    output_path = Path(args.output).expanduser()
    if not input_path.is_file():
        raise SystemExit(f"Input video does not exist: {input_path}")
    output_path.parent.mkdir(parents=True, exist_ok=True)

    capture = cv2.VideoCapture(str(input_path))
    if not capture.isOpened():
        raise SystemExit(f"Could not open input video: {input_path}")

    ok, frame = capture.read()
    if not ok:
        capture.release()
        raise SystemExit("Input video contains no readable frames")

    height, width = frame.shape[:2]
    fps = capture.get(cv2.CAP_PROP_FPS)
    if not np.isfinite(fps) or fps <= 0:
        fps = 30.0

    writer = cv2.VideoWriter(
        str(output_path), cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height)
    )
    if not writer.isOpened():
        capture.release()
        raise SystemExit(f"Could not create output video: {output_path}")

    previous_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tracks = detect_features(
        previous_gray, [], args.max_features, args.quality, args.min_distance
    )
    frame_number = 0

    lk_params = dict(
        winSize=(21, 21),
        maxLevel=3,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
    )

    try:
        while True:
            annotated = frame.copy()
            if tracks:
                previous_points = np.float32([track[-1] for track in tracks])
                current_points, forward_status, _ = cv2.calcOpticalFlowPyrLK(
                    previous_gray,
                    cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY),
                    previous_points,
                    None,
                    **lk_params,
                )
                if current_points is None or forward_status is None:
                    tracks = []
                else:
                    reverse_points, reverse_status, _ = cv2.calcOpticalFlowPyrLK(
                        cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY),
                        previous_gray,
                        current_points,
                        None,
                        **lk_params,
                    )
                    if reverse_points is None or reverse_status is None:
                        tracks = []
                    else:
                        error = np.max(np.abs(previous_points - reverse_points), axis=1)
                        valid = forward_status.ravel().astype(bool)
                        valid &= reverse_status.ravel().astype(bool)
                        valid &= error < 1.0

                        updated_tracks = []
                        for track, point, keep in zip(tracks, current_points, valid):
                            x, y = point
                            if not keep or not (0 <= x < width and 0 <= y < height):
                                continue
                            track = np.vstack((track, point))[-args.trail_length :]
                            updated_tracks.append(track)
                        tracks = updated_tracks

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            if frame_number % args.redetect_every == 0 or len(tracks) < args.max_features // 2:
                tracks = detect_features(
                    gray, tracks, args.max_features, args.quality, args.min_distance
                )

            for track in tracks:
                path = np.round(track).astype(np.int32)
                if args.draw_trails and len(path) > 1:
                    cv2.polylines(annotated, [path], False, (0, 255, 0), 1, cv2.LINE_AA)
                x, y = path[-1]
                cv2.circle(annotated, (x, y), 3, (0, 0, 255), -1, cv2.LINE_AA)

            cv2.putText(
                annotated,
                f"Tracked features: {len(tracks)}",
                (15, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
            writer.write(annotated)

            if args.display:
                cv2.imshow("Feature tracking", annotated)
                if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    break

            previous_gray = gray
            ok, frame = capture.read()
            if not ok:
                break
            frame_number += 1
    finally:
        capture.release()
        writer.release()
        cv2.destroyAllWindows()

    print(f"Saved feature-tracking video: {output_path}")


if __name__ == "__main__":
    main()
