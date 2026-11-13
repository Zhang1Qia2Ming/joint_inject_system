#!/usr/bin/env python3
"""Decode Sensor4-7 pano H265 into native 1920x1200 UYVY on the NVMe cache."""

import json
import shutil
import subprocess
import sys
from pathlib import Path

from build_video_cache import (
    Lane,
    _extract_lanes,
    _probe_h265,
    _sha256,
    _write_timestamps,
)

WIDTH = 1920
HEIGHT = 1200
FRAME_BYTES = WIDTH * HEIGHT * 2
LANES = (
    Lane("s4", "/sensor/camera_pano_rightrear/orig/h265", 10, 30),
    Lane("s5", "/sensor/camera_pano_leftrear/orig/h265", 10, 30),
    Lane("s6", "/sensor/camera_pano_leftfront/orig/h265", 10, 30),
    Lane("s7", "/sensor/camera_pano_rightfront/orig/h265", 10, 30),
)


def _decode(input_path, output_path):
    _probe_h265(input_path, WIDTH, HEIGHT)
    command = [
        "ffmpeg", "-y", "-v", "error", "-i", str(input_path),
        "-f", "rawvideo", "-pix_fmt", "uyvy422", str(output_path),
    ]
    completed = subprocess.run(command, stderr=subprocess.PIPE, check=False)
    if completed.returncode != 0:
        raise RuntimeError(
            "ffmpeg failed for %s: %s"
            % (input_path, completed.stderr.decode("utf-8", "replace").strip())
        )
    size = Path(output_path).stat().st_size
    if size % FRAME_BYTES:
        raise RuntimeError("ffmpeg produced a partial UYVY frame for " + input_path.name)
    return size // FRAME_BYTES


def _free_bytes(path):
    usage = shutil.disk_usage(path)
    return usage.free


def build(mcap_path, output_dir):
    mcap_path = Path(mcap_path)
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    needed = FRAME_BYTES * 400 * len(LANES)
    free = _free_bytes(output_dir)
    if free < needed + 2 * 1024 ** 3:
        raise RuntimeError(
            "NVMe free %d bytes is below the %d-byte pano cache plus 2 GiB reserve"
            % (free, needed)
        )
    staging = output_dir / ".staging"
    if staging.exists():
        shutil.rmtree(staging)
    staging.mkdir()
    print("extract", mcap_path, flush=True)
    timestamps = _extract_lanes(mcap_path, LANES, staging)
    lanes = {}
    for lane in LANES:
        source = staging / (lane.name + ".h265")
        payload = output_dir / (lane.name + ".uyvy")
        expected = len(timestamps[lane.name])
        if payload.exists() and payload.stat().st_size == expected * FRAME_BYTES:
            count = expected
            print("reuse", payload, flush=True)
        else:
            partial = output_dir / (lane.name + ".uyvy.partial")
            if partial.exists():
                partial.unlink()
            print("decode", lane.name, expected, flush=True)
            count = _decode(source, partial)
            partial.replace(payload)
        if count != expected:
            raise RuntimeError(
                "%s decoded %d frames, timestamps %d" % (lane.name, count, expected)
            )
        stamp_path = output_dir / (lane.name + ".timestamps_ns")
        _write_timestamps(stamp_path, timestamps[lane.name])
        lanes[lane.name] = {
            "sensor": int(lane.name[1:]),
            "topic": lane.topic,
            "source_fps": lane.source_fps,
            "output_fps": lane.output_fps,
            "frame_count": count,
            "frame_bytes": FRAME_BYTES,
            "payload_file": payload.name,
            "timestamps_file": stamp_path.name,
            "sha256": _sha256(payload),
        }
        source.unlink()
    manifest = {
        "version": 1,
        "role": "shadow_before_h2c",
        "width": WIDTH,
        "height": HEIGHT,
        "pixel_format": "uyvy422",
        "source_mcap": str(mcap_path.resolve()),
        "lanes": lanes,
    }
    (output_dir / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    shutil.rmtree(staging)
    print(output_dir.resolve(), flush=True)


def main(argv=None):
    parser_argv = list(sys.argv[1:] if argv is None else argv)
    if len(parser_argv) != 2:
        print("usage: build_shadow_pano_cache.py MCAP OUTPUT_DIR", file=sys.stderr)
        return 2
    build(parser_argv[0], parser_argv[1])
    return 0


if __name__ == "__main__":
    sys.exit(main())
