#!/usr/bin/env python3
"""Build a persistent, timestamped UYVY cache from video MCAP topics."""

import argparse
import fcntl
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
import uuid
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from pathlib import Path

from mcap.reader import make_reader


WIDTH = 3840
HEIGHT = 2160
FRAME_BYTES = WIDTH * HEIGHT * 2
CONVERTER_VERSION = "joint-sensor-video-cache-1"


@dataclass(frozen=True)
class Lane:
    name: str
    topic: str
    source_fps: int
    output_fps: int = 30


DEFAULT_LANES = (
    Lane("tx1", "/sensor/camera_forward_far/orig/h265", 10),
    Lane("tx2", "/sensor/camera_forward_wide/orig/h265", 30),
    Lane("tx4", "/sensor/camera_backward/orig/h265", 10),
)


def _annex_b_payload(data):
    starts = [offset for offset in (data.find(b"\x00\x00\x00\x01"),
                                    data.find(b"\x00\x00\x01"))
              if offset >= 0]
    return None if not starts else data[min(starts):]


def _nal_units(payload):
    cursor = 0
    while cursor < len(payload):
        four = payload.find(b"\x00\x00\x00\x01", cursor)
        three = payload.find(b"\x00\x00\x01", cursor)
        starts = [(four, 4), (three, 3)]
        starts = [(offset, length) for offset, length in starts
                  if offset >= 0]
        if not starts:
            return
        start, marker_length = min(starts)
        nal_start = start + marker_length
        if nal_start < len(payload):
            yield payload[nal_start:]
        cursor = nal_start + 1


def _has_vps(payload):
    return any(((unit[0] >> 1) & 0x3F) == 32
               for unit in _nal_units(payload) if unit)


def _extract_lanes(mcap_path, lanes, staging):
    by_topic = {lane.topic: lane for lane in lanes}
    outputs = {
        lane.name: (staging / (lane.name + ".h265")).open("wb")
        for lane in lanes
    }
    timestamps = {lane.name: [] for lane in lanes}
    started = {lane.name: False for lane in lanes}
    try:
        with Path(mcap_path).open("rb") as source:
            reader = make_reader(source)
            for _schema, channel, message in reader.iter_messages(
                    topics=list(by_topic)):
                lane = by_topic.get(channel.topic)
                if lane is None:
                    continue
                payload = _annex_b_payload(message.data)
                if payload is None:
                    continue
                if not started[lane.name]:
                    if not _has_vps(payload):
                        continue
                    started[lane.name] = True
                lane_times = timestamps[lane.name]
                timestamp = int(message.log_time)
                if lane_times and timestamp <= lane_times[-1]:
                    raise RuntimeError(
                        "%s timestamps are not strictly increasing" % lane.name
                    )
                outputs[lane.name].write(payload)
                lane_times.append(timestamp)
    finally:
        for output in outputs.values():
            output.close()
    for lane in lanes:
        if not timestamps[lane.name]:
            raise RuntimeError("no VPS-gated messages for topic " + lane.topic)
    return timestamps


def _decode_h265(input_path, output_path, _width, _height):
    _probe_h265(input_path, _width, _height)
    command = [
        "ffmpeg", "-v", "error", "-i", str(input_path),
        "-f", "rawvideo", "-pix_fmt", "uyvy422", "pipe:1",
    ]
    with Path(output_path).open("wb") as output:
        completed = subprocess.run(
            command, stdout=output, stderr=subprocess.PIPE, check=False
        )
    if completed.returncode != 0:
        raise RuntimeError(
            "ffmpeg failed for %s: %s"
            % (input_path, completed.stderr.decode("utf-8", "replace").strip())
        )
    size = Path(output_path).stat().st_size
    if size % FRAME_BYTES:
        raise RuntimeError("ffmpeg produced a partial UYVY frame")
    return size // FRAME_BYTES


def _probe_h265(input_path, expected_width, expected_height):
    command = [
        "ffprobe", "-v", "error", "-select_streams", "v:0",
        "-show_entries", "stream=width,height", "-of", "json",
        str(input_path),
    ]
    completed = subprocess.run(
        command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False
    )
    if completed.returncode != 0:
        raise RuntimeError(
            "ffprobe failed for %s: %s"
            % (input_path, completed.stderr.decode("utf-8", "replace").strip())
        )
    try:
        streams = json.loads(completed.stdout.decode("utf-8"))["streams"]
        dimensions = [(int(stream["width"]), int(stream["height"]))
                      for stream in streams]
    except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
        raise RuntimeError("ffprobe returned invalid dimensions") from error
    expected = (expected_width, expected_height)
    if dimensions != [expected]:
        actual = "none" if not dimensions else "x".join(
            str(value) for value in dimensions[0]
        )
        raise RuntimeError(
            "expected exactly one %dx%d video stream, got %s"
            % (expected_width, expected_height, actual)
        )


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _write_timestamps(path, timestamps):
    Path(path).write_text(
        "".join("%d\n" % timestamp for timestamp in timestamps),
        encoding="ascii",
    )


def cache_key(dataset_id, lanes, width=WIDTH, height=HEIGHT,
              pixel_format="uyvy422",
              converter_version=CONVERTER_VERSION):
    identity = {
        "converter_version": converter_version,
        "dataset_id": dataset_id,
        "height": height,
        "pixel_format": pixel_format,
        "topics": [lane.topic for lane in lanes],
        "width": width,
    }
    encoded = json.dumps(
        identity, ensure_ascii=False, separators=(",", ":"), sort_keys=True
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _plain_child(root, name):
    relative = Path(name)
    if not name or relative.is_absolute() or relative.name != name:
        raise RuntimeError("cache file must be a plain relative filename")
    return root / relative


def _source_identity(mcap_path):
    stat = mcap_path.stat()
    return {
        "path": str(mcap_path.resolve()),
        "size": stat.st_size,
        "mtime_ns": stat.st_mtime_ns,
        "sha256": _sha256(mcap_path),
    }


def _validate_cache(cache_dir, source, key, lanes, frame_bytes):
    try:
        manifest = json.loads(
            (cache_dir / "manifest.json").read_text(encoding="utf-8")
        )
        if (
            manifest["version"] != 1
            or manifest["dataset_id"] != source["sha256"]
            or manifest["cache_key"] != key
            or cache_dir.name != key
            or manifest["converter_version"] != CONVERTER_VERSION
            or manifest["width"] != WIDTH
            or manifest["height"] != HEIGHT
            or manifest["pixel_format"] != "uyvy422"
            or set(manifest["lanes"]) != {lane.name for lane in lanes}
        ):
            return False
        cached_source = manifest["source_mcap"]
        if (
            cached_source["path"] != source["path"]
            or cached_source["size"] != source["size"]
            or cached_source["sha256"] != source["sha256"]
        ):
            return False
        for lane in lanes:
            entry = manifest["lanes"][lane.name]
            if (
                entry["topic"] != lane.topic
                or entry["source_fps"] != lane.source_fps
                or entry["output_fps"] != lane.output_fps
                or entry["pixel_format"] != "uyvy422"
                or entry["frame_bytes"] != frame_bytes
                or entry["frame_count"] <= 0
            ):
                return False
            payload = _plain_child(cache_dir, entry["payload_file"])
            if (
                payload.stat().st_size
                != entry["frame_count"] * entry["frame_bytes"]
                or _sha256(payload) != entry["sha256"]
            ):
                return False
            timestamp_meta = entry["timestamps"]
            timestamp_path = _plain_child(cache_dir, timestamp_meta["file"])
            if (
                timestamp_path.stat().st_size != timestamp_meta["byte_size"]
                or _sha256(timestamp_path) != timestamp_meta["sha256"]
            ):
                return False
            timestamps = [
                int(line)
                for line in timestamp_path.read_text(encoding="ascii").splitlines()
            ]
            if (
                len(timestamps) != entry["frame_count"]
                or any(current <= previous for previous, current
                       in zip(timestamps, timestamps[1:]))
            ):
                return False
        return True
    except (KeyError, OSError, TypeError, UnicodeError, ValueError,
            json.JSONDecodeError):
        return False


def build_cache(mcap_path, output_dir, lanes=DEFAULT_LANES, *,
                frame_bytes=FRAME_BYTES, decoder=None):
    """Build or reuse a cache while serializing transitions for one key."""
    mcap_path = Path(mcap_path)
    output_dir = Path(output_dir)
    lanes = tuple(lanes)
    if len(lanes) != 3:
        raise ValueError("exactly three video lanes are required")
    if frame_bytes <= 0:
        raise ValueError("frame_bytes must be positive")
    output_dir.mkdir(parents=True, exist_ok=True)
    decoder = decoder or _decode_h265
    while True:
        before_lock = _source_identity(mcap_path)
        key = cache_key(before_lock["sha256"], lanes)
        lock_path = output_dir / (key + ".lock")
        with lock_path.open("a+b") as lock_file:
            fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX)
            try:
                source = _source_identity(mcap_path)
                if (source["sha256"] != before_lock["sha256"]
                        or source["size"] != before_lock["size"]):
                    continue
                return _build_cache_locked(
                    mcap_path, output_dir, lanes, frame_bytes,
                    decoder, source, key,
                )
            finally:
                fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)


def _build_cache_locked(mcap_path, output_dir, lanes, frame_bytes,
                        decoder, source, key):
    final_dir = output_dir / key
    quarantine = None
    if final_dir.exists():
        if _validate_cache(final_dir, source, key, lanes, frame_bytes):
            return final_dir
        quarantine = output_dir / (
            "." + key + ".quarantine." + uuid.uuid4().hex
        )
        try:
            os.rename(final_dir, quarantine)
        except OSError:
            if (final_dir.is_dir()
                    and _validate_cache(final_dir, source, key,
                                        lanes, frame_bytes)):
                return final_dir
            raise

    staging = None
    try:
        staging = Path(
            tempfile.mkdtemp(prefix="." + key + ".", dir=output_dir)
        )
        timestamps = _extract_lanes(mcap_path, lanes, staging)
        with ThreadPoolExecutor(max_workers=3) as pool:
            futures = {
                lane.name: pool.submit(
                    decoder,
                    staging / (lane.name + ".h265"),
                    staging / (lane.name + ".uyvy"),
                    WIDTH,
                    HEIGHT,
                )
                for lane in lanes
            }
            decoded_counts = {
                name: future.result() for name, future in futures.items()
            }

        manifest = {
            "version": 1,
            "dataset_id": source["sha256"],
            "cache_key": key,
            "converter_version": CONVERTER_VERSION,
            "source_mcap": source,
            "width": WIDTH,
            "height": HEIGHT,
            "pixel_format": "uyvy422",
            "lanes": {},
        }
        for lane in lanes:
            lane_times = timestamps[lane.name]
            frame_count = decoded_counts[lane.name]
            payload = staging / (lane.name + ".uyvy")
            expected_size = frame_count * frame_bytes
            if frame_count != len(lane_times):
                raise RuntimeError(
                    "%s decoded frame count %d does not match timestamp count %d"
                    % (lane.name, frame_count, len(lane_times))
                )
            if payload.stat().st_size != expected_size:
                raise RuntimeError(
                    "%s payload size does not match frame count" % lane.name
                )
            timestamp_name = lane.name + ".timestamps_ns"
            timestamp_path = staging / timestamp_name
            _write_timestamps(timestamp_path, lane_times)
            manifest["lanes"][lane.name] = {
                "topic": lane.topic,
                "source_fps": lane.source_fps,
                "output_fps": lane.output_fps,
                "pixel_format": "uyvy422",
                "frame_count": frame_count,
                "frame_bytes": frame_bytes,
                "payload_file": lane.name + ".uyvy",
                "timestamps": {
                    "file": timestamp_name,
                    "byte_size": timestamp_path.stat().st_size,
                    "sha256": _sha256(timestamp_path),
                },
                "sha256": _sha256(payload),
            }

        for lane in lanes:
            (staging / (lane.name + ".h265")).unlink()
        manifest_path = staging / "manifest.json"
        manifest_path.write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        size_before_hash = mcap_path.stat().st_size
        current_dataset_id = _sha256(mcap_path)
        size_after_hash = mcap_path.stat().st_size
        if (size_before_hash != source["size"]
                or size_after_hash != source["size"]
                or current_dataset_id != source["sha256"]):
            raise RuntimeError("source MCAP changed while building cache")
        try:
            os.rename(staging, final_dir)
        except OSError:
            if (final_dir.is_dir()
                    and _validate_cache(final_dir, source, key,
                                        lanes, frame_bytes)):
                if quarantine is not None and quarantine.exists():
                    shutil.rmtree(quarantine)
                return final_dir
            raise
        if quarantine is not None:
            shutil.rmtree(quarantine)
        return final_dir
    except BaseException:
        if quarantine is not None and quarantine.exists():
            if (final_dir.is_dir()
                    and _validate_cache(final_dir, source, key,
                                        lanes, frame_bytes)):
                shutil.rmtree(quarantine)
            elif not final_dir.exists():
                os.rename(quarantine, final_dir)
            else:
                conflict = output_dir / (
                    "." + key + ".conflict." + uuid.uuid4().hex
                )
                os.rename(final_dir, conflict)
                try:
                    os.rename(quarantine, final_dir)
                except BaseException:
                    if not final_dir.exists() and conflict.exists():
                        os.rename(conflict, final_dir)
                    raise
                else:
                    shutil.rmtree(conflict)
        raise
    finally:
        if staging is not None and staging.exists():
            shutil.rmtree(staging)


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("mcap")
    parser.add_argument("output_dir")
    parser.add_argument(
        "--lane", nargs=4, action="append",
        metavar=("NAME", "TOPIC", "SOURCE_FPS", "OUTPUT_FPS"),
    )
    args = parser.parse_args(argv)
    lanes = DEFAULT_LANES
    if args.lane:
        lanes = tuple(
            Lane(name, topic, int(source_fps), int(output_fps))
            for name, topic, source_fps, output_fps in args.lane
        )
    cache_dir = build_cache(args.mcap, args.output_dir, lanes)
    print(cache_dir.resolve())
    return 0


if __name__ == "__main__":
    sys.exit(main())
