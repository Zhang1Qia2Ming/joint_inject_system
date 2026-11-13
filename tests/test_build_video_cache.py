import hashlib
import json
import multiprocessing
import os
import shutil
import threading
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest import mock

from tools import build_video_cache


def annex_b(nal_type, body=b"x"):
    return b"\x00\x00\x00\x01" + bytes([nal_type << 1, 1]) + body


class FakeReader:
    def __init__(self, messages):
        self.messages = messages

    def iter_messages(self, topics=None):
        selected = set(topics or ())
        for topic, timestamp, payload in self.messages:
            if topic in selected:
                yield None, SimpleNamespace(topic=topic), SimpleNamespace(
                    log_time=timestamp, data=payload
                )


def concurrent_build_worker(mcap, output, lanes, role, first_decoding,
                            release_first, second_entered,
                            second_decoding, results):
    messages = []
    for lane in lanes:
        messages.extend(
            [
                (lane.topic, 50, annex_b(1, b"before-vps")),
                (lane.topic, 100, annex_b(32, b"vps")),
                (lane.topic, 200, annex_b(1, b"frame")),
            ]
        )
    build_video_cache.make_reader = lambda _stream: FakeReader(messages)
    signaled = False
    signal_lock = threading.Lock()

    def decode(_input_path, output_path, _width, _height):
        nonlocal signaled
        if role == "first":
            with signal_lock:
                should_wait = not signaled
                signaled = True
            if should_wait:
                first_decoding.set()
                if not release_first.wait(timeout=5):
                    raise RuntimeError("first builder release timed out")
        else:
            second_decoding.set()
        output_path.write_bytes(b"AB")
        return 2

    if role == "second":
        second_entered.set()
    try:
        cache_dir = build_video_cache.build_cache(
            mcap, output, lanes, frame_bytes=1, decoder=decode
        )
        results.put((role, "ok", str(cache_dir)))
    except BaseException as error:
        results.put((role, "error", repr(error)))
        raise


class VideoCacheBuilderTest(unittest.TestCase):
    def setUp(self):
        self.temp = TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.mcap = self.root / "video.mcap"
        self.mcap.write_bytes(b"fixture-mcap")
        self.output = self.root / "cache"
        self.lanes = (
            build_video_cache.Lane("tx1", "/tx1", 10, 30),
            build_video_cache.Lane("tx2", "/tx2", 30, 30),
            build_video_cache.Lane("tx4", "/tx4", 10, 30),
        )

    def tearDown(self):
        self.temp.cleanup()

    def messages(self):
        result = []
        for lane in self.lanes:
            result.extend(
                [
                    (lane.topic, 50, annex_b(1, b"before-vps")),
                    (lane.topic, 100, annex_b(32, b"vps")),
                    (lane.topic, 200, annex_b(1, b"frame")),
                ]
            )
        return result

    def reader(self, _stream):
        return FakeReader(self.messages())

    def test_manifest_preserves_post_vps_log_times_and_hash(self):
        frame_bytes = 8

        def decode(_input_path, output_path, _width, _height):
            output_path.write_bytes(b"A" * (2 * frame_bytes))
            return 2

        with mock.patch.object(build_video_cache, "make_reader", self.reader):
            cache_dir = build_video_cache.build_cache(
                self.mcap,
                self.output,
                self.lanes,
                frame_bytes=frame_bytes,
                decoder=decode,
            )

        manifest = json.loads(
            (cache_dir / "manifest.json").read_text(encoding="utf-8")
        )
        source_stat = self.mcap.stat()
        self.assertEqual(1, manifest["version"])
        self.assertEqual(
            hashlib.sha256(self.mcap.read_bytes()).hexdigest(),
            manifest["dataset_id"],
        )
        self.assertEqual(cache_dir.name, manifest["cache_key"])
        self.assertEqual(source_stat.st_size, manifest["source_mcap"]["size"])
        self.assertEqual(source_stat.st_mtime_ns, manifest["source_mcap"]["mtime_ns"])
        self.assertEqual(build_video_cache.CONVERTER_VERSION,
                         manifest["converter_version"])
        for lane in self.lanes:
            entry = manifest["lanes"][lane.name]
            self.assertEqual(lane.topic, entry["topic"])
            self.assertEqual(lane.source_fps, entry["source_fps"])
            self.assertEqual(lane.output_fps, entry["output_fps"])
            self.assertEqual("uyvy422", entry["pixel_format"])
            self.assertEqual(2, entry["frame_count"])
            self.assertEqual(frame_bytes, entry["frame_bytes"])
            timestamp_meta = entry["timestamps"]
            timestamp_path = cache_dir / timestamp_meta["file"]
            timestamps = (
                timestamp_path.read_text(encoding="ascii").splitlines()
            )
            self.assertEqual(["100", "200"], timestamps)
            self.assertEqual(timestamp_path.stat().st_size,
                             timestamp_meta["byte_size"])
            self.assertEqual(
                hashlib.sha256(timestamp_path.read_bytes()).hexdigest(),
                timestamp_meta["sha256"],
            )
            payload = cache_dir / entry["payload_file"]
            self.assertEqual(
                hashlib.sha256(payload.read_bytes()).hexdigest(), entry["sha256"]
            )

    def test_three_lanes_decode_in_parallel(self):
        lock = threading.Lock()
        release = threading.Event()
        active = 0
        peak = 0

        def decode(_input_path, output_path, _width, _height):
            nonlocal active, peak
            with lock:
                active += 1
                peak = max(peak, active)
                if peak == 3:
                    release.set()
            self.assertTrue(release.wait(timeout=2))
            output_path.write_bytes(b"AB")
            with lock:
                active -= 1
            return 2

        with mock.patch.object(build_video_cache, "make_reader", self.reader):
            build_video_cache.build_cache(
                self.mcap,
                self.output,
                self.lanes,
                frame_bytes=1,
                decoder=decode,
            )
        self.assertEqual(3, peak)

    def test_frame_timestamp_mismatch_leaves_no_final_cache(self):
        def decode(_input_path, output_path, _width, _height):
            output_path.write_bytes(b"A")
            return 1

        with mock.patch.object(build_video_cache, "make_reader", self.reader):
            with self.assertRaisesRegex(RuntimeError, "timestamp"):
                build_video_cache.build_cache(
                    self.mcap,
                    self.output,
                    self.lanes,
                    frame_bytes=1,
                    decoder=decode,
                )

        self.assertEqual([], [path for path in self.output.iterdir()
                              if path.is_dir()
                              and not path.name.startswith(".")])

    def test_cache_key_covers_dataset_topics_format_and_converter(self):
        dataset = hashlib.sha256(b"dataset").hexdigest()
        base = build_video_cache.cache_key(
            dataset,
            self.lanes,
            width=3840,
            height=2160,
            pixel_format="uyvy422",
            converter_version="v1",
        )
        changes = [
            (hashlib.sha256(b"other").hexdigest(), self.lanes,
             3840, 2160, "uyvy422", "v1"),
            (dataset,
             (build_video_cache.Lane("tx1", "/changed", 10, 30),)
             + self.lanes[1:],
             3840, 2160, "uyvy422", "v1"),
            (dataset, self.lanes, 1920, 2160, "uyvy422", "v1"),
            (dataset, self.lanes, 3840, 1080, "uyvy422", "v1"),
            (dataset, self.lanes, 3840, 2160, "nv12", "v1"),
            (dataset, self.lanes, 3840, 2160, "uyvy422", "v2"),
        ]
        for arguments in changes:
            self.assertNotEqual(base, build_video_cache.cache_key(*arguments))

    def test_valid_cache_hit_returns_key_directory_without_decoding(self):
        def decode(_input_path, output_path, _width, _height):
            output_path.write_bytes(b"AB")
            return 2

        with mock.patch.object(build_video_cache, "make_reader", self.reader):
            first = build_video_cache.build_cache(
                self.mcap, self.output, self.lanes,
                frame_bytes=1, decoder=decode,
            )
        with mock.patch.object(
            build_video_cache, "make_reader",
            side_effect=AssertionError("cache hit must not read MCAP"),
        ):
            with mock.patch.object(
                build_video_cache.os, "rename",
                side_effect=AssertionError("cache hit must not publish"),
            ):
                second = build_video_cache.build_cache(
                    self.mcap, self.output, self.lanes,
                    frame_bytes=1,
                    decoder=lambda *_args: self.fail("cache hit decoded"),
                )
        self.assertEqual(first, second)

    def test_timestamp_tampering_rebuilds_same_key_cache(self):
        def decode(_input_path, output_path, _width, _height):
            output_path.write_bytes(b"AB")
            return 2

        with mock.patch.object(build_video_cache, "make_reader", self.reader):
            cache_dir = build_video_cache.build_cache(
                self.mcap, self.output, self.lanes,
                frame_bytes=1, decoder=decode,
            )
        timestamp_path = cache_dir / "tx1.timestamps_ns"
        timestamp_path.write_text("100\n201\n", encoding="ascii")
        with mock.patch.object(build_video_cache, "make_reader", self.reader):
            rebuilt = build_video_cache.build_cache(
                self.mcap, self.output, self.lanes,
                frame_bytes=1, decoder=decode,
            )
        self.assertEqual(cache_dir, rebuilt)
        self.assertEqual("100\n200\n", timestamp_path.read_text(encoding="ascii"))
        self.assertEqual([], list(self.output.glob("*.quarantine.*")))

    def test_mtime_only_change_keeps_valid_cache_hit(self):
        def decode(_input_path, output_path, _width, _height):
            output_path.write_bytes(b"AB")
            return 2

        with mock.patch.object(build_video_cache, "make_reader", self.reader):
            cache_dir = build_video_cache.build_cache(
                self.mcap, self.output, self.lanes,
                frame_bytes=1, decoder=decode,
            )
        stat = self.mcap.stat()
        os.utime(self.mcap, ns=(stat.st_atime_ns, stat.st_mtime_ns + 1_000_000))
        with mock.patch.object(
            build_video_cache, "make_reader",
            side_effect=AssertionError("mtime-only hit must not rebuild"),
        ):
            hit = build_video_cache.build_cache(
                self.mcap, self.output, self.lanes,
                frame_bytes=1, decoder=lambda *_args: self.fail("decoded"),
            )
        self.assertEqual(cache_dir, hit)

    def test_failed_rebuild_restores_corrupt_same_key_directory(self):
        def decode(_input_path, output_path, _width, _height):
            output_path.write_bytes(b"AB")
            return 2

        with mock.patch.object(build_video_cache, "make_reader", self.reader):
            cache_dir = build_video_cache.build_cache(
                self.mcap, self.output, self.lanes,
                frame_bytes=1, decoder=decode,
            )
        payload = cache_dir / "tx1.uyvy"
        payload.write_bytes(b"XX")
        with mock.patch.object(build_video_cache, "make_reader", self.reader):
            with self.assertRaisesRegex(RuntimeError, "decode failed"):
                build_video_cache.build_cache(
                    self.mcap, self.output, self.lanes,
                    frame_bytes=1,
                    decoder=lambda *_args: (_ for _ in ()).throw(
                        RuntimeError("decode failed")
                    ),
                )
        self.assertTrue(cache_dir.is_dir())
        self.assertEqual(b"XX", payload.read_bytes())
        self.assertEqual([], list(self.output.glob("*.quarantine.*")))

    def test_failed_rebuild_discards_quarantine_when_valid_final_appears(self):
        def decode(_input_path, output_path, _width, _height):
            output_path.write_bytes(b"AB")
            return 2

        with mock.patch.object(build_video_cache, "make_reader", self.reader):
            cache_dir = build_video_cache.build_cache(
                self.mcap, self.output, self.lanes,
                frame_bytes=1, decoder=decode,
            )
        valid_template = self.root / "valid-template"
        shutil.copytree(cache_dir, valid_template)
        (cache_dir / "tx1.uyvy").write_bytes(b"XX")
        installed = False
        install_lock = threading.Lock()

        def fail_after_external_winner(_input_path, _output_path,
                                       _width, _height):
            nonlocal installed
            with install_lock:
                if not installed:
                    shutil.copytree(valid_template, cache_dir)
                    installed = True
            raise RuntimeError("build failed")

        with mock.patch.object(build_video_cache, "make_reader", self.reader):
            with self.assertRaisesRegex(RuntimeError, "build failed"):
                build_video_cache.build_cache(
                    self.mcap, self.output, self.lanes,
                    frame_bytes=1, decoder=fail_after_external_winner,
                )
        self.assertEqual(b"AB", (cache_dir / "tx1.uyvy").read_bytes())
        self.assertEqual([], list(self.output.glob("*.quarantine.*")))

    def test_publish_failure_restores_quarantined_same_key_directory(self):
        def decode(_input_path, output_path, _width, _height):
            output_path.write_bytes(b"AB")
            return 2

        with mock.patch.object(build_video_cache, "make_reader", self.reader):
            cache_dir = build_video_cache.build_cache(
                self.mcap, self.output, self.lanes,
                frame_bytes=1, decoder=decode,
            )
        payload = cache_dir / "tx1.uyvy"
        payload.write_bytes(b"XX")
        real_rename = build_video_cache.os.rename
        calls = 0

        def fail_publish(source, destination):
            nonlocal calls
            calls += 1
            if calls == 2:
                raise OSError("publish failed")
            return real_rename(source, destination)

        with mock.patch.object(build_video_cache, "make_reader", self.reader):
            with mock.patch.object(
                build_video_cache.os, "rename", side_effect=fail_publish
            ):
                with self.assertRaisesRegex(OSError, "publish failed"):
                    build_video_cache.build_cache(
                        self.mcap, self.output, self.lanes,
                        frame_bytes=1, decoder=decode,
                    )
        self.assertTrue(cache_dir.is_dir())
        self.assertEqual(b"XX", payload.read_bytes())
        self.assertEqual([], list(self.output.glob("*.quarantine.*")))

    def test_staging_creation_failure_restores_quarantined_directory(self):
        def decode(_input_path, output_path, _width, _height):
            output_path.write_bytes(b"AB")
            return 2

        with mock.patch.object(build_video_cache, "make_reader", self.reader):
            cache_dir = build_video_cache.build_cache(
                self.mcap, self.output, self.lanes,
                frame_bytes=1, decoder=decode,
            )
        payload = cache_dir / "tx1.uyvy"
        payload.write_bytes(b"XX")
        with mock.patch.object(
            build_video_cache.tempfile, "mkdtemp",
            side_effect=OSError("no staging"),
        ):
            with self.assertRaisesRegex(OSError, "no staging"):
                build_video_cache.build_cache(
                    self.mcap, self.output, self.lanes,
                    frame_bytes=1, decoder=decode,
                )
        self.assertTrue(cache_dir.is_dir())
        self.assertEqual(b"XX", payload.read_bytes())
        self.assertEqual([], list(self.output.glob("*.quarantine.*")))

    def test_final_directory_contains_only_manifest_declared_products(self):
        def decode(_input_path, output_path, _width, _height):
            output_path.write_bytes(b"AB")
            return 2

        with mock.patch.object(build_video_cache, "make_reader", self.reader):
            cache_dir = build_video_cache.build_cache(
                self.mcap, self.output, self.lanes,
                frame_bytes=1, decoder=decode,
            )
        manifest = json.loads(
            (cache_dir / "manifest.json").read_text(encoding="utf-8")
        )
        declared = {"manifest.json"}
        for entry in manifest["lanes"].values():
            declared.add(entry["payload_file"])
            declared.add(entry["timestamps"]["file"])
        self.assertEqual(declared, {path.name for path in cache_dir.iterdir()})

    def test_equal_size_mtime_restored_source_change_rejects_publish(self):
        original_stat = self.mcap.stat()
        changed = False
        lock = threading.Lock()

        def decode(_input_path, output_path, _width, _height):
            nonlocal changed
            with lock:
                if not changed:
                    self.mcap.write_bytes(b"changed-mcap")
                    os.utime(
                        self.mcap,
                        ns=(original_stat.st_atime_ns, original_stat.st_mtime_ns),
                    )
                    changed = True
            output_path.write_bytes(b"AB")
            return 2

        with mock.patch.object(build_video_cache, "make_reader", self.reader):
            with self.assertRaisesRegex(RuntimeError, "source MCAP changed"):
                build_video_cache.build_cache(
                    self.mcap, self.output, self.lanes,
                    frame_bytes=1, decoder=decode,
                )
        self.assertEqual([], [path for path in self.output.iterdir()
                              if path.is_dir()
                              and not path.name.startswith(".")])

    def test_concurrent_same_key_publish_validates_and_reuses_winner(self):
        def decode(_input_path, output_path, _width, _height):
            output_path.write_bytes(b"AB")
            return 2

        def publish_winner(source, destination):
            shutil.copytree(source, destination)
            raise FileExistsError("concurrent publisher won")

        with mock.patch.object(build_video_cache, "make_reader", self.reader):
            with mock.patch.object(
                build_video_cache.os, "rename", side_effect=publish_winner
            ):
                cache_dir = build_video_cache.build_cache(
                    self.mcap, self.output, self.lanes,
                    frame_bytes=1, decoder=decode,
                )
        self.assertTrue((cache_dir / "manifest.json").is_file())

    def test_multiprocess_same_key_build_serializes_and_late_builder_hits(self):
        def decode(_input_path, output_path, _width, _height):
            output_path.write_bytes(b"AB")
            return 2

        with mock.patch.object(build_video_cache, "make_reader", self.reader):
            cache_dir = build_video_cache.build_cache(
                self.mcap, self.output, self.lanes,
                frame_bytes=1, decoder=decode,
            )
        (cache_dir / "tx1.uyvy").write_bytes(b"XX")

        context = multiprocessing.get_context("fork")
        first_decoding = context.Event()
        release_first = context.Event()
        second_entered = context.Event()
        second_decoding = context.Event()
        results = context.Queue()
        arguments = (
            self.mcap, self.output, self.lanes,
            first_decoding, release_first, second_entered,
            second_decoding, results,
        )
        first = context.Process(
            target=concurrent_build_worker,
            args=(arguments[0], arguments[1], arguments[2], "first",
                  *arguments[3:]),
        )
        second = context.Process(
            target=concurrent_build_worker,
            args=(arguments[0], arguments[1], arguments[2], "second",
                  *arguments[3:]),
        )
        first.start()
        try:
            self.assertTrue(first_decoding.wait(timeout=3))
            second.start()
            self.assertTrue(second_entered.wait(timeout=3))
            self.assertFalse(
                second_decoding.wait(timeout=0.5),
                "late same-key builder entered decode instead of waiting",
            )
        finally:
            release_first.set()
            first.join(timeout=5)
            if second.pid is not None:
                second.join(timeout=5)
            if first.is_alive():
                first.kill()
            if second.pid is not None and second.is_alive():
                second.kill()

        self.assertEqual(0, first.exitcode)
        self.assertEqual(0, second.exitcode)
        outcomes = sorted([results.get(timeout=1), results.get(timeout=1)])
        self.assertEqual(["ok", "ok"], [item[1] for item in outcomes])
        self.assertEqual(outcomes[0][2], outcomes[1][2])
        self.assertEqual([], list(self.output.glob("*.quarantine.*")))
        self.assertTrue((cache_dir / "manifest.json").is_file())

    def test_publish_interruption_does_not_pollute_old_cache(self):
        def decode(_input_path, output_path, _width, _height):
            output_path.write_bytes(b"AB")
            return 2

        with mock.patch.object(build_video_cache, "make_reader", self.reader):
            old_cache = build_video_cache.build_cache(
                self.mcap, self.output, self.lanes,
                frame_bytes=1, decoder=decode,
            )
        old_manifest = (old_cache / "manifest.json").read_bytes()
        self.mcap.write_bytes(b"changed-source")
        with mock.patch.object(build_video_cache, "make_reader", self.reader):
            with mock.patch.object(
                build_video_cache.os, "rename",
                side_effect=OSError("interrupted publish"),
            ):
                with self.assertRaisesRegex(OSError, "interrupted"):
                    build_video_cache.build_cache(
                        self.mcap, self.output, self.lanes,
                        frame_bytes=1, decoder=decode,
                    )
        self.assertEqual(old_manifest, (old_cache / "manifest.json").read_bytes())
        visible = [path for path in self.output.iterdir()
                   if path.is_dir() and not path.name.startswith(".")]
        self.assertEqual([old_cache], visible)

    def test_ffprobe_rejects_non_4k_before_ffmpeg(self):
        completed = SimpleNamespace(
            returncode=0,
            stdout=json.dumps(
                {"streams": [{"width": 1920, "height": 1080}]}
            ).encode(),
            stderr=b"",
        )
        with mock.patch.object(
            build_video_cache.subprocess, "run", return_value=completed
        ) as run:
            with self.assertRaisesRegex(RuntimeError, "3840x2160"):
                build_video_cache._decode_h265(
                    self.root / "lane.h265",
                    self.root / "lane.uyvy",
                    3840,
                    2160,
                )
        self.assertEqual(1, run.call_count)
        self.assertEqual("ffprobe", run.call_args.args[0][0])


if __name__ == "__main__":
    unittest.main()
