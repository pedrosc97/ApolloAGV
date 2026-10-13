#!/usr/bin/env python3
"""Apollo - RealSense smoke test.

Talks to the camera through librealsense directly (no ROS needed, and the
realsense2_camera node must NOT be running, or it will hold the device).

Checks, in order:
  1. A RealSense enumerates, and it negotiated USB 3 (not USB 2).
  2. Depth + colour start at the spec'd mode (640x480 @ 30 fps).
  3. Frames arrive at >= 90% of the nominal rate with no timeouts.
  4. Depth is not empty and colour is not black.

It also saves sample pictures, spread evenly over the run, into
realsense_samples/<timestamp>/ (change with --save, turn off with --samples 0):
  color_NN.png      the colour image
  depth_NN.png      depth as a colour map: blue = near, red = far, black = no data
  depth_raw_NN.png  16-bit depth in camera units (normally 1 unit = 1 mm)
  pair_NN.png       colour and depth map side by side

Usage:
  python3 realsense_smoke_test.py                 # 10 s quick check, 5 samples
  python3 realsense_smoke_test.py --seconds 600   # Phase 1 soak (10 min)
  python3 realsense_smoke_test.py --samples 12 --save out/
  python3 realsense_smoke_test.py --samples 0     # no pictures

Exit code: 0 = pass, 1 = a check failed, 2 = could not run at all.

Install:
  pip install pyrealsense2 numpy        (PNGs are written without opencv)
  If pip has no wheel for the Pi, build librealsense from source with
  -DBUILD_PYTHON_BINDINGS=ON -DFORCE_RSUSB_BACKEND=ON.
"""
import argparse
import os
import statistics
import struct
import sys
import time
import zlib

WARMUP_FRAMES = 30        # let auto-exposure settle before measuring
MAX_TIMEOUTS = 5          # give up after this many consecutive 2 s timeouts
MIN_FPS_FRACTION = 0.90   # pass threshold relative to nominal fps
MIN_DEPTH_FILL = 0.30     # warn if fewer than 30% of depth pixels are valid
MIN_COLOR_MEAN = 5.0      # warn if the colour image is essentially black


def parse_args():
    p = argparse.ArgumentParser(description="RealSense smoke test")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--seconds", type=float, default=10.0,
                   help="measurement time after warm-up (600 for the soak test)")
    p.add_argument("--samples", type=int, default=5,
                   help="number of sample pictures to save (0 = none, max 50)")
    p.add_argument("--save", metavar="DIR", default=None,
                   help="folder for the pictures "
                        "(default realsense_samples/<timestamp>)")
    p.add_argument("--allow-usb2", action="store_true",
                   help="do not fail when the camera is on a USB 2 link")
    return p.parse_args()


def info(dev, rs, key):
    return dev.get_info(key) if dev.supports(key) else "n/a"


def write_png(path, arr, np):
    """Write an HxWx3 uint8 (RGB) or HxW uint8/uint16 (grey) array as a PNG."""
    arr = np.ascontiguousarray(arr)
    h, w = arr.shape[:2]
    if arr.dtype == np.uint16:
        bit_depth, color_type = 16, 0
        body = arr.astype(">u2")          # PNG stores 16-bit samples big-endian
    else:
        bit_depth, color_type = 8, (2 if arr.ndim == 3 else 0)
        body = arr.astype(np.uint8)
    rows = body.view(np.uint8).reshape(h, -1)
    raw = np.hstack([np.zeros((h, 1), np.uint8), rows]).tobytes()  # filter byte 0

    def chunk(tag, data):
        return (struct.pack(">I", len(data)) + tag + data
                + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))

    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n"
                + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, bit_depth,
                                             color_type, 0, 0, 0))
                + chunk(b"IDAT", zlib.compress(raw, 6))
                + chunk(b"IEND", b""))


def depth_to_rgb(depth, np):
    """Colour-map raw depth: blue = near, red = far, black = no data."""
    valid = depth > 0
    out = np.zeros(depth.shape + (3,), np.uint8)
    if not valid.any():
        return out
    lo, hi = np.percentile(depth[valid], [2, 98])
    x = np.clip((depth.astype(np.float32) - lo) / max(hi - lo, 1.0), 0.0, 1.0)
    for ch, centre in enumerate((3.0, 2.0, 1.0)):        # R, G, B
        out[..., ch] = np.clip(1.5 - np.abs(4.0 * x - centre), 0.0, 1.0) * 255
    out[~valid] = 0
    return out


def save_samples(out_dir, samples, depth_scale, np):
    """samples: list of (seconds into run, BGR colour array, raw depth array)."""
    os.makedirs(out_dir, exist_ok=True)
    for i, (t, color_bgr, depth) in enumerate(samples, start=1):
        color = color_bgr[..., ::-1]                       # BGR -> RGB
        depth_vis = depth_to_rgb(depth, np)
        write_png(os.path.join(out_dir, f"color_{i:02d}.png"), color, np)
        write_png(os.path.join(out_dir, f"depth_{i:02d}.png"), depth_vis, np)
        write_png(os.path.join(out_dir, f"depth_raw_{i:02d}.png"), depth, np)
        if color.shape == depth_vis.shape:
            write_png(os.path.join(out_dir, f"pair_{i:02d}.png"),
                      np.hstack([color, depth_vis]), np)
        valid = depth[depth > 0]
        span = (f"{valid.min() * depth_scale:.2f}-{valid.max() * depth_scale:.2f} m"
                if valid.size else "no valid depth")
        print(f"       sample {i:02d} at t={t:.1f} s  depth range {span}")
    print(f"       saved {len(samples)} sample set(s) to {out_dir}/ "
          f"(raw depth: 1 unit = {depth_scale * 1000:g} mm)")


def main():
    args = parse_args()
    W, H, FPS = args.width, args.height, args.fps

    try:
        import numpy as np
        import pyrealsense2 as rs
    except ImportError as e:
        print(f"[FAIL] missing dependency: {e}")
        return 2

    results = []  # (level, text) where level is PASS / WARN / FAIL

    def record(level, text):
        results.append((level, text))
        print(f"[{level}] {text}")

    # ---- 1. enumerate -----------------------------------------------------
    devices = rs.context().query_devices()
    if len(devices) == 0:
        print("[FAIL] no RealSense device found. Check the cable, the port, and "
              "that no other process (e.g. realsense2_camera) has it open.")
        return 2
    dev = devices[0]
    name = info(dev, rs, rs.camera_info.name)
    serial = info(dev, rs, rs.camera_info.serial_number)
    firmware = info(dev, rs, rs.camera_info.firmware_version)
    usb = info(dev, rs, rs.camera_info.usb_type_descriptor)
    record("PASS", f"found {name}  SN {serial}  FW {firmware}")

    if usb.startswith("3"):
        record("PASS", f"USB link type {usb}")
    elif args.allow_usb2:
        record("WARN", f"USB link type {usb} (USB 2, allowed by flag)")
    else:
        record("FAIL", f"USB link type {usb}: expected 3.x. Use a blue USB 3 port "
                       "and a USB 3 cable; try flipping the USB-C plug.")

    # ---- 2. start streams -------------------------------------------------
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(serial)
    config.enable_stream(rs.stream.depth, W, H, rs.format.z16, FPS)
    config.enable_stream(rs.stream.color, W, H, rs.format.bgr8, FPS)
    try:
        profile = pipeline.start(config)
    except Exception as e:
        record("FAIL", f"could not start depth+colour {W}x{H}@{FPS}: {e}")
        return 1

    try:
        depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        intr = (profile.get_stream(rs.stream.color)
                .as_video_stream_profile().get_intrinsics())
        record("PASS", f"streams started: depth+colour {W}x{H}@{FPS}")
        print(f"       colour intrinsics fx={intr.fx:.1f} fy={intr.fy:.1f} "
              f"cx={intr.ppx:.1f} cy={intr.ppy:.1f}, depth scale {depth_scale} m/unit")

        # ---- warm-up ------------------------------------------------------
        for _ in range(WARMUP_FRAMES):
            pipeline.wait_for_frames(5000)

        # ---- 3. measure ---------------------------------------------------
        print(f"       measuring for {args.seconds:.0f} s ...")
        n = 0
        timeouts = consecutive_timeouts = incomplete = 0
        skipped = {"depth": 0, "color": 0}
        last_num = {"depth": None, "color": None}
        fills, centers, color_means = [], [], []
        n_samples = max(0, min(args.samples, 50))
        sample_times = [(k + 0.5) * args.seconds / n_samples
                        for k in range(n_samples)]
        samples = []   # copies only; PNGs are written after measuring
        report_every = 5.0 if args.seconds <= 60 else 60.0
        t0 = time.monotonic()
        next_report = t0 + report_every

        while True:
            now = time.monotonic()
            if now - t0 >= args.seconds:
                break
            try:
                frames = pipeline.wait_for_frames(2000)
                consecutive_timeouts = 0
            except RuntimeError:
                timeouts += 1
                consecutive_timeouts += 1
                print(f"       timeout waiting for frames at t={now - t0:.1f} s")
                if consecutive_timeouts >= MAX_TIMEOUTS:
                    print("       too many consecutive timeouts, stopping early")
                    break
                continue

            d = frames.get_depth_frame()
            c = frames.get_color_frame()
            if not d or not c:
                incomplete += 1
                continue
            n += 1

            if len(samples) < n_samples and now - t0 >= sample_times[len(samples)]:
                samples.append((now - t0,
                                np.asanyarray(c.get_data()).copy(),
                                np.asanyarray(d.get_data()).copy()))

            for key, f in (("depth", d), ("color", c)):
                num = f.get_frame_number()
                if last_num[key] is not None and num > last_num[key] + 1:
                    skipped[key] += num - last_num[key] - 1
                last_num[key] = num

            if n % FPS == 1:  # sample image content about once a second
                depth = np.asanyarray(d.get_data())
                fills.append(float((depth > 0).mean()))
                patch = depth[H // 2 - 10:H // 2 + 10, W // 2 - 10:W // 2 + 10]
                valid = patch[patch > 0]
                if valid.size:
                    centers.append(float(np.median(valid)) * depth_scale)
                color_means.append(float(np.asanyarray(c.get_data()).mean()))

            if now >= next_report:
                print(f"       t={now - t0:5.0f} s  {n / (now - t0):5.1f} fps  "
                      f"timeouts={timeouts}  skipped d/c="
                      f"{skipped['depth']}/{skipped['color']}")
                next_report += report_every

        elapsed = time.monotonic() - t0
        fps = n / elapsed if elapsed > 0 else 0.0

        # ---- verdicts -----------------------------------------------------
        level = "PASS" if fps >= MIN_FPS_FRACTION * FPS else "FAIL"
        record(level, f"frame rate {fps:.1f} fps over {elapsed:.0f} s "
                      f"({n} framesets, need >= {MIN_FPS_FRACTION * FPS:.0f})")

        record("PASS" if timeouts == 0 else "FAIL", f"timeouts: {timeouts}")

        total_skipped = skipped["depth"] + skipped["color"]
        level = "PASS" if total_skipped <= 0.01 * max(n, 1) else "WARN"
        record(level, f"skipped frames: depth {skipped['depth']}, colour "
                      f"{skipped['color']}, incomplete framesets {incomplete}")

        if fills:
            fill = statistics.median(fills)
            level = "PASS" if fill >= MIN_DEPTH_FILL else "WARN"
            record(level, f"depth fill {fill * 100:.0f}% valid pixels "
                          "(point it at a wall 1-3 m away for a fair reading)")
        else:
            record("FAIL", "no depth frames analysed")

        if centers:
            record("PASS", f"range at image centre {statistics.median(centers):.2f} m "
                           "(compare with a tape measure)")
        else:
            record("WARN", "no valid depth at image centre (too close, too far, "
                           "or a shiny/black surface)")

        if color_means:
            mean = statistics.median(color_means)
            level = "PASS" if mean >= MIN_COLOR_MEAN else "WARN"
            record(level, f"colour mean brightness {mean:.0f}/255")

    finally:
        pipeline.stop()

    if samples:
        out_dir = args.save or os.path.join("realsense_samples",
                                            time.strftime("%Y%m%d-%H%M%S"))
        try:
            save_samples(out_dir, samples, depth_scale, np)
        except OSError as e:
            record("WARN", f"could not save sample pictures: {e}")
    elif n_samples:
        record("WARN", "no sample pictures saved (no complete frames arrived)")

    failed = [t for lvl, t in results if lvl == "FAIL"]
    warned = [t for lvl, t in results if lvl == "WARN"]
    print()
    if failed:
        print(f"RESULT: FAIL ({len(failed)} failed, {len(warned)} warnings)")
        return 1
    print(f"RESULT: PASS ({len(warned)} warnings)")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\ninterrupted")
        sys.exit(130)
