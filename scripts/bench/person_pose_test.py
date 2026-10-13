#!/usr/bin/env python3
"""Apollo - person detection + relative position prototype (no ROS).

Pipeline (the spec's Perception steps 1-5, without the Kalman filter yet):
  1. Capture   RealSense colour + depth, depth aligned to the colour image.
  2. Detect    nano YOLO person detector on the colour image (NCNN on the CPU).
  3. Select    closest person at start, then keep that person frame to frame.
  4. Range     median depth of a patch on the upper torso of the box.
  5. Project   back-project the patch centre through the colour intrinsics.

Output per frame is the target's POSITION relative to the camera. A bounding
box gives no orientation, so there is no rotation part to the pose.

Two frames are printed:
  optical      x right, y down, z forward      (librealsense / OpenCV)
  camera_link  x forward, y left, z up         (ROS body convention)
  range   = horizontal distance, sqrt(x^2 + y^2) in camera_link, metres
  bearing = atan2(y, x) in camera_link, degrees, positive = person to the LEFT

Usage:
  python3 person_pose_test.py                     # 30 s live run
  python3 person_pose_test.py --seconds 0         # until Ctrl-C
  python3 person_pose_test.py --record rec/       # also save raw frames
  python3 person_pose_test.py --replay rec/       # re-run offline (no camera,
                                                  #   works on the Mac too)
  python3 person_pose_test.py --imgsz 416         # faster, shorter reach
  python3 person_pose_test.py --log run.csv       # one row per frame

Annotated pictures go to person_samples/<timestamp>/ (--save, --samples 0 = off).

Install (in the Python 3.12 environment):
  uv pip install ultralytics ncnn pyrealsense2 numpy
  First run downloads yolo11n.pt and converts it to NCNN once (needs internet).
  "libGL.so.1" error on import: sudo apt install libgl1 libglib2.0-0
"""
import argparse
import csv
import glob
import json
import math
import os
import statistics
import sys
import time

PERSON_CLASS = 0
DEPTH_MIN_M, DEPTH_MAX_M = 0.3, 10.0   # ignore depth outside this band
TORSO_X = (0.35, 0.65)                 # patch: middle 30% of the box width
TORSO_Y = (0.20, 0.45)                 #        20-45% down from the box top
MIN_VALID_FRACTION = 0.2               # patch needs this much valid depth
TARGET_FPS = 10.0                      # spec: detector at 10 fps or better


def parse_args():
    p = argparse.ArgumentParser(description="Person detection + relative position")
    p.add_argument("--model", default="yolo11n.pt",
                   help="YOLO weights (.pt) or an exported *_ncnn_model folder")
    p.add_argument("--backend", choices=("ncnn", "torch"), default="ncnn")
    p.add_argument("--imgsz", type=int, default=640, help="detector input size")
    p.add_argument("--conf", type=float, default=0.5, help="min detection confidence")
    p.add_argument("--seconds", type=float, default=30.0, help="0 = until Ctrl-C")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--gate", type=float, default=1.0,
                   help="metres a locked target may jump between detections")
    p.add_argument("--lost", type=float, default=5.0,
                   help="seconds without the target before re-selecting the closest")
    p.add_argument("--samples", type=int, default=10,
                   help="annotated pictures to save, one per second (0 = none)")
    p.add_argument("--save", metavar="DIR", default=None,
                   help="folder for annotated pictures")
    p.add_argument("--record", metavar="DIR", default=None,
                   help="save raw colour + aligned depth frames for --replay")
    p.add_argument("--record-max", type=int, default=300)
    p.add_argument("--replay", metavar="DIR", default=None,
                   help="read frames from a --record folder instead of the camera")
    p.add_argument("--log", metavar="CSV", default=None)
    p.add_argument("--show", action="store_true", help="live window (needs a display)")
    return p.parse_args()


# ---------------------------------------------------------------- sources ---

class RealSenseSource:
    """Live camera. Depth is aligned to colour, so pixel (u, v) matches in both."""

    def __init__(self, width, height, fps):
        import pyrealsense2 as rs
        self.rs = rs
        self.pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        profile = self.pipeline.start(cfg)
        self.align = rs.align(rs.stream.color)
        self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        i = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.intr = {"fx": i.fx, "fy": i.fy, "cx": i.ppx, "cy": i.ppy,
                     "width": i.width, "height": i.height}
        self.t0 = time.monotonic()
        for _ in range(15):                       # let auto-exposure settle
            self.pipeline.wait_for_frames(5000)

    def read(self):
        import numpy as np
        frames = self.align.process(self.pipeline.wait_for_frames(2000))
        d, c = frames.get_depth_frame(), frames.get_color_frame()
        if not d or not c:
            return None
        return (time.monotonic() - self.t0,
                np.asanyarray(c.get_data()).copy(),
                np.asanyarray(d.get_data()).copy())

    def close(self):
        self.pipeline.stop()


class ReplaySource:
    """Frames saved by --record: color_NNNN.png, depth_NNNN.png, meta.json."""

    def __init__(self, folder):
        with open(os.path.join(folder, "meta.json")) as f:
            meta = json.load(f)
        self.intr, self.depth_scale = meta["intrinsics"], meta["depth_scale"]
        self.times = meta["times"]
        self.colors = sorted(glob.glob(os.path.join(folder, "color_*.png")))
        self.depths = sorted(glob.glob(os.path.join(folder, "depth_*.png")))
        if not self.colors or len(self.colors) != len(self.depths):
            raise RuntimeError(f"{folder}: colour/depth frame count mismatch or empty")
        self.i = 0

    def read(self):
        import cv2
        if self.i >= len(self.colors):
            raise EOFError
        color = cv2.imread(self.colors[self.i], cv2.IMREAD_COLOR)
        depth = cv2.imread(self.depths[self.i], cv2.IMREAD_UNCHANGED)
        t = self.times[self.i] if self.i < len(self.times) else self.i / 30.0
        self.i += 1
        return t, color, depth

    def close(self):
        pass


class Recorder:
    def __init__(self, folder, source, limit):
        os.makedirs(folder, exist_ok=True)
        self.folder, self.source, self.limit, self.times = folder, source, limit, []

    def add(self, t, color, depth):
        import cv2
        n = len(self.times)
        if n >= self.limit:
            return
        cv2.imwrite(os.path.join(self.folder, f"color_{n:04d}.png"), color)
        cv2.imwrite(os.path.join(self.folder, f"depth_{n:04d}.png"), depth)
        self.times.append(round(t, 4))

    def close(self):
        with open(os.path.join(self.folder, "meta.json"), "w") as f:
            json.dump({"intrinsics": self.source.intr,
                       "depth_scale": self.source.depth_scale,
                       "times": self.times}, f, indent=1)
        print(f"recorded {len(self.times)} frames to {self.folder}/")


# --------------------------------------------------------------- detector ---

def load_detector(args):
    """Returns (model, description). Converts .pt weights to NCNN on first use."""
    from ultralytics import YOLO
    path = args.model
    if args.backend == "ncnn" and path.endswith(".pt"):
        ncnn_dir = f"{path[:-3]}_{args.imgsz}_ncnn_model"
        if not os.path.isdir(ncnn_dir):
            print(f"converting {path} to NCNN at {args.imgsz} px (one-off) ...")
            try:
                exported = YOLO(path).export(format="ncnn", imgsz=args.imgsz)
                os.rename(exported, ncnn_dir)
            except Exception as e:
                print(f"[WARN] NCNN conversion failed ({e}); using PyTorch, "
                      "which is slower on the Pi")
                ncnn_dir = None
        if ncnn_dir:
            path = ncnn_dir
    return YOLO(path, task="detect"), path


def detect_people(model, color_bgr, args):
    """Returns a list of (x1, y1, x2, y2, confidence) in pixels."""
    r = model.predict(color_bgr, classes=[PERSON_CLASS], conf=args.conf,
                      imgsz=args.imgsz, verbose=False)[0]
    out = []
    for xyxy, conf, cls in zip(r.boxes.xyxy.tolist(), r.boxes.conf.tolist(),
                               r.boxes.cls.tolist()):
        if int(cls) == PERSON_CLASS:
            out.append((*xyxy, conf))
    return out


# ------------------------------------------------------- range + project ---

def locate(box, depth_m, intr):
    """Box + depth image (metres) -> position dict, or None if depth is unusable."""
    import numpy as np
    x1, y1, x2, y2 = box[:4]
    w, h = x2 - x1, y2 - y1
    H, W = depth_m.shape
    px1 = int(max(0, min(W - 1, x1 + TORSO_X[0] * w)))
    px2 = int(max(px1 + 1, min(W, x1 + TORSO_X[1] * w)))
    py1 = int(max(0, min(H - 1, y1 + TORSO_Y[0] * h)))
    py2 = int(max(py1 + 1, min(H, y1 + TORSO_Y[1] * h)))
    patch = depth_m[py1:py2, px1:px2]
    valid = patch[(patch > DEPTH_MIN_M) & (patch < DEPTH_MAX_M)]
    if valid.size < max(10, MIN_VALID_FRACTION * patch.size):
        return None
    z = float(np.median(valid))                     # distance along the optical axis
    u, v = (px1 + px2) / 2.0, (py1 + py2) / 2.0
    x = (u - intr["cx"]) / intr["fx"] * z           # pinhole back-projection
    y = (v - intr["cy"]) / intr["fy"] * z
    link = (z, -x, -y)                              # optical -> camera_link
    return {"optical": (x, y, z), "link": link,
            "range": math.hypot(link[0], link[1]),
            "bearing": math.degrees(math.atan2(link[1], link[0])),
            "patch": (px1, py1, px2, py2)}


# ---------------------------------------------------------------- select ---

class TargetSelector:
    """Closest person at start; then the detection nearest the last position."""
    MAX_SPEED = 2.0   # m/s, widens the gate while the target is unseen

    def __init__(self, gate, lost):
        self.gate, self.lost = gate, lost
        self.last, self.last_t = None, None

    def update(self, people, t):
        """people: list of position dicts. Returns the index of the target or None."""
        if not people:
            return None
        if self.last is None or t - self.last_t > self.lost:
            idx = min(range(len(people)), key=lambda i: people[i]["range"])
        else:
            dists = [math.dist(p["link"], self.last) for p in people]
            idx = min(range(len(people)), key=lambda i: dists[i])
            if dists[idx] > self.gate + self.MAX_SPEED * (t - self.last_t):
                return None                         # someone else; stay locked
        self.last, self.last_t = people[idx]["link"], t
        return idx


# ---------------------------------------------------------------- output ---

def annotate(color, boxes, people, target_i):
    import cv2
    img = color.copy()
    for i, (box, p) in enumerate(zip(boxes, people)):
        is_target = p is not None and i == target_i
        col = (0, 220, 0) if is_target else (160, 160, 160)
        x1, y1, x2, y2 = (int(v) for v in box[:4])
        cv2.rectangle(img, (x1, y1), (x2, y2), col, 2)
        if p is None:
            label = "no depth"
        else:
            a, b, c, d = p["patch"]
            cv2.rectangle(img, (a, b), (c, d), col, 1)
            label = f"{p['range']:.2f} m  {p['bearing']:+.1f} deg"
        cv2.putText(img, label, (x1, max(14, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, col, 2, cv2.LINE_AA)
    return img


def main():
    args = parse_args()
    try:
        import cv2
        import numpy as np
    except ImportError as e:
        print(f"[FAIL] missing dependency: {e}")
        return 2

    try:
        model, model_desc = load_detector(args)
    except Exception as e:
        print(f"[FAIL] could not load the detector: {e}")
        return 2

    try:
        source = (ReplaySource(args.replay) if args.replay
                  else RealSenseSource(args.width, args.height, args.fps))
    except Exception as e:
        print(f"[FAIL] could not open the frame source: {e}")
        return 2
    intr = source.intr
    print(f"detector {model_desc} at {args.imgsz} px, conf >= {args.conf}")
    print(f"intrinsics fx={intr['fx']:.1f} fy={intr['fy']:.1f} "
          f"cx={intr['cx']:.1f} cy={intr['cy']:.1f}")
    detect_people(model, np.zeros((intr["height"], intr["width"], 3), np.uint8), args)

    recorder = Recorder(args.record, source, args.record_max) if args.record else None
    selector = TargetSelector(args.gate, args.lost)
    save_dir = args.save or os.path.join("person_samples", time.strftime("%Y%m%d-%H%M%S"))
    log_file = open(args.log, "w", newline="") if args.log else None
    log = csv.writer(log_file) if log_file else None
    if log:
        log.writerow(["t", "people", "target", "x_fwd", "y_left", "z_up",
                      "range_m", "bearing_deg", "detect_ms"])

    frames = seen = saved = 0
    detect_ms, ranges, bearings = [], [], []
    next_print = next_save = 0.0
    wall0 = time.monotonic()
    try:
        while True:
            wall = time.monotonic() - wall0
            if args.seconds > 0 and not args.replay and wall >= args.seconds:
                break
            try:
                item = source.read()
            except EOFError:
                break
            except RuntimeError as e:
                print(f"[WARN] frame read failed: {e}")
                continue
            if item is None:
                continue
            t, color, depth = item
            if recorder:
                recorder.add(t, color, depth)

            t_det = time.monotonic()
            boxes = detect_people(model, color, args)
            detect_ms.append((time.monotonic() - t_det) * 1000.0)

            depth_m = depth.astype(np.float32) * source.depth_scale
            people = [locate(b, depth_m, intr) for b in boxes]
            located = [i for i, p in enumerate(people) if p is not None]
            pick = selector.update([people[i] for i in located], t)
            target_i = located[pick] if pick is not None else None
            target = people[target_i] if target_i is not None else None
            frames += 1

            if target:
                seen += 1
                ranges.append(target["range"])
                bearings.append(target["bearing"])
            if log:
                row = [f"{t:.3f}", len(boxes), int(target is not None)]
                row += ([f"{v:.3f}" for v in target["link"]]
                        + [f"{target['range']:.3f}", f"{target['bearing']:.2f}"]
                        if target else [""] * 5)
                log.writerow(row + [f"{detect_ms[-1]:.1f}"])

            if wall >= next_print or args.replay:
                next_print = wall + 0.5
                rate = frames / max(time.monotonic() - wall0, 1e-6)
                if target:
                    x, y, z = target["link"]
                    ox, oy, oz = target["optical"]
                    print(f"t={t:6.2f}  people={len(boxes)}  {rate:4.1f} fps  "
                          f"link=({x:+.2f}, {y:+.2f}, {z:+.2f})  "
                          f"optical=({ox:+.2f}, {oy:+.2f}, {oz:+.2f})  "
                          f"range {target['range']:.2f} m  "
                          f"bearing {target['bearing']:+.1f} deg")
                else:
                    print(f"t={t:6.2f}  people={len(boxes)}  {rate:4.1f} fps  no target")

            want_save = saved < args.samples and (wall >= next_save or args.replay)
            if want_save or args.show:
                img = annotate(color, boxes, people, target_i)
                if want_save:
                    os.makedirs(save_dir, exist_ok=True)
                    cv2.imwrite(os.path.join(save_dir, f"people_{saved + 1:02d}.png"), img)
                    saved += 1
                    next_save = wall + 1.0
                if args.show:
                    cv2.imshow("apollo person pose", img)
                    if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                        break
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        source.close()
        if recorder:
            recorder.close()
        if log_file:
            log_file.close()

    elapsed = time.monotonic() - wall0
    print()
    if frames == 0:
        print("RESULT: FAIL (no frames processed)")
        return 1
    rate = frames / elapsed
    print(f"frames {frames} in {elapsed:.1f} s = {rate:.1f} fps "
          f"(spec target >= {TARGET_FPS:.0f}); detector median "
          f"{statistics.median(detect_ms):.0f} ms")
    print(f"target located in {seen}/{frames} frames ({100.0 * seen / frames:.0f}%)")
    if ranges:
        spread = statistics.pstdev(ranges) if len(ranges) > 1 else 0.0
        print(f"target range median {statistics.median(ranges):.2f} m "
              f"(spread {spread * 100:.1f} cm), bearing median "
              f"{statistics.median(bearings):+.1f} deg")
        print("stand still at a taped 2 m and 4 m to check the range: "
              "spec allows 10 cm error")
    if saved:
        print(f"saved {saved} annotated picture(s) to {save_dir}/")
    ok = seen > 0 and (args.replay or rate >= TARGET_FPS)
    print("RESULT: PASS" if ok else "RESULT: FAIL "
          + ("(no person located)" if seen == 0 else "(below the frame-rate target)"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
