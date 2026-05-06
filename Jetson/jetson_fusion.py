#!/usr/bin/env python3
"""
ADAS Sensor Fusion Manager — Jetson Side
-----------------------------------------
Combines OAK-D Pro / YOLOv4 camera detections with ultrasonic proximity data
streamed from the Raspberry Pi Pico 2 W over USB serial.

Serial protocol (newline-terminated):
  Pico  → Jetson : "US,<float_inches>\n"          e.g. "US,7.23\n"
  Jetson → Pico  : "CMD,<type>,<param>\n"
      CMD,NONE,0          — silence all alerts
      CMD,PARK,<0-100>    — parking proximity (100 = closest)
      CMD,COLLISION,<1|2> — camera threat (1=slow, 2=full stop)
      CMD,LANE,<1|2>      — lane departure (1=left, 2=right) [future]

Fusion priority (highest → lowest):
  1. COLLISION — camera confirms imminent forward threat
  2. PARK      — ultrasonic close-range parking assist
  3. NONE      — all clear
Lane departure runs as a parallel, non-exclusive alert.
"""

import time
import threading
import serial
import serial.tools.list_ports
import depthai as dai
import numpy as np
import cv2
from dataclasses import dataclass
from typing import Optional
from enum import Enum

# ─────────────────────────────────────────────
# Camera CONFIG (preserved from original)
# ─────────────────────────────────────────────
BLOB_PATH = "/home/vehicles/Desktop/vehicles/Multi-Modal-Autonomous-Vehicle/models/yolov4_tiny_coco_416x416_openvino_2022.1_6shave.blob"
CONF_TH = 0.5
IOU_TH  = 0.5

MIN_MM  = 250
MAX_MM  = 8000
DIST_PERCENTILE = 10.0

ROI_HALF       = 70
ROI_PERCENTILE = 10.0

FRONT_STOP_DISTANCE = 1.0   # metres — FULL_STOP
FRONT_SLOW_DISTANCE = 2.0   # metres — SLOW_DOWN

PRINT_HZ    = 2.0
DECISION_HZ = 20.0

SHOW_VIDEO    = True
WINDOW_NAME   = "ADAS Sensor Fusion"
DRAW_LIMIT    = 20
PRINT_LIST_LIMIT = 10

ROAD_LABELS = {"person", "bicycle", "motorbike", "car", "bus", "truck", "train"}
ROAD_Y_MIN  = 0.55
USE_CENTER_CORRIDOR = True
CENTER_X_MIN = 0.20
CENTER_X_MAX = 0.80

COCO = [
    "person","bicycle","car","motorbike","aeroplane","bus","train","truck","boat","traffic light",
    "fire hydrant","stop sign","parking meter","bench","bird","cat","dog","horse","sheep","cow",
    "elephant","bear","zebra","giraffe","backpack","umbrella","handbag","tie","suitcase","frisbee",
    "skis","snowboard","sports ball","kite","baseball bat","baseball glove","skateboard","surfboard",
    "tennis racket","bottle","wine glass","cup","fork","knife","spoon","bowl","banana","apple",
    "sandwich","orange","broccoli","carrot","hot dog","pizza","donut","cake","chair","sofa",
    "pottedplant","bed","diningtable","toilet","tvmonitor","laptop","mouse","remote","keyboard",
    "cell phone","microwave","oven","toaster","sink","refrigerator","book","clock","vase",
    "scissors","teddy bear","hair drier","toothbrush"
]

# ─────────────────────────────────────────────
# Serial CONFIG
# ─────────────────────────────────────────────
SERIAL_PORT      = "/dev/ttyACM0"   # change if Pico enumerates differently
SERIAL_BAUD      = 115200
SERIAL_TIMEOUT   = 0.01             # non-blocking read timeout (seconds)

# Ultrasonic distance thresholds (inches, matching Pico firmware)
ULTRA_MAX_INCHES = 12.0             # beyond this → no parking alert
ULTRA_MIN_INCHES =  3.0             # at or below this → maximum alert

# ─────────────────────────────────────────────
# Fusion thresholds
# ─────────────────────────────────────────────
# Ultrasonic AND camera both detecting threat within these distances
# causes an escalation to COLLISION,2 even if camera alone says SLOW_DOWN
FUSION_ESCALATE_ULTRA_INCHES = 6.0  # ultrasonic < 6in while camera says slow → escalate

# ─────────────────────────────────────────────────────────────────────────────
# Alert types
# ─────────────────────────────────────────────────────────────────────────────
class AlertType(Enum):
    NONE      = "NONE"
    PARK      = "PARK"
    COLLISION = "COLLISION"
    LANE      = "LANE"

@dataclass
class FusionResult:
    alert_type: AlertType = AlertType.NONE
    param:      int       = 0       # PARK: 0-100 severity | COLLISION: 1/2 | LANE: 1/2
    timestamp:  float     = 0.0


# ─────────────────────────────────────────────────────────────────────────────
# Serial bridge — runs in its own thread
# ─────────────────────────────────────────────────────────────────────────────
class SerialBridge:
    """
    Background thread that:
      • Continuously reads 'US,<inches>' lines from the Pico
      • Exposes the latest ultrasonic reading via get_ultrasonic_inches()
      • Sends 'CMD,<type>,<param>' commands to the Pico via send_command()
    """

    def __init__(self, port: str, baud: int):
        self._lock            = threading.Lock()
        self._latest_inches   = None
        self._last_cmd        = None
        self._running         = False
        self._ser             = None

        try:
            self._ser = serial.Serial(port, baud, timeout=SERIAL_TIMEOUT)
            self._ser.dtr = False
            self._ser.rts = False
            self._running = True
            self._thread = threading.Thread(target=self._reader_loop, daemon=True)
            self._thread.start()
            print(f"[Serial] Connected to Pico on {port} @ {baud} baud")
        except serial.SerialException as e:
            print(f"[Serial] WARNING — could not open {port}: {e}")
            print("[Serial] Continuing without ultrasonic data. "
                  "Check SERIAL_PORT constant or run: ls /dev/ttyACM*")

    def _reader_loop(self):
        buf = ""
        while self._running:
            try:
                chunk = self._ser.read(64).decode("utf-8", errors="ignore")
                buf += chunk
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.strip()
                    if line.startswith("US,"):
                        try:
                            inches = float(line[3:])
                            with self._lock:
                                self._latest_inches = inches
                        except ValueError:
                            pass
            except Exception:
                time.sleep(0.05)

    def get_ultrasonic_inches(self) -> Optional[float]:
        with self._lock:
            return self._latest_inches

    def send_command(self, result: FusionResult):
        """Send a CMD line to the Pico. Deduplicates — only sends on change."""
        cmd = f"CMD,{result.alert_type.value},{result.param}\n"
        if cmd == self._last_cmd:
            return
        self._last_cmd = cmd
        if self._ser and self._ser.is_open:
            try:
                self._ser.write(cmd.encode("utf-8"))
                self._ser.flush()
            except Exception as e:
                print(f"[Serial] Write error: {e}")

    def close(self):
        self._running = False
        if self._ser:
            self._ser.close()


# ─────────────────────────────────────────────────────────────────────────────
# Fusion engine
# ─────────────────────────────────────────────────────────────────────────────
class FusionEngine:
    """
    Stateless fusion of camera and ultrasonic data into a single FusionResult.

    Decision hierarchy:
      COLLISION,2  — camera FULL_STOP, OR (camera SLOW_DOWN AND ultra < 6in)
      COLLISION,1  — camera SLOW_DOWN only
      PARK,<sev>   — ultrasonic within range, camera not alerting
      NONE,0       — all clear

    Lane departure is additive: it can stack on top of PARK or NONE but does
    NOT override a COLLISION. It is returned as a secondary field so the Pico
    can layer a lane tone on top of proximity beeping.
    """

    def update(
        self,
        camera_action:    str,           # "FULL_STOP" | "SLOW_DOWN" | "KEEP_SPEED"
        ultra_inches:     Optional[float],
        lane_status:      int = 0        # 0=none, 1=left, 2=right  (future)
    ) -> FusionResult:

        result = FusionResult(timestamp=time.time())

        # ── 1. Collision priority ────────────────────────────────────────────
        if camera_action == "FULL_STOP":
            result.alert_type = AlertType.COLLISION
            result.param      = 2

        elif camera_action == "SLOW_DOWN":
            # Escalate if ultrasonic also confirms close range
            if ultra_inches is not None and ultra_inches <= FUSION_ESCALATE_ULTRA_INCHES:
                result.alert_type = AlertType.COLLISION
                result.param      = 2
            else:
                result.alert_type = AlertType.COLLISION
                result.param      = 1

        # ── 2. Parking assist (ultrasonic, camera not alarming) ──────────────
        elif ultra_inches is not None and ultra_inches <= ULTRA_MAX_INCHES:
            result.alert_type = AlertType.PARK
            # Map distance to severity 0-100 (100 = closest)
            ratio = (ULTRA_MAX_INCHES - ultra_inches) / (ULTRA_MAX_INCHES - ULTRA_MIN_INCHES)
            ratio = max(0.0, min(1.0, ratio))
            result.param = int(ratio * 100)

        # ── 3. Lane departure (additive — only when no collision) ────────────
        #    Override result with LANE only when camera is clear.
        #    If there is already a PARK alert we still send LANE so the Pico
        #    can play the lane tone briefly between parking beeps.
        elif lane_status != 0:
            result.alert_type = AlertType.LANE
            result.param      = lane_status   # 1=left, 2=right

        # ── 4. All clear ─────────────────────────────────────────────────────
        else:
            result.alert_type = AlertType.NONE
            result.param      = 0

        return result


# ─────────────────────────────────────────────────────────────────────────────
# Camera helpers (preserved from original, unchanged)
# ─────────────────────────────────────────────────────────────────────────────
def _is_road_candidate(label: str, det) -> bool:
    if label not in ROAD_LABELS:
        return False
    if float(det.ymax) < ROAD_Y_MIN:
        return False
    if USE_CENTER_CORRIDOR:
        cx = (float(det.xmin) + float(det.xmax)) / 2.0
        if not (CENTER_X_MIN <= cx <= CENTER_X_MAX):
            return False
    return True


def depth_stat_in_bbox_mm(depth_mm, x0, y0, x1, y1, percentile=DIST_PERCENTILE):
    h, w = depth_mm.shape
    x0 = max(0, min(w - 1, x0)); x1 = max(0, min(w, x1))
    y0 = max(0, min(h - 1, y0)); y1 = max(0, min(h, y1))
    if x1 <= x0 or y1 <= y0:
        return None
    roi   = depth_mm[y0:y1, x0:x1]
    valid = roi[(roi >= MIN_MM) & (roi <= MAX_MM)]
    return int(np.percentile(valid, percentile)) if valid.size else None


def depth_only_nearest_mm(depth_mm, roi_half=ROI_HALF, percentile=ROI_PERCENTILE):
    h, w   = depth_mm.shape
    cx, cy = w // 2, h // 2
    x0 = max(0, cx - roi_half); x1 = min(w, cx + roi_half)
    y0 = max(0, cy - roi_half); y1 = min(h, cy + roi_half)
    roi   = depth_mm[y0:y1, x0:x1]
    valid = roi[(roi >= MIN_MM) & (roi <= MAX_MM)]
    if valid.size == 0:
        return None, (x0, y0, x1, y1)
    return int(np.percentile(valid, percentile)), (x0, y0, x1, y1)


def decide_camera_action(closest_m, nearest_ahead_m) -> str:
    if closest_m is not None:
        d = closest_m
    elif nearest_ahead_m is not None:
        d = nearest_ahead_m
    else:
        d = float("inf")

    if d <= FRONT_STOP_DISTANCE:
        return "FULL_STOP"
    elif d <= FRONT_SLOW_DISTANCE:
        return "SLOW_DOWN"
    return "KEEP_SPEED"


def build_pipeline():
    pipeline = dai.Pipeline()

    cam = pipeline.createColorCamera()
    cam.setPreviewSize(416, 416)
    cam.setInterleaved(False)
    cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

    left  = pipeline.createMonoCamera()
    right = pipeline.createMonoCamera()
    left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
    left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

    stereo = pipeline.createStereoDepth()
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
    stereo.setLeftRightCheck(True)
    stereo.setSubpixel(True)
    left.out.link(stereo.left)
    right.out.link(stereo.right)

    yolo = pipeline.createYoloDetectionNetwork()
    yolo.setBlobPath(BLOB_PATH)
    yolo.setConfidenceThreshold(CONF_TH)
    yolo.setIouThreshold(IOU_TH)
    yolo.input.setBlocking(False)
    yolo.setNumClasses(80)
    yolo.setCoordinateSize(4)
    yolo.setAnchors([10,14, 23,27, 37,58, 81,82, 135,169, 344,319])
    yolo.setAnchorMasks({"side13": [3,4,5], "side26": [0,1,2]})
    cam.preview.link(yolo.input)

    xout_det   = pipeline.createXLinkOut(); xout_det.setStreamName("detections")
    xout_depth = pipeline.createXLinkOut(); xout_depth.setStreamName("depth")
    xout_rgb   = pipeline.createXLinkOut(); xout_rgb.setStreamName("rgb")

    yolo.out.link(xout_det.input)
    stereo.depth.link(xout_depth.input)
    cam.preview.link(xout_rgb.input)

    return pipeline


# ─────────────────────────────────────────────────────────────────────────────
# Main loop
# ─────────────────────────────────────────────────────────────────────────────
def main():
    serial_bridge = SerialBridge(SERIAL_PORT, SERIAL_BAUD)
    fusion        = FusionEngine()
    pipeline      = build_pipeline()

    print_period    = 1.0 / max(0.1, PRINT_HZ)
    decision_period = 1.0 / max(0.1, DECISION_HZ)
    last_print      = 0.0
    last_decision   = 0.0
    last_action     = "KEEP_SPEED"
    last_fusion     = FusionResult()

    with dai.Device(pipeline) as device:
        q_det   = device.getOutputQueue("detections", maxSize=4, blocking=False)
        q_depth = device.getOutputQueue("depth",      maxSize=4, blocking=False)
        q_rgb   = device.getOutputQueue("rgb",        maxSize=4, blocking=True)

        latest_depth = None
        latest_dets  = []

        print("✅ ADAS Sensor Fusion running. Press 'q' to quit.")

        while True:
            # ── Pull latest sensor frames ────────────────────────────────────
            d = q_depth.tryGet()
            if d is not None:
                latest_depth = d.getFrame()
                latest_depth = np.rot90(latest_depth, 2)

            det_msg = q_det.tryGet()
            if det_msg is not None:
                latest_dets = det_msg.detections

            frame       = q_rgb.get().getCvFrame()
            frame = cv2.rotate(frame, cv2.ROTATE_180)
            fh, fw      = frame.shape[:2]

            closest_det_m   = None
            closest_label   = None
            closest_conf    = None
            nearest_ahead_m = None
            roi_rect        = None
            det_lines       = []

            if latest_depth is not None:
                dh, dw = latest_depth.shape

                mm_roi, roi_rect = depth_only_nearest_mm(latest_depth)
                if mm_roi is not None:
                    nearest_ahead_m = mm_roi / 1000.0

                for det in latest_dets[:DRAW_LIMIT]:
                    lid   = int(det.label)
                    label = COCO[lid] if 0 <= lid < len(COCO) else str(lid)
                    if not _is_road_candidate(label, det):
                        continue

                    dx0 = int(det.xmin * dw); dy0 = int(det.ymin * dh)
                    dx1 = int(det.xmax * dw); dy1 = int(det.ymax * dh)
                    mm     = depth_stat_in_bbox_mm(latest_depth, dx0, dy0, dx1, dy1)
                    dist_m = (mm / 1000.0) if mm is not None else None
                    det_lines.append((label, float(det.confidence), dist_m, det))

                    if dist_m is not None and (closest_det_m is None or dist_m < closest_det_m):
                        closest_det_m  = dist_m
                        closest_label  = label
                        closest_conf   = float(det.confidence)

            now = time.time()

            # ── Fusion + serial output at DECISION_HZ ────────────────────────
            if now - last_decision >= decision_period:
                last_decision = now
                last_action   = decide_camera_action(closest_det_m, nearest_ahead_m)
                ultra_inches  = serial_bridge.get_ultrasonic_inches()

                # lane_status = 0 until lane departure model is integrated
                last_fusion = fusion.update(last_action, ultra_inches, lane_status=0)
                serial_bridge.send_command(last_fusion)
                print(f"[DEBUG] Sent → CMD,{last_fusion.alert_type.value},{last_fusion.param}")

            # ── Console print at PRINT_HZ ────────────────────────────────────
            if now - last_print >= print_period:
                last_print    = now
                nearest_str   = "N/A" if nearest_ahead_m is None else f"{nearest_ahead_m:.2f}m"
                ultra_inches  = serial_bridge.get_ultrasonic_inches()
                ultra_str     = "N/A" if ultra_inches is None else f"{ultra_inches:.1f}in"

                print(f"\n[{time.strftime('%H:%M:%S')}] "
                      f"road_dets={len(det_lines)} | "
                      f"nearest_ahead={nearest_str} | "
                      f"ultrasonic={ultra_str}")

                if closest_det_m is not None:
                    print(f"  >>> Closest ROAD: {closest_label}  "
                          f"dist={closest_det_m:.2f}m  conf={closest_conf:.2f}")
                else:
                    print("  No ROAD-relevant detections with valid depth.")

                for (lbl, conf, dist_m, _) in det_lines[:PRINT_LIST_LIMIT]:
                    dist_str = "N/A" if dist_m is None else f"{dist_m:.2f}m"
                    print(f"  - {lbl:<12} conf={conf:.2f}  dist={dist_str}")

                print(f"  CAMERA: {last_action}")
                print(f"  FUSION → {last_fusion.alert_type.value}, param={last_fusion.param}")

            # ── Video overlay ────────────────────────────────────────────────
            if SHOW_VIDEO:
                for (lbl, conf, dist_m, det) in det_lines[:DRAW_LIMIT]:
                    x0 = int(det.xmin * fw); y0 = int(det.ymin * fh)
                    x1 = int(det.xmax * fw); y1 = int(det.ymax * fh)
                    dist_str = "N/A" if dist_m is None else f"{dist_m:.2f}m"
                    cv2.rectangle(frame, (x0, y0), (x1, y1), (0, 255, 0), 2)
                    cv2.putText(frame, f"{lbl} {conf:.2f} {dist_str}",
                                (x0, max(0, y0 - 6)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                if roi_rect is not None and latest_depth is not None:
                    dh, dw = latest_depth.shape
                    rx0, ry0, rx1, ry1 = roi_rect
                    cv2.rectangle(frame,
                                  (int(rx0/dw*fw), int(ry0/dh*fh)),
                                  (int(rx1/dw*fw), int(ry1/dh*fh)),
                                  (255, 0, 0), 2)
                    cv2.putText(frame, f"Nearest ahead: {nearest_str}",
                                (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                (255, 255, 255), 2)

                # Fusion alert overlay
                alert_colours = {
                    AlertType.NONE:      (200, 200, 200),
                    AlertType.PARK:      (0,   200, 255),
                    AlertType.COLLISION: (0,   0,   255),
                    AlertType.LANE:      (0,   255, 200),
                }
                colour     = alert_colours.get(last_fusion.alert_type, (255, 255, 255))
                alert_text = f"ALERT: {last_fusion.alert_type.value} [{last_fusion.param}]"
                cv2.putText(frame, alert_text, (10, fh - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, colour, 2)

                cv2.imshow(WINDOW_NAME, frame)
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break

    serial_bridge.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()