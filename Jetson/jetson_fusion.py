#!/usr/bin/env python3
"""
ADAS Sensor Fusion Manager - Jetson Side (DepthAI 3.x + YOLOv8n TensorRT)
---------------------------------------------------------------------------
Architecture:
  - InferenceThread   : YOLOv8n TensorRT on GPU, decoupled from capture
  - LaneInferenceThread: UFLD lane detection on GPU, decoupled from capture
  - Main thread       : camera capture, depth, fusion, display
  - SerialBridge      : ultrasonic read/write in background thread

Serial protocol (newline-terminated):
  Pico  -> Jetson : "US,<float_inches>\n"
  Jetson -> Pico  : "CMD,<type>,<param>\n"
      CMD,NONE,0
      CMD,PARK,<0-100>
      CMD,COLLISION,<1|2>
      CMD,LANE,<1|2>
"""

import os
import time
import threading
import traceback
import serial
import serial.tools.list_ports
import depthai as dai
import numpy as np
import cv2
import tensorrt as trt
import torch
from dataclasses import dataclass, field
from typing import Optional
from enum import Enum
from ultralytics import YOLO

# ---------------------------------------------------------------
# Source CONFIG
# ---------------------------------------------------------------
USE_VIDEO_FILE   = False
VIDEO_FILE_PATH  = "test_video.mp4"

# ---------------------------------------------------------------
# Recording CONFIG
# ---------------------------------------------------------------
RECORD_VIDEO      = False
RECORD_OUTPUT_PATH = "/home/vehicles/Desktop/vehicles/Multi-Modal-Autonomous-Vehicle/test_videos/recording.mp4"
RECORD_FPS        = 20
RECORD_DURATION   = 5 * 60   # seconds (0 = unlimited)

# ---------------------------------------------------------------
# Model CONFIG
# ---------------------------------------------------------------
ENGINE_PATH      = "/home/vehicles/Desktop/vehicles/Multi-Modal-Autonomous-Vehicle/models/yolov8n_640_nms.engine"
MODEL_INPUT_SIZE = 640

CONF_TH = 0.5
IOU_TH  = 0.5

# ---------------------------------------------------------------
# Display CONFIG
# ---------------------------------------------------------------
DISPLAY_W    = 1920
DISPLAY_H    = 1080
SHOW_VIDEO   = True
WINDOW_NAME  = "ADAS Sensor Fusion"
DRAW_LIMIT   = 20

# ---------------------------------------------------------------
# Depth CONFIG
# ---------------------------------------------------------------
MIN_MM          = 250
MAX_MM          = 8000
DIST_PERCENTILE = 10.0

FRONT_STOP_DISTANCE = 1.0   # metres
FRONT_SLOW_DISTANCE = 2.0

# ---------------------------------------------------------------
# Loop rate CONFIG
# ---------------------------------------------------------------
PRINT_HZ    = 2.0
DECISION_HZ = 20.0
PRINT_LIST_LIMIT = 10

# ---------------------------------------------------------------
# COCO labels
# ---------------------------------------------------------------
COCO = [
    "person","bicycle","car","motorbike","aeroplane","bus","train","truck","boat",
    "traffic light","fire hydrant","stop sign","parking meter","bench","bird","cat",
    "dog","horse","sheep","cow","elephant","bear","zebra","giraffe","backpack",
    "umbrella","handbag","tie","suitcase","frisbee","skis","snowboard","sports ball",
    "kite","baseball bat","baseball glove","skateboard","surfboard","tennis racket",
    "bottle","wine glass","cup","fork","knife","spoon","bowl","banana","apple",
    "sandwich","orange","broccoli","carrot","hot dog","pizza","donut","cake","chair",
    "sofa","pottedplant","bed","diningtable","toilet","tvmonitor","laptop","mouse",
    "remote","keyboard","cell phone","microwave","oven","toaster","sink",
    "refrigerator","book","clock","vase","scissors","teddy bear","hair drier",
    "toothbrush"
]

ROAD_LABELS    = {"person", "bicycle", "motorbike", "car", "bus", "truck", "train"}
ROAD_CLASS_IDS = {COCO.index(l) for l in ROAD_LABELS if l in COCO}

ROAD_Y_MIN          = 0.55
USE_CENTER_CORRIDOR = True

# ---------------------------------------------------------------
# Serial CONFIG
# ---------------------------------------------------------------
SERIAL_PORT    = "/dev/ttyACM0"
SERIAL_BAUD    = 115200
SERIAL_TIMEOUT = 0.01

ULTRA_MAX_INCHES = 12.0
ULTRA_MIN_INCHES =  3.0

FUSION_ESCALATE_ULTRA_INCHES = 6.0

# ---------------------------------------------------------------
# Lane CONFIG
# ---------------------------------------------------------------
LANE_ENGINE_PATH   = "/home/vehicles/Desktop/vehicles/Multi-Modal-Autonomous-Vehicle/models/ultra-fast-lane-det-culane.engine"
LANE_INPUT_W       = 800
LANE_INPUT_H       = 288
LANE_ROW_ANCHORS   = 18
LANE_COL_SAMPLE    = 200
LANE_DEPART_MARGIN = 0.08
LANE_CONF_TH       = 0.10   # raised from 0.01 to filter ghost detections


# ---------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------
class AlertType(Enum):
    NONE      = "NONE"
    PARK      = "PARK"
    COLLISION = "COLLISION"
    LANE      = "LANE"

@dataclass
class FusionResult:
    alert_type: AlertType = AlertType.NONE
    param:      int       = 0
    timestamp:  float     = 0.0

@dataclass
class DetectionResult:
    boxes:     list  = field(default_factory=list)
    timestamp: float = 0.0

@dataclass
class LaneResult:
    status:    int   = 0
    left_x:    float = 0.3
    right_x:   float = 0.7
    # Full per-row anchor points for polyline drawing: list of (x, y) or None
    left_pts:  list  = None
    right_pts: list  = None
    timestamp: float = 0.0

    def __post_init__(self):
        if self.left_pts is None:
            self.left_pts = []
        if self.right_pts is None:
            self.right_pts = []


# ---------------------------------------------------------------
# YOLOv8 Inference Thread
# ---------------------------------------------------------------
class InferenceThread:
    def __init__(self, engine_path: str):
        print("[Infer] Loading TensorRT engine...")
        try:
            self._model = YOLO(engine_path, task="detect")
        except Exception as e:
            print(f"[Infer] ERROR loading YOLO engine '{engine_path}': {e}")
            traceback.print_exc()
            raise
        print("[Infer] Engine loaded.")

        self._lock        = threading.Lock()
        self._input_frame = None
        self._latest      = DetectionResult()
        self._new_frame   = threading.Event()
        self._running     = True

        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def submit_frame(self, frame: np.ndarray):
        with self._lock:
            self._input_frame = frame.copy()
        self._new_frame.set()

    def get_latest(self) -> DetectionResult:
        with self._lock:
            return self._latest

    def _loop(self):
        try:
            dummy = np.zeros((MODEL_INPUT_SIZE, MODEL_INPUT_SIZE, 3), dtype=np.uint8)
            self._model(dummy, verbose=False)
            print("[Infer] Warmup complete, inference thread running.")
        except Exception as e:
            print(f"[Infer] Warmup failed: {e}")
            traceback.print_exc()
            self._running = False
            return

        while self._running:
            if not self._new_frame.wait(timeout=0.1):
                continue
            self._new_frame.clear()

            with self._lock:
                frame = self._input_frame
            if frame is None:
                continue

            try:
                results   = self._model(frame, conf=CONF_TH, iou=IOU_TH,
                                        classes=list(ROAD_CLASS_IDS), verbose=False)
                boxes_out = []
                boxes = results[0].boxes
                for i in range(min(len(boxes), DRAW_LIMIT)):
                    cls_id = int(boxes.cls[i].item())
                    conf   = float(boxes.conf[i].item())
                    label  = COCO[cls_id] if 0 <= cls_id < len(COCO) else str(cls_id)
                    xn     = boxes.xyxyn[i].cpu().numpy()
                    boxes_out.append((label, conf, xn[0], xn[1], xn[2], xn[3]))

                with self._lock:
                    self._latest = DetectionResult(boxes=boxes_out, timestamp=time.time())
            except Exception as e:
                print(f"[Infer] Inference error: {e}")
                traceback.print_exc()
                continue

    def stop(self):
        self._running = False


# ---------------------------------------------------------------
# UFLD Lane Inference Thread
# ---------------------------------------------------------------
class LaneInferenceThread:
    def __init__(self, engine_path: str):
        print("[Lane] Loading lane TensorRT engine...")
        logger = trt.Logger(trt.Logger.WARNING)
        trt.init_libnvinfer_plugins(logger, "")
        runtime = trt.Runtime(logger)

        if not os.path.exists(engine_path):
            raise FileNotFoundError(f"[Lane] Engine file not found: {engine_path}")

        with open(engine_path, 'rb') as f:
            engine_data = f.read()

        if not engine_data:
            raise RuntimeError(f"[Lane] Engine file is empty: {engine_path}")

        self._engine = None
        for attempt in range(1, 4):
            self._engine = runtime.deserialize_cuda_engine(engine_data)
            if self._engine is not None:
                break
            print(f"[Lane] WARNING: engine deserialize attempt {attempt} failed.")
            time.sleep(0.5)

        if self._engine is None:
            print(f"[Lane] ERROR: Failed to deserialize TensorRT engine from '{engine_path}' after 3 attempts.")
            print(f"[Lane]   Engine path: {engine_path}")
            print(f"[Lane]   Engine size: {len(engine_data)} bytes")
            print(f"[Lane]   TensorRT version: {trt.__version__}")
            raise RuntimeError(
                f"[Lane] Failed to deserialize TensorRT engine from '{engine_path}' after 3 attempts. "
                "Check TensorRT/CUDA compatibility, ensure the engine file is valid, "
                "and confirm the file is not corrupted or still being written."
            )

        self._context = self._engine.create_execution_context()

        self._input_name = None
        self._output_name = None
        for idx in range(self._engine.num_io_tensors):
            name = self._engine.get_tensor_name(idx)
            mode = self._engine.get_tensor_mode(name)
            if mode == trt.TensorIOMode.INPUT:
                self._input_name = name
            elif mode == trt.TensorIOMode.OUTPUT:
                self._output_name = name

        if self._input_name is None or self._output_name is None:
            raise RuntimeError("[Lane] Engine must expose one input and one output tensor.")

        input_shape = tuple(1 if d == -1 else d for d in self._engine.get_tensor_shape(self._input_name))
        output_shape = tuple(1 if d == -1 else d for d in self._engine.get_tensor_shape(self._output_name))

        self._d_input = torch.zeros(input_shape, dtype=torch.float32, device='cuda')
        self._d_output = torch.zeros(output_shape, dtype=torch.float32, device='cuda')

        self._lock        = threading.Lock()
        self._input_frame = None
        self._latest      = LaneResult()
        self._new_frame   = threading.Event()
        self._running     = True

        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        print("[Lane] Engine loaded, lane thread running.")

    def submit_frame(self, frame: np.ndarray):
        with self._lock:
            self._input_frame = frame.copy()
        self._new_frame.set()

    def get_latest(self) -> LaneResult:
        with self._lock:
            return self._latest

    def _preprocess(self, frame: np.ndarray) -> torch.Tensor:
        img  = cv2.resize(frame, (LANE_INPUT_W, LANE_INPUT_H))
        img  = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img  = img.astype(np.float32) / 255.0
        mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
        std  = np.array([0.229, 0.224, 0.225], dtype=np.float32)
        img  = (img - mean) / std
        img  = img.transpose(2, 0, 1)[np.newaxis]
        return torch.from_numpy(np.ascontiguousarray(img)).cuda()
    
    def _postprocess(self, output_tensor: torch.Tensor,
                    display_w: int, display_h: int) -> LaneResult:
        logits = output_tensor.cpu().numpy()[0]   # [201, 18, 4]

        # Softmax
        logits = logits - np.max(logits, axis=0, keepdims=True)
        prob   = np.exp(logits)
        prob  /= np.sum(prob, axis=0, keepdims=True)

        col_idx    = np.argmax(prob, axis=0)   # [18, 4]
        confidence = np.max(prob,   axis=0)   # [18, 4]
        valid      = (col_idx > 0) & (confidence >= LANE_CONF_TH)

        lanes_x = np.where(
            valid,
            (col_idx - 1) / (LANE_COL_SAMPLE - 1) * display_w,
            np.nan
        )

        # Remove edge-clamped artifacts — points within 3% of frame edges
        # are model uncertainty outputs, not real lane detections
        edge_mask = (lanes_x < display_w * 0.03) | (lanes_x > display_w * 0.97)
        lanes_x[edge_mask] = np.nan

        # Row anchor y positions scaled to display height
        row_ys = np.linspace(
            display_h * 0.42,
            display_h * 0.96,
            LANE_ROW_ANCHORS
        ).astype(np.int32)

        # Build clean point list for each lane
        def build_pts(lane_idx):
            pts = []
            prev_x = None
            for row_i in range(LANE_ROW_ANCHORS):
                x = lanes_x[row_i, lane_idx]
                if np.isnan(x):
                    continue
                # Skip points that jump more than 15% of frame width from previous
                if prev_x is not None and abs(x - prev_x) > display_w * 0.15:
                    continue
                pts.append((int(x), int(row_ys[row_i])))
                prev_x = x
            return pts

        # Build all 4 lanes
        all_pts = [build_pts(i) for i in range(4)]

        # Select the 2 inner lanes (closest to frame center from each side)
        center_x = display_w / 2.0

        def mean_x(pts):
            return sum(p[0] for p in pts) / len(pts) if pts else None

        left_candidates  = [(i, pts) for i, pts in enumerate(all_pts)
                            if pts and mean_x(pts) < center_x]
        right_candidates = [(i, pts) for i, pts in enumerate(all_pts)
                            if pts and mean_x(pts) >= center_x]

        # Closest to center from each side
        left_best  = max(left_candidates,  key=lambda t: mean_x(t[1])) \
                    if left_candidates  else None
        right_best = min(right_candidates, key=lambda t: mean_x(t[1])) \
                    if right_candidates else None

        left_pts  = left_best[1]  if left_best  else []
        right_pts = right_best[1] if right_best else []

        # Departure detection using bottom 6 rows of selected lanes
        def bottom_mean_x(pts):
            if not pts:
                return None
            bottom = sorted(pts, key=lambda p: p[1], reverse=True)[:6]
            return sum(p[0] for p in bottom) / len(bottom) / display_w

        left_x  = bottom_mean_x(left_pts)  or 0.3
        right_x = bottom_mean_x(right_pts) or 0.7

        lanes_valid = (left_pts and right_pts
                    and left_x < right_x
                    and left_x > 0.05
                    and right_x < 0.95)

        status = 0
        if lanes_valid and 0.5 < left_x - LANE_DEPART_MARGIN:
            status = 1
        elif lanes_valid and 0.5 > right_x + LANE_DEPART_MARGIN:
            status = 2

        return LaneResult(status=status, left_x=left_x, right_x=right_x,
                        left_pts=left_pts, right_pts=right_pts,
                        timestamp=time.time())


    def _run_inference(self):
        self._context.set_tensor_address(self._input_name, self._d_input.data_ptr())
        self._context.set_tensor_address(self._output_name, self._d_output.data_ptr())
        self._context.execute_async_v3(
            stream_handle=torch.cuda.current_stream().cuda_stream)
        torch.cuda.synchronize()

    def _loop(self):
        # Warmup
        dummy = np.zeros((LANE_INPUT_H, LANE_INPUT_W, 3), dtype=np.uint8)
        self._d_input.copy_(self._preprocess(dummy))
        self._run_inference()
        print("[Lane] Warmup complete.")

        while self._running:
            if not self._new_frame.wait(timeout=0.1):
                continue
            self._new_frame.clear()

            with self._lock:
                frame = self._input_frame
            if frame is None:
                continue

            fh, fw = frame.shape[:2]
            self._d_input.copy_(self._preprocess(frame))
            self._run_inference()

            result = self._postprocess(self._d_output, fw, fh)
            with self._lock:
                self._latest = result

    def stop(self):
        self._running = False


class DummyLaneInferenceThread:
    def __init__(self):
        print("[Lane] Lane inference disabled; no lane engine available.")
        self._latest = LaneResult()
        self._running = False

    def submit_frame(self, frame: np.ndarray):
        pass

    def get_latest(self) -> LaneResult:
        return self._latest

    def stop(self):
        self._running = False


# ---------------------------------------------------------------
# Serial bridge
# ---------------------------------------------------------------
class SerialBridge:
    def __init__(self, port: str, baud: int):
        self._lock          = threading.Lock()
        self._latest_inches = None
        self._last_cmd      = None
        self._running       = False
        self._ser           = None

        try:
            self._ser = serial.Serial(port, baud, timeout=SERIAL_TIMEOUT)
            self._ser.dtr = False
            self._ser.rts = False
            self._running = True
            self._thread  = threading.Thread(target=self._reader_loop, daemon=True)
            self._thread.start()
            print(f"[Serial] Connected to Pico on {port} @ {baud} baud")
        except serial.SerialException as e:
            print(f"[Serial] WARNING - could not open {port}: {e}")

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
                            with self._lock:
                                self._latest_inches = float(line[3:])
                        except ValueError:
                            pass
            except Exception:
                time.sleep(0.05)

    def get_ultrasonic_inches(self) -> Optional[float]:
        with self._lock:
            return self._latest_inches

    def send_command(self, result: FusionResult):
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


# ---------------------------------------------------------------
# Fusion engine
# ---------------------------------------------------------------
class FusionEngine:
    def update(self, camera_action: str, ultra_inches: Optional[float],
               lane_status: int = 0) -> FusionResult:
        result = FusionResult(timestamp=time.time())
        if camera_action == "FULL_STOP":
            result.alert_type = AlertType.COLLISION
            result.param      = 2
        elif camera_action == "SLOW_DOWN":
            if ultra_inches is not None and ultra_inches <= FUSION_ESCALATE_ULTRA_INCHES:
                result.alert_type = AlertType.COLLISION
                result.param      = 2
            else:
                result.alert_type = AlertType.COLLISION
                result.param      = 1
        elif ultra_inches is not None and ultra_inches <= ULTRA_MAX_INCHES:
            result.alert_type = AlertType.PARK
            ratio = (ULTRA_MAX_INCHES - ultra_inches) / (ULTRA_MAX_INCHES - ULTRA_MIN_INCHES)
            result.param = int(max(0.0, min(1.0, ratio)) * 100)
        elif lane_status != 0:
            result.alert_type = AlertType.LANE
            result.param      = lane_status
        else:
            result.alert_type = AlertType.NONE
            result.param      = 0
        return result


# ---------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------
def _is_road_candidate(label, xmin, ymin, xmax, ymax) -> bool:
    if label not in ROAD_LABELS:
        return False
    if float(ymax) < ROAD_Y_MIN:
        return False
    if USE_CENTER_CORRIDOR:
        cx = (float(xmin) + float(xmax)) / 2.0
        cy = float(ymax)
        t  = max(0.0, min(1.0, (cy - ROAD_Y_MIN) / (1.0 - ROAD_Y_MIN)))
        x_min = 0.30 + t * (0.05 - 0.30)
        x_max = 0.70 + t * (0.95 - 0.70)
        if not (x_min <= cx <= x_max):
            return False
    return True


def depth_stat_in_bbox_mm(depth_mm, x0, y0, x1, y1, percentile=DIST_PERCENTILE):
    h, w = depth_mm.shape
    x0 = max(0, min(w-1, x0)); x1 = max(0, min(w, x1))
    y0 = max(0, min(h-1, y0)); y1 = max(0, min(h, y1))
    if x1 <= x0 or y1 <= y0:
        return None

    # Use central 50% of the bounding box to avoid background bleed
    cx = (x0 + x1) // 2
    cy = (y0 + y1) // 2
    half_w = max(1, (x1 - x0) // 4)   # 25% each side = 50% total width
    half_h = max(1, (y1 - y0) // 4)   # 25% each side = 50% total height
    cx0 = max(0, cx - half_w)
    cx1 = min(w, cx + half_w)
    cy0 = max(0, cy - half_h)
    cy1 = min(h, cy + half_h)

    roi   = depth_mm[cy0:cy1, cx0:cx1]
    valid = roi[(roi >= MIN_MM) & (roi <= MAX_MM)]
    if valid.size == 0:
        # Fall back to full box if center crop has no valid depth
        roi   = depth_mm[y0:y1, x0:x1]
        valid = roi[(roi >= MIN_MM) & (roi <= MAX_MM)]
    return int(np.percentile(valid, percentile)) if valid.size else None


def decide_camera_action(closest_m) -> str:
    d = closest_m if closest_m is not None else float("inf")
    if d <= FRONT_STOP_DISTANCE:
        return "FULL_STOP"
    elif d <= FRONT_SLOW_DISTANCE:
        return "SLOW_DOWN"
    return "KEEP_SPEED"


def _letterbox(frame: np.ndarray) -> tuple:
    """Letterbox frame to MODEL_INPUT_SIZE square. Returns (lb, scale, pad_x, pad_y)."""
    fh, fw = frame.shape[:2]
    scale  = min(MODEL_INPUT_SIZE / fw, MODEL_INPUT_SIZE / fh)
    new_w  = int(fw * scale)
    new_h  = int(fh * scale)
    pad_x  = (MODEL_INPUT_SIZE - new_w) // 2
    pad_y  = (MODEL_INPUT_SIZE - new_h) // 2
    lb     = np.zeros((MODEL_INPUT_SIZE, MODEL_INPUT_SIZE, 3), dtype=np.uint8)
    lb[pad_y:pad_y+new_h, pad_x:pad_x+new_w] = cv2.resize(frame, (new_w, new_h))
    return lb, scale, pad_x, pad_y


def _unscale_boxes(det_result, fw, fh, scale, pad_x, pad_y):
    """Convert normalised letterbox coords to pixel coords in display frame."""
    lines = []
    for (label, conf, xmin, ymin, xmax, ymax) in det_result.boxes:
        x0 = int((xmin * MODEL_INPUT_SIZE - pad_x) / scale)
        y0 = int((ymin * MODEL_INPUT_SIZE - pad_y) / scale)
        x1 = int((xmax * MODEL_INPUT_SIZE - pad_x) / scale)
        y1 = int((ymax * MODEL_INPUT_SIZE - pad_y) / scale)
        x0 = max(0, min(fw, x0)); x1 = max(0, min(fw, x1))
        y0 = max(0, min(fh, y0)); y1 = max(0, min(fh, y1))
        xmin_n = x0/fw; ymin_n = y0/fh
        xmax_n = x1/fw; ymax_n = y1/fh
        if not _is_road_candidate(label, xmin_n, ymin_n, xmax_n, ymax_n):
            continue
        lines.append((label, conf, x0, y0, x1, y1, xmin_n, ymin_n, xmax_n, ymax_n))
    return lines


def _draw_overlay(frame, det_lines, lane_result, last_fusion, fh, fw):
    alert_colours = {
        AlertType.NONE:      (200, 200, 200),
        AlertType.PARK:      (0,   200, 255),
        AlertType.COLLISION: (0,   0,   255),
        AlertType.LANE:      (0,   255, 200),
    }

    # Detection boxes
    for (lbl, conf, x0, y0, x1, y1, *rest) in det_lines[:DRAW_LIMIT]:
        # rest[-1] is dist_m if present
        dist_m = rest[-1] if rest else None
        dist_str = f" {dist_m:.1f}m" if dist_m is not None else ""
        cv2.rectangle(frame, (x0, y0), (x1, y1), (255, 0, 0), 2)
        cv2.putText(frame, f"{lbl} {conf:.2f}{dist_str}",
                    (x0, max(0, y0-6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    # Lane dots — left lane cyan, right lane yellow
    for pt in lane_result.left_pts:
        cv2.circle(frame, pt, 5, (0, 255, 255), -1)
    for pt in lane_result.right_pts:
        cv2.circle(frame, pt, 5, (0, 215, 255), -1)

    # Alert banner
    colour = alert_colours.get(last_fusion.alert_type, (255, 255, 255))
    cv2.putText(frame,
                f"ALERT: {last_fusion.alert_type.value} [{last_fusion.param}]",
                (10, fh-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, colour, 2)

    # Lane departure text
    if lane_result.status != 0:
        direction = "LEFT" if lane_result.status == 1 else "RIGHT"
        cv2.putText(frame, f"LANE DEPARTURE: {direction}",
                    (10, fh-40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 200), 2)


def _make_writer():
    if not RECORD_VIDEO:
        return None
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = cv2.VideoWriter(RECORD_OUTPUT_PATH, fourcc, RECORD_FPS,
                              (DISPLAY_W, DISPLAY_H))
    print(f"[Record] Saving to {RECORD_OUTPUT_PATH}")
    return writer


# ---------------------------------------------------------------
# DepthAI pipeline
# ---------------------------------------------------------------
def build_pipeline():
    device_info = dai.DeviceInfo()
    device      = dai.Device(device_info, dai.UsbSpeed.SUPER_PLUS)
    pipeline    = dai.Pipeline(device)

    cam   = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    q_rgb = cam.requestOutput(
        (DISPLAY_W, DISPLAY_H),
        type=dai.ImgFrame.Type.BGR888p
    ).createOutputQueue(maxSize=2, blocking=False)

    left  = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

    stereo = pipeline.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.FAST_DENSITY)
    stereo.setLeftRightCheck(True)
    stereo.setSubpixel(True)
    left.requestOutput((640, 400)).link(stereo.left)
    right.requestOutput((640, 400)).link(stereo.right)

    q_depth = stereo.depth.createOutputQueue(maxSize=2, blocking=False)
    return pipeline, device, q_rgb, q_depth


# ---------------------------------------------------------------
# Live camera loop
# ---------------------------------------------------------------
def _run_live(pipeline, q_rgb, q_depth, infer, lane_infer, serial_bridge, fusion):
    print_period    = 1.0 / max(0.1, PRINT_HZ)
    decision_period = 1.0 / max(0.1, DECISION_HZ)
    last_print      = 0.0
    last_decision   = 0.0
    last_action     = "KEEP_SPEED"
    last_fusion     = FusionResult()
    latest_depth    = None
    writer          = _make_writer()
    record_start    = time.time()

    pipeline.start()
    print("ADAS Sensor Fusion running. Press 'q' to quit.")

    try:
        while pipeline.isRunning():
            if SHOW_VIDEO:
                cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
                cv2.setWindowProperty(WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            # Drain depth queue
            d = None
            while True:
                msg = q_depth.tryGet()
                if msg is None:
                    break
                d = msg
            if d is not None:
                latest_depth = d.getFrame()
                latest_depth = np.rot90(latest_depth, 2)

            # Drain RGB queue
            rgb_msg = None
            while True:
                msg = q_rgb.tryGet()
                if msg is None:
                    break
                rgb_msg = msg
            if rgb_msg is None:
                continue

            frame = rgb_msg.getCvFrame()
            # The RGB camera is mounted upside down, so rotate 180 degrees to correct orientation.  
            # frame = cv2.rotate(frame, cv2.ROTATE_180)
            fh, fw = frame.shape[:2]

            # Letterbox for YOLO inference
            lb, scale, pad_x, pad_y = _letterbox(frame)

            # Submit to both inference threads
            infer.submit_frame(lb)
            lane_infer.submit_frame(frame)

            # Parse detections
            det_result = infer.get_latest()
            det_lines  = _unscale_boxes(det_result, fw, fh, scale, pad_x, pad_y)

            # Depth lookup per detection
            closest_det_m = None
            closest_label = None
            closest_conf  = None
            det_lines_with_depth = []

            if latest_depth is not None:
                dh, dw = latest_depth.shape
                for (lbl, conf, x0, y0, x1, y1, xmin_n, ymin_n, xmax_n, ymax_n) in det_lines:
                    dx0 = int(xmin_n * dw); dy0 = int(ymin_n * dh)
                    dx1 = int(xmax_n * dw); dy1 = int(ymax_n * dh)
                    mm     = depth_stat_in_bbox_mm(latest_depth, dx0, dy0, dx1, dy1)
                    dist_m = (mm / 1000.0) if mm is not None else None
                    det_lines_with_depth.append((lbl, conf, x0, y0, x1, y1, dist_m))
                    if dist_m is not None and (closest_det_m is None or dist_m < closest_det_m):
                        closest_det_m = dist_m
                        closest_label = lbl
                        closest_conf  = conf
            else:
                det_lines_with_depth = [
                    (lbl, conf, x0, y0, x1, y1, None)
                    for (lbl, conf, x0, y0, x1, y1, *_) in det_lines
                ]

            now = time.time()

            # Fusion + serial at DECISION_HZ
            if now - last_decision >= decision_period:
                last_decision = now
                last_action   = decide_camera_action(closest_det_m)
                ultra_inches  = serial_bridge.get_ultrasonic_inches()
                lane_result   = lane_infer.get_latest()
                last_fusion   = fusion.update(last_action, ultra_inches,
                                              lane_status=lane_result.status)
                serial_bridge.send_command(last_fusion)

            # Console print at PRINT_HZ
            if now - last_print >= print_period:
                last_print   = now
                ultra_inches = serial_bridge.get_ultrasonic_inches()
                lane_result  = lane_infer.get_latest()
                print(f"\n[{time.strftime('%H:%M:%S')}] "
                      f"road_dets={len(det_lines_with_depth)} | "
                      f"ultrasonic={'N/A' if ultra_inches is None else f'{ultra_inches:.1f}in'}")
                if closest_det_m is not None:
                    print(f"  >>> Closest: {closest_label}  "
                          f"dist={closest_det_m:.2f}m  conf={closest_conf:.2f}")
                else:
                    print("  No road detections with valid depth.")
                for (lbl, conf, *_, dist_m) in det_lines_with_depth[:PRINT_LIST_LIMIT]:
                    print(f"  - {lbl:<12} conf={conf:.2f}  "
                          f"dist={'N/A' if dist_m is None else f'{dist_m:.2f}m'}")
                print(f"  LANE status={lane_result.status} "
                      f"L={lane_result.left_x:.2f} R={lane_result.right_x:.2f}")
                print(f"  CAMERA: {last_action}")
                print(f"  FUSION -> {last_fusion.alert_type.value}, param={last_fusion.param}")

            # Display
            if SHOW_VIDEO:
                lane_result = lane_infer.get_latest()
                _draw_overlay(frame, det_lines_with_depth, lane_result, last_fusion, fh, fw)

                if writer is not None:
                    writer.write(cv2.resize(frame, (DISPLAY_W, DISPLAY_H)))
                    if RECORD_DURATION > 0 and time.time() - record_start >= RECORD_DURATION:
                        print("[Record] Duration reached, stopping recording.")
                        writer.release()
                        writer = None

                cv2.imshow(WINDOW_NAME, frame)
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break

    finally:
        if writer is not None:
            writer.release()
        pipeline.stop()


# ---------------------------------------------------------------
# Video file loop
# ---------------------------------------------------------------
def _run_video(cap, infer, lane_infer, serial_bridge, fusion):
    print_period    = 1.0 / max(0.1, PRINT_HZ)
    decision_period = 1.0 / max(0.1, DECISION_HZ)
    last_print      = 0.0
    last_decision   = 0.0
    last_fusion     = FusionResult()
    writer          = _make_writer()
    record_start    = time.time()

    try:
        while True:
            if SHOW_VIDEO:
                cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
                cv2.setWindowProperty(WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            ret, frame = cap.read()
            if not ret:
                print("[Video] End of file.")
                break

            fh, fw = frame.shape[:2]
            lb, scale, pad_x, pad_y = _letterbox(frame)

            infer.submit_frame(lb)
            lane_infer.submit_frame(frame)

            det_result = infer.get_latest()
            det_lines  = _unscale_boxes(det_result, fw, fh, scale, pad_x, pad_y)
            det_lines_with_depth = [
                (lbl, conf, x0, y0, x1, y1, None)
                for (lbl, conf, x0, y0, x1, y1, *_) in det_lines
            ]

            closest_label = det_lines[0][0] if det_lines else None

            now = time.time()

            if now - last_decision >= decision_period:
                last_decision = now
                camera_action = "SLOW_DOWN" if det_lines else "KEEP_SPEED"
                ultra_inches  = serial_bridge.get_ultrasonic_inches()
                lane_result   = lane_infer.get_latest()
                last_fusion   = fusion.update(camera_action, ultra_inches,
                                              lane_status=lane_result.status)
                serial_bridge.send_command(last_fusion)

            if now - last_print >= print_period:
                last_print  = now
                lane_result = lane_infer.get_latest()
                print(f"\n[{time.strftime('%H:%M:%S')}] road_dets={len(det_lines)}")
                print(f"  LANE status={lane_result.status} "
                      f"L={lane_result.left_x:.2f} R={lane_result.right_x:.2f}")
                print(f"  FUSION -> {last_fusion.alert_type.value}, param={last_fusion.param}")

            if SHOW_VIDEO:
                lane_result = lane_infer.get_latest()
                _draw_overlay(frame, det_lines_with_depth, lane_result, last_fusion, fh, fw)

                if writer is not None:
                    writer.write(cv2.resize(frame, (DISPLAY_W, DISPLAY_H)))
                    if RECORD_DURATION > 0 and time.time() - record_start >= RECORD_DURATION:
                        writer.release()
                        writer = None

                cv2.imshow(WINDOW_NAME, frame)
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break

    finally:
        if writer is not None:
            writer.release()
        cap.release()


# ---------------------------------------------------------------
# Main
# ---------------------------------------------------------------
def main():
    infer         = InferenceThread(ENGINE_PATH)
    try:
        lane_infer = LaneInferenceThread(LANE_ENGINE_PATH)
    except RuntimeError as e:
        print(f"[Lane] WARNING: {e}")
        traceback.print_exc()
        lane_infer = DummyLaneInferenceThread()
    serial_bridge = SerialBridge(SERIAL_PORT, SERIAL_BAUD)
    fusion        = FusionEngine()

    try:
        if USE_VIDEO_FILE:
            cap = cv2.VideoCapture(VIDEO_FILE_PATH)
            if not cap.isOpened():
                print(f"[ERROR] Could not open video: {VIDEO_FILE_PATH}")
                return
            print(f"[Video] Playing: {VIDEO_FILE_PATH}")
            _run_video(cap, infer, lane_infer, serial_bridge, fusion)
        else:
            pipeline, device, q_rgb, q_depth = build_pipeline()
            _run_live(pipeline, q_rgb, q_depth,
                      infer, lane_infer, serial_bridge, fusion)
    finally:
        infer.stop()
        lane_infer.stop()
        serial_bridge.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()