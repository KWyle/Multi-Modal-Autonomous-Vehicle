import time
import depthai as dai
import numpy as np

# If you want the live window, you need OpenCV
import cv2

# -----------------------
# CONFIG
# -----------------------
BLOB_PATH = "models/yolov4_tiny_coco_416x416_openvino_2022.1_6shave.blob"
CONF_TH = 0.5
IOU_TH = 0.5

PRINT_HZ = 2.0                  # terminal update rate
DIST_PERCENTILE = 10.0          # stable "closest" inside bbox
MIN_MM = 250                    # ignore too-close noise
MAX_MM = 8000                   # ignore far junk

ENABLE_DEPTH_ONLY_NEAREST = True
ROI_HALF = 70
ROI_PERCENTILE = 10.0

SHOW_VIDEO = True               # <-- live feed window
WINDOW_NAME = "OAK-D YOLO + Distance"
DRAW_LIMIT = 20                 # draw up to N detections

# COCO labels (0-79)
COCO = [
    "person","bicycle","car","motorbike","aeroplane","bus","train","truck","boat","traffic light",
    "fire hydrant","stop sign","parking meter","bench","bird","cat","dog","horse","sheep","cow",
    "elephant","bear","zebra","giraffe","backpack","umbrella","handbag","tie","suitcase","frisbee",
    "skis","snowboard","sports ball","kite","baseball bat","baseball glove","skateboard","surfboard","tennis racket","bottle",
    "wine glass","cup","fork","knife","spoon","bowl","banana","apple","sandwich","orange",
    "broccoli","carrot","hot dog","pizza","donut","cake","chair","sofa","pottedplant","bed",
    "diningtable","toilet","tvmonitor","laptop","mouse","remote","keyboard","cell phone","microwave","oven",
    "toaster","sink","refrigerator","book","clock","vase","scissors","teddy bear","hair drier","toothbrush"
]


def depth_stat_in_bbox_mm(depth_mm: np.ndarray, x0: int, y0: int, x1: int, y1: int,
                          percentile: float = DIST_PERCENTILE) -> int | None:
    """Return stable nearest-ish distance in bbox using low percentile of valid depth."""
    h, w = depth_mm.shape
    x0 = max(0, min(w - 1, x0))
    x1 = max(0, min(w, x1))
    y0 = max(0, min(h - 1, y0))
    y1 = max(0, min(h, y1))
    if x1 <= x0 or y1 <= y0:
        return None

    roi = depth_mm[y0:y1, x0:x1]
    valid = roi[(roi >= MIN_MM) & (roi <= MAX_MM)]
    if valid.size == 0:
        return None

    return int(np.percentile(valid, percentile))


def depth_only_nearest_mm(depth_mm: np.ndarray, roi_half: int = ROI_HALF,
                          percentile: float = ROI_PERCENTILE) -> tuple[int | None, tuple]:
    """Nearest obstacle ahead from a central ROI. Returns (mm, roi_rect)."""
    h, w = depth_mm.shape
    cx, cy = w // 2, h // 2
    x0 = max(0, cx - roi_half)
    x1 = min(w, cx + roi_half)
    y0 = max(0, cy - roi_half)
    y1 = min(h, cy + roi_half)
    roi = depth_mm[y0:y1, x0:x1]

    valid = roi[(roi >= MIN_MM) & (roi <= MAX_MM)]
    if valid.size == 0:
        return None, (x0, y0, x1, y1)

    mm = int(np.percentile(valid, percentile))
    return mm, (x0, y0, x1, y1)


def build_pipeline() -> dai.Pipeline:
    pipeline = dai.Pipeline()

    # Color camera (NN input + preview output)
    cam = pipeline.createColorCamera()
    cam.setPreviewSize(416, 416)  # yolov4-tiny-416
    cam.setInterleaved(False)
    cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

    # Mono + Stereo depth
    left = pipeline.createMonoCamera()
    right = pipeline.createMonoCamera()
    left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
    left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)   # 640x400
    right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

    stereo = pipeline.createStereoDepth()
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
    stereo.setLeftRightCheck(True)
    stereo.setSubpixel(True)

    left.out.link(stereo.left)
    right.out.link(stereo.right)

    # YOLO Detection Network (NOT spatial)
    yolo = pipeline.createYoloDetectionNetwork()
    yolo.setBlobPath(BLOB_PATH)
    yolo.setConfidenceThreshold(CONF_TH)
    yolo.setIouThreshold(IOU_TH)
    yolo.input.setBlocking(False)

    # YOLOv4-tiny COCO standard config
    yolo.setNumClasses(80)
    yolo.setCoordinateSize(4)
    yolo.setAnchors([10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319])
    yolo.setAnchorMasks({"side13": [3, 4, 5], "side26": [0, 1, 2]})

    cam.preview.link(yolo.input)

    # Outputs
    xout_det = pipeline.createXLinkOut()
    xout_det.setStreamName("detections")
    yolo.out.link(xout_det.input)

    xout_depth = pipeline.createXLinkOut()
    xout_depth.setStreamName("depth")
    stereo.depth.link(xout_depth.input)

    xout_rgb = pipeline.createXLinkOut()
    xout_rgb.setStreamName("rgb")
    cam.preview.link(xout_rgb.input)

    return pipeline


def main():
    pipeline = build_pipeline()

    print_period = 1.0 / max(0.1, PRINT_HZ)
    last_print = 0.0

    with dai.Device(pipeline) as device:
        q_det = device.getOutputQueue("detections", maxSize=4, blocking=False)
        q_depth = device.getOutputQueue("depth", maxSize=4, blocking=False)
        q_rgb = device.getOutputQueue("rgb", maxSize=4, blocking=True)

        latest_depth = None
        latest_dets = []

        print("✅ Live feed + detections + distance. Press 'q' in the window to quit.")

        while True:
            # Grab latest depth (if available)
            d = q_depth.tryGet()
            if d is not None:
                latest_depth = d.getFrame()  # uint16 mm

            # Grab latest detections (if available)
            det_msg = q_det.tryGet()
            if det_msg is not None:
                latest_dets = det_msg.detections

            # RGB frame (blocking so we display smoothly)
            frame = q_rgb.get().getCvFrame()
            fh, fw = frame.shape[:2]

            # Compute optional nearest-ahead
            nearest_ahead_str = None
            roi_rect = None
            if ENABLE_DEPTH_ONLY_NEAREST and latest_depth is not None:
                mm_roi, roi_rect = depth_only_nearest_mm(latest_depth)
                nearest_ahead_str = "N/A" if mm_roi is None else f"{mm_roi/1000.0:.2f}m"

            # Overlay detections (distance comes from depth mapped to depth frame size)
            closest = None  # (dist_m, label, conf)
            if latest_depth is not None:
                dh, dw = latest_depth.shape
            else:
                dh, dw = None, None

            for det in latest_dets[:DRAW_LIMIT]:
                # bbox on RGB frame (normalized coords)
                x0 = int(det.xmin * fw)
                y0 = int(det.ymin * fh)
                x1 = int(det.xmax * fw)
                y1 = int(det.ymax * fh)

                label_id = int(det.label)
                label = COCO[label_id] if 0 <= label_id < len(COCO) else str(label_id)

                dist_text = "dist=N/A"
                if latest_depth is not None:
                    # bbox mapped to depth frame size
                    dx0 = int(det.xmin * dw)
                    dy0 = int(det.ymin * dh)
                    dx1 = int(det.xmax * dw)
                    dy1 = int(det.ymax * dh)

                    mm = depth_stat_in_bbox_mm(latest_depth, dx0, dy0, dx1, dy1, percentile=DIST_PERCENTILE)
                    if mm is not None:
                        dist_m = mm / 1000.0
                        dist_text = f"dist={dist_m:.2f}m"
                        if closest is None or dist_m < closest[0]:
                            closest = (dist_m, label, float(det.confidence))

                # draw bbox + text
                cv2.rectangle(frame, (x0, y0), (x1, y1), (0, 255, 0), 2)
                txt = f"{label} {det.confidence:.2f} {dist_text}"
                cv2.putText(frame, txt, (x0, max(0, y0 - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            # draw ROI rectangle (where nearest-ahead is computed) projected onto RGB frame
            if roi_rect is not None and latest_depth is not None:
                # roi_rect is in depth coords; map to rgb coords
                dh, dw = latest_depth.shape
                rx0, ry0, rx1, ry1 = roi_rect
                fx0 = int(rx0 / dw * fw)
                fy0 = int(ry0 / dh * fh)
                fx1 = int(rx1 / dw * fw)
                fy1 = int(ry1 / dh * fh)
                cv2.rectangle(frame, (fx0, fy0), (fx1, fy1), (255, 0, 0), 2)
                if nearest_ahead_str is not None:
                    cv2.putText(frame, f"Nearest ahead: {nearest_ahead_str}",
                                (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                (255, 255, 255), 2)

            if SHOW_VIDEO:
                cv2.imshow(WINDOW_NAME, frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break

            # Terminal printing (rate-limited)
            now = time.time()
            if now - last_print >= print_period:
                last_print = now

                header = f"[{time.strftime('%H:%M:%S')}] dets={len(latest_dets)}"
                if nearest_ahead_str is not None:
                    header += f" | nearest_ahead={nearest_ahead_str}"
                print("\n" + header)

                if closest:
                    print(f"  >>> Closest detected: {closest[1]}  dist={closest[0]:.2f}m  conf={closest[2]:.2f}")
                elif latest_dets:
                    print("  >>> Closest detected: N/A (no valid depth in bbox)")
                else:
                    print("  No detections.")

        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
