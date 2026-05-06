import depthai as dai
import numpy as np
import time

def nearest_depth_m(depth_mm: np.ndarray, cx: int, cy: int, half: int = 30):
    h, w = depth_mm.shape
    x0, x1 = max(0, cx-half), min(w, cx+half)
    y0, y1 = max(0, cy-half), min(h, cy+half)
    roi = depth_mm[y0:y1, x0:x1]
    roi = roi[roi > 0]
    if roi.size == 0:
        return None
    return float(np.percentile(roi, 10)) / 1000.0  # 10th percentile = “closest” but stable

pipeline = dai.Pipeline()

left  = pipeline.createMonoCamera()
right = pipeline.createMonoCamera()
left.setBoardSocket(dai.CameraBoardSocket.LEFT)
right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

stereo = pipeline.createStereoDepth()
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
stereo.setLeftRightCheck(True)
stereo.setSubpixel(True)

left.out.link(stereo.left)
right.out.link(stereo.right)

xout = pipeline.createXLinkOut()
xout.setStreamName("depth")
stereo.depth.link(xout.input)

with dai.Device(pipeline) as device:
    q = device.getOutputQueue("depth", maxSize=4, blocking=True)
    print("✅ Nearest depth-only distance (meters). Ctrl+C to stop.")
    last = time.time()
    while True:
        depth = q.get().getFrame().astype(np.int32)  # mm
        h, w = depth.shape
        d_m = nearest_depth_m(depth, w//2, h//2, half=35)
        now = time.time()
        if now - last > 0.1:  # 10 Hz print
            print("nearest_m:", d_m)
            last = now
