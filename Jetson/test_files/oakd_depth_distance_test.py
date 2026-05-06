import depthai as dai
import numpy as np

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
    print("✅ Printing center ROI distance (mm). Ctrl+C to stop.")

    while True:
        depth = q.get().getFrame().astype(np.int32)  # mm

        h, w = depth.shape
        roi = depth[h//2-20:h//2+20, w//2-20:w//2+20]
        roi = roi[roi > 0]  # remove invalid zeros

        if roi.size == 0:
            print("No valid depth in ROI")
            continue

        dist_mm = int(np.median(roi))
        print("Distance (median ROI):", dist_mm, "mm")
