import depthai as dai
import cv2
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
    print("✅ OAK-D depth streaming. Press q to quit.")
    while True:
        depth = q.get().getFrame()  # uint16 depth in millimeters

        # visualize
        depth_vis = (depth / np.max(depth) * 255).astype(np.uint8)
        depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)

        cv2.imshow("OAK-D Depth", depth_vis)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
