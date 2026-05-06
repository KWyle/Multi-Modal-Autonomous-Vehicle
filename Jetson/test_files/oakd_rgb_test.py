import depthai as dai
import cv2

pipeline = dai.Pipeline()

cam = pipeline.createColorCamera()
cam.setPreviewSize(640, 400)
cam.setInterleaved(False)
cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

xout = pipeline.createXLinkOut()
xout.setStreamName("rgb")
cam.preview.link(xout.input)

with dai.Device(pipeline) as device:
    q = device.getOutputQueue("rgb", maxSize=4, blocking=True)
    print("✅ OAK-D RGB streaming. Press q to quit.")
    while True:
        frame = q.get().getCvFrame()
        cv2.imshow("OAK-D RGB", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
