import os
import json
from typing import List

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String

import numpy as np
import cv2

from ultralytics import YOLO
import torch
import torch.serialization as ts
from ultralytics.nn.tasks import DetectionModel  
import torch.nn as nn
from ultralytics.nn.modules.block import C3k2   

# Force torch.load
if 'weights_only' in torch.load.__code__.co_varnames:
    _orig_torch_load = torch.load

    def _torch_load_unsafe(*args, **kwargs):
        kwargs.setdefault('weights_only', False)
        return _orig_torch_load(*args, **kwargs)

    torch.load = _torch_load_unsafe

try:
    ts.add_safe_globals([
        DetectionModel, 
        nn.Sequential, 
        C3k2
    ])
except Exception:
    # Older PyTorch may not have add_safe_globals; ignore.
    pass

class InferenceNode(Node):
    def __init__(self) -> None:
        super().__init__('inference_node')

        # Parameters
        self.declare_parameter('image_topic', '/oak/color/image_raw')
        self.declare_parameter('model_path', '')
        self.declare_parameter('conf_thresh', 0.25)
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('publish_json', True)

        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.conf_thresh = float(self.get_parameter('conf_thresh').value)
        self.publish_annotated = bool(self.get_parameter('publish_annotated').value)
        self.publish_json = bool(self.get_parameter('publish_json').value)

        if not self.model_path:
            from ament_index_python.packages import get_package_share_directory
            pkg_share = get_package_share_directory('perception_inference')
            self.model_path = os.path.join(pkg_share, 'models', 'yolo12n.pt')

        self.get_logger().info(f'Loading model: {self.model_path}')

        # Load YOLO model (CPU), I have AMD GPU so cuda() won't work
        self.model = YOLO(self.model_path)

        # Publishers
        self.pub_img = self.create_publisher(Image, '/inference/image_marked', 10)
        self.pub_json = self.create_publisher(String, '/inference/detections', 10)

        # Subscriber
        self.sub_img = self.create_subscription(Image, self.image_topic, self._image_cb, 10)

        # CV bridge-less converters
        self._encoding = 'bgr8'  # Gazebo camera via libgazebo_ros_camera
        self.get_logger().info(
            f'Subscribed to {self.image_topic} | conf_thresh={self.conf_thresh:.2f}'
        )

    # helpers
    def _rosimg_to_cv2(self, msg: Image) -> np.ndarray:
        """Convert ROS Image to OpenCV BGR ndarray."""
        # Support common encodings
        if msg.encoding.lower() in ('bgr8', 'bgra8'):
            dtype = np.uint8
            nchan = 3 if msg.encoding.lower() == 'bgr8' else 4
            img = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width, nchan)
            if nchan == 4:
                img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
            return img
        elif msg.encoding.lower() in ('rgb8', 'rgba8'):
            dtype = np.uint8
            nchan = 3 if msg.encoding.lower() == 'rgb8' else 4
            img = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width, nchan)
            if nchan == 4:
                img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
            else:
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            return img
        else:
            # Fallback: treat as mono8
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
            return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    def _cv2_to_rosimg(self, img_bgr: np.ndarray, stamp, frame_id: str) -> Image:
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height, msg.width = img_bgr.shape[:2]
        msg.encoding = 'bgr8'
        msg.step = msg.width * 3
        msg.data = img_bgr.tobytes()
        return msg

    # callback
    def _image_cb(self, msg: Image) -> None:
        img_bgr = self._rosimg_to_cv2(msg)
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)

        results = self.model.predict(
            img_rgb, conf=self.conf_thresh, verbose=False
        )
        res = results[0]

        # Build JSON-friendly detections
        dets: List[dict] = []
        if res.boxes is not None and len(res.boxes) > 0:
            boxes = res.boxes.xyxy.cpu().numpy()
            confs = res.boxes.conf.cpu().numpy()
            clss  = res.boxes.cls.cpu().numpy().astype(int)
            names = self.model.model.names if hasattr(self.model, 'model') else {}

            for (x1, y1, x2, y2), c, k in zip(boxes, confs, clss):
                label = names.get(k, str(k))
                dets.append({
                    'class_id': int(k),
                    'label': label,
                    'confidence': float(c),
                    'bbox_xyxy': [float(x1), float(y1), float(x2), float(y2)],
                })
                # Draw on image
                p1, p2 = (int(x1), int(y1)), (int(x2), int(y2))
                cv2.rectangle(img_bgr, p1, p2, (0, 255, 0), 2)
                txt = f'{label} {c:.2f}'
                cv2.putText(img_bgr, txt, (p1[0], max(0, p1[1] - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Publish annotated image
        if self.publish_annotated:
            out_img = self._cv2_to_rosimg(img_bgr, msg.header.stamp, msg.header.frame_id)
            self.pub_img.publish(out_img)

        # Publish detections JSON
        if self.publish_json:
            s = String()
            s.data = json.dumps({'detections': dets})
            self.pub_json.publish(s)


def main() -> None:
    rclpy.init()
    try:
        node = InferenceNode()
        rclpy.spin(node)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
