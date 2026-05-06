#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from vision_msgs.msg import Detection2DArray


class OakDetectionDistanceNode(Node):
    def __init__(self):
        super().__init__('oak_detection_distance')

        # Parameters
        self.declare_parameter('depth_topic', '/oak/depth/oak_depth/depth/image_raw')
        self.declare_parameter('detections_topic', '/inference/detections')

        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        detections_topic = self.get_parameter('detections_topic').get_parameter_value().string_value

        # Latest center depth straight ahead
        self.center_distance = None

        self.detected_label = None

        # Publisher for other nodes
        self.front_distance_pub = self.create_publisher(Float32, 'front_distance', 10)

        # Subscriptions
        self.create_subscription(Image, depth_topic, self._depth_cb, 10)
        self.create_subscription(Detection2DArray, detections_topic, self._detections_cb, 10)

        # 1 Hz summary timer
        self.create_timer(1.0, self._publish_summary)

        self.get_logger().info(
            "OakDetectionDistanceNode running (depth + detections)\n"
            f"  depth_topic      = {depth_topic}\n"
            f"  detections_topic = {detections_topic}\n"
            f"  summary rate     = 1 Hz"
        )

    # Depth callback
    def _depth_cb(self, depth_msg: Image):
        """Update straight-ahead distance from the depth center pixel."""
        if depth_msg.encoding not in ('32FC1', 'TYPE_32FC1'):
            return

        try:
            depth_arr = np.frombuffer(depth_msg.data, dtype=np.float32).reshape(
                depth_msg.height, depth_msg.width
            )
        except Exception:
            return

        h, w = depth_arr.shape
        cy = h // 2
        cx = w // 2

        center_depth = float(depth_arr[cy, cx])

        if not np.isfinite(center_depth) or center_depth <= 0.0:
            return

        self.center_distance = center_depth

    # Detections callback
    def _detections_cb(self, msg: Detection2DArray):
        """
        When YOLO sees something, remember its label.
        We'll pair this label with the depth-based center distance.
        """
        if not msg.detections:
            # No detections this frame
            self.detected_label = None
            return

        # Just take the first detection for now
        det = msg.detections[0]

        label = None
        if det.results:
            label = det.results[0].hypothesis.class_id
        if not label:
            label = "unknown"

        self.detected_label = label

    #  1 Hz summary publisher
    def _publish_summary(self):
        """Once per second, print depth distance, with label if we have one."""
        if self.center_distance is None:
            return

        # Publish numeric distance
        msg = Float32(data=self.center_distance)
        self.front_distance_pub.publish(msg)

        if self.detected_label:
            self.get_logger().info(
                f"Closest detection: {self.detected_label} at {self.center_distance:.2f} m"
            )
        else:
            self.get_logger().info(
                f"Closest distance in front: {self.center_distance:.2f} m"
            )


def main(args=None):
    rclpy.init(args=args)
    node = OakDetectionDistanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
