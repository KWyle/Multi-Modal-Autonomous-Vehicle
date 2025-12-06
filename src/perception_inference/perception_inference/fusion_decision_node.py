#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String


class FusionDecisionNode(Node):
    def __init__(self):
        super().__init__('fusion_decision')

        # --- Thresholds (meters) ---
        self.declare_parameter('front_stop_distance', 2.0)   # hard stop distance
        self.declare_parameter('front_slow_distance', 4.0)   # start slowing distance
        self.declare_parameter('side_avoid_distance', 0.6)   # side too-close distance

        self.front_stop_distance = self.get_parameter(
            'front_stop_distance').get_parameter_value().double_value
        self.front_slow_distance = self.get_parameter(
            'front_slow_distance').get_parameter_value().double_value
        self.side_avoid_distance = self.get_parameter(
            'side_avoid_distance').get_parameter_value().double_value

        # Latest readings (meters)
        self.front_dist = None
        self.left_dist = None
        self.right_dist = None

        # OAK front distance (from OakDetectionDistanceNode)
        self.create_subscription(
            Float32,
            '/front_distance',
            self.front_cb,
            10
        )

        # UltrasonicMonitor cleaned outputs
        self.create_subscription(
            Float32,
            '/ultrasonic/left/distance',
            self.left_cb,
            10
        )
        self.create_subscription(
            Float32,
            '/ultrasonic/right/distance',
            self.right_cb,
            10
        )

        self.action_pub = self.create_publisher(String, '/desired_action', 10)

        # Run fusion at 1 Hz
        self.timer = self.create_timer(1.0, self.evaluate_action)

        self.get_logger().info('FusionDecisionNode running (OAK + ultrasonics).')

    # Callbacks
    def front_cb(self, msg: Float32):
        self.front_dist = float(msg.data)

    def left_cb(self, msg: Float32):
        self.left_dist = float(msg.data)

    def right_cb(self, msg: Float32):
        self.right_dist = float(msg.data)

    # Helper
    @staticmethod
    def _norm_dist(d):
        if d is None or not math.isfinite(d):
            return float('inf')
        return float(d)

    # Core fusion logic
    def evaluate_action(self):
        # If any sensor hasn't reported yet, print a waiting message
        if self.front_dist is None or self.left_dist is None or self.right_dist is None:
            self.get_logger().info(
                f'[fusion] waiting for sensors: '
                f'front={self.front_dist}, left={self.left_dist}, right={self.right_dist}'
            )
            return

        # Normalize distances
        f = self._norm_dist(self.front_dist)
        l = self._norm_dist(self.left_dist)
        r = self._norm_dist(self.right_dist)

        fs = self.front_stop_distance
        fslow = self.front_slow_distance
        sside = self.side_avoid_distance

        # 1) Emergency stop: obstacle very close ahead
        if f <= fs:
            action = 'FULL_STOP (OBSTACLE AHEAD)'

        # 2) Boxed in and close in front
        elif l <= sside and r <= sside and f <= fslow:
            action = 'STOP (BOXED IN)'

        # 3) Getting close in front: slow down (maybe with steering)
        elif f <= fslow:
            if l <= sside < r:
                action = 'SLOW_DOWN + STEER_RIGHT (LEFT TOO CLOSE)'
            elif r <= sside < l:
                action = 'SLOW_DOWN + STEER_LEFT (RIGHT TOO CLOSE)'
            else:
                action = 'SLOW_DOWN (FRONT GETTING CLOSE)'

        # 4) Front clear: use sides for lane keeping
        else:
            if l <= sside and r > sside:
                action = 'STEER_RIGHT (LEFT TOO CLOSE)'
            elif r <= sside and l > sside:
                action = 'STEER_LEFT (RIGHT TOO CLOSE)'
            elif l <= sside and r <= sside:
                action = 'GO_STRAIGHT_SLOW (NARROW CORRIDOR)'
            else:
                action = 'KEEP_SPEED (PATH CLEAR)'

        # Log fused decision once per second
        self.get_logger().info(
            f'[fusion] front={f:.2f} m, left={l:.2f} m, right={r:.2f} m -> {action}'
        )

        # Publish for downstream nodes
        msg = String()
        msg.data = action
        self.action_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FusionDecisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
            pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
