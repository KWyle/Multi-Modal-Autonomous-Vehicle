import math
import struct
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Float32


LS_TYPE = 'sensor_msgs/msg/LaserScan'
PC2_TYPE = 'sensor_msgs/msg/PointCloud2'


# PointCloud2 min-distance
def pc2_min_distance(pc: PointCloud2) -> float:
    """Compute minimum distance from PointCloud2 without numpy."""
    try:
        x_off = next(f.offset for f in pc.fields if f.name == 'x')
        y_off = next(f.offset for f in pc.fields if f.name == 'y')
        z_off = next(f.offset for f in pc.fields if f.name == 'z')
    except StopIteration:
        return math.inf

    m = math.inf
    step = pc.point_step
    data = pc.data

    for i in range(0, len(data), step):
        try:
            x = struct.unpack_from('<f', data, i + x_off)[0]
            y = struct.unpack_from('<f', data, i + y_off)[0]
            z = struct.unpack_from('<f', data, i + z_off)[0]
        except struct.error:
            continue

        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            d = math.sqrt(x*x + y*y + z*z)
            if d < m:
                m = d

    return m


# LaserScan min-distance
def laserscan_min_distance(msg: LaserScan) -> float:
    vals = [r for r in msg.ranges if math.isfinite(r)]
    return min(vals) if vals else math.inf


class UltrasonicMonitor(Node):
    def __init__(self):
        super().__init__('ultrasonic_monitor')

        # Parameters
        self.declare_parameter('left_topic',  '/ultrasonic/left/scan')
        self.declare_parameter('right_topic', '/ultrasonic/right/scan')
        self.declare_parameter('print_period_s', 0.5)

        self.left_topic  = self.get_parameter('left_topic').value
        self.right_topic = self.get_parameter('right_topic').value

        # Distances
        self.left_dist  = math.inf
        self.right_dist = math.inf

        # Publishers
        self.pub_left_clean  = self.create_publisher(Float32, '/ultrasonic/left/distance', 10)
        self.pub_right_clean = self.create_publisher(Float32, '/ultrasonic/right/distance', 10)

        # Subscriptions created dynamically
        self.left_sub  = None
        self.right_sub = None

        # Timers
        period = float(self.get_parameter('print_period_s').value)
        self.create_timer(period, self._print_status)
        self.create_timer(0.25, self._ensure_subscriptions)

        self.get_logger().info("Ultrasonic monitor started.")

    # Create subscriptions once topics appear in ROS graph
    def _ensure_subscriptions(self):
        names_and_types = dict(self.get_topic_names_and_types())
        qos = qos_profile_sensor_data

        # LEFT
        if self.left_sub is None and self.left_topic in names_and_types:
            types = names_and_types[self.left_topic]

            if LS_TYPE in types:
                self.left_sub = self.create_subscription(
                    LaserScan, self.left_topic, self._left_ls_cb, qos)
                self.get_logger().info(f"LEFT: Subscribed as LaserScan")

            elif PC2_TYPE in types:
                self.left_sub = self.create_subscription(
                    PointCloud2, self.left_topic, self._left_pc2_cb, qos)
                self.get_logger().info(f"LEFT: Subscribed as PointCloud2")

            else:
                self.get_logger().warn(f"LEFT topic type unsupported: {types}")

        # RIGHT
        if self.right_sub is None and self.right_topic in names_and_types:
            types = names_and_types[self.right_topic]

            if LS_TYPE in types:
                self.right_sub = self.create_subscription(
                    LaserScan, self.right_topic, self._right_ls_cb, qos)
                self.get_logger().info(f"RIGHT: Subscribed as LaserScan")

            elif PC2_TYPE in types:
                self.right_sub = self.create_subscription(
                    PointCloud2, self.right_topic, self._right_pc2_cb, qos)
                self.get_logger().info(f"RIGHT: Subscribed as PointCloud2")

            else:
                self.get_logger().warn(f"RIGHT topic type unsupported: {types}")

    # Callbacks
    def _left_ls_cb(self, msg: LaserScan):
        self.left_dist = laserscan_min_distance(msg)
        self._publish_clean_left()

    def _right_ls_cb(self, msg: LaserScan):
        self.right_dist = laserscan_min_distance(msg)
        self._publish_clean_right()

    def _left_pc2_cb(self, msg: PointCloud2):
        self.left_dist = pc2_min_distance(msg)
        self._publish_clean_left()

    def _right_pc2_cb(self, msg: PointCloud2):
        self.right_dist = pc2_min_distance(msg)
        self._publish_clean_right()

    # Clean outputs for fusion
    def _publish_clean_left(self):
        msg = Float32()
        msg.data = float(self.left_dist)
        self.pub_left_clean.publish(msg)

    def _publish_clean_right(self):
        msg = Float32()
        msg.data = float(self.right_dist)
        self.pub_right_clean.publish(msg)

    # Debug print timer
    def _print_status(self):
        l = f"{self.left_dist:.2f}" if math.isfinite(self.left_dist) else "inf"
        r = f"{self.right_dist:.2f}" if math.isfinite(self.right_dist) else "inf"
        self.get_logger().info(f"[Ultrasonic] Left: {l} m | Right: {r} m")


def main():
    rclpy.init()
    node = UltrasonicMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
