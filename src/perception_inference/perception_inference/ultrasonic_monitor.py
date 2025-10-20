import math
import struct
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2

LS_TYPE = 'sensor_msgs/msg/LaserScan'
PC2_TYPE = 'sensor_msgs/msg/PointCloud2'

def pc2_min_distance(pc: PointCloud2) -> float:
    # unpack x,y,z from PointCloud2 (no numpy)
    try:
        x_off = next(f.offset for f in pc.fields if f.name == 'x')
        y_off = next(f.offset for f in pc.fields if f.name == 'y')
        z_off = next(f.offset for f in pc.fields if f.name == 'z')
    except StopIteration:
        return math.inf
    step = pc.point_step
    data = pc.data
    m = math.inf
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

def laserscan_min_distance(scan: LaserScan) -> float:
    m = math.inf
    for r in scan.ranges:
        if r is not None and math.isfinite(r) and r < m:
            m = r
    return m

class UltrasonicMonitor(Node):
    def __init__(self):
        super().__init__('ultrasonic_monitor')

        # params
        self.declare_parameter('left_topic',  '/ultrasonic/left/scan')
        self.declare_parameter('right_topic', '/ultrasonic/right/scan')
        self.declare_parameter('print_period_s', 0.5)

        self.left_topic  = self.get_parameter('left_topic').value
        self.right_topic = self.get_parameter('right_topic').value
        self.left_dist   = math.inf
        self.right_dist  = math.inf

        # subs start as None; we create them after we detect types
        self.left_sub  = None
        self.right_sub = None

        # timer to (a) create subs once topics are visible, (b) print distances
        period = float(self.get_parameter('print_period_s').value)
        self.create_timer(period, self._tick)
        self.create_timer(0.25, self._ensure_subscriptions)

    def _ensure_subscriptions(self):
        names_and_types = dict(self.get_topic_names_and_types())
        qos = qos_profile_sensor_data

        # LEFT
        if self.left_sub is None and self.left_topic in names_and_types:
            types = names_and_types[self.left_topic]
            if LS_TYPE in types:
                self.left_sub = self.create_subscription(
                    LaserScan, self.left_topic, self._left_ls_cb, qos)
                self.get_logger().info(f"Subscribed LEFT as LaserScan on {self.left_topic}")
            elif PC2_TYPE in types:
                self.left_sub = self.create_subscription(
                    PointCloud2, self.left_topic, self._left_pc_cb, qos)
                self.get_logger().info(f"Subscribed LEFT as PointCloud2 on {self.left_topic}")
            else:
                self.get_logger().warn(f"LEFT topic {self.left_topic} has unsupported type(s): {types}")

        # RIGHT
        if self.right_sub is None and self.right_topic in names_and_types:
            types = names_and_types[self.right_topic]
            if LS_TYPE in types:
                self.right_sub = self.create_subscription(
                    LaserScan, self.right_topic, self._right_ls_cb, qos)
                self.get_logger().info(f"Subscribed RIGHT as LaserScan on {self.right_topic}")
            elif PC2_TYPE in types:
                self.right_sub = self.create_subscription(
                    PointCloud2, self.right_topic, self._right_pc_cb, qos)
                self.get_logger().info(f"Subscribed RIGHT as PointCloud2 on {self.right_topic}")
            else:
                self.get_logger().warn(f"RIGHT topic {self.right_topic} has unsupported type(s): {types}")

    # callbacks for types for usage
    def _left_ls_cb(self, msg: LaserScan):    self.left_dist  = laserscan_min_distance(msg)
    def _right_ls_cb(self, msg: LaserScan):   self.right_dist = laserscan_min_distance(msg)
    def _left_pc_cb(self, msg: PointCloud2):  self.left_dist  = pc2_min_distance(msg)
    def _right_pc_cb(self, msg: PointCloud2): self.right_dist = pc2_min_distance(msg)

    def _tick(self):
        l = f"{self.left_dist:.2f} m"  if math.isfinite(self.left_dist)  else "inf"
        r = f"{self.right_dist:.2f} m" if math.isfinite(self.right_dist) else "inf"
        self.get_logger().info(f"[Ultrasonic] Left: {l} | Right: {r}")

def main():
    rclpy.init()
    rclpy.spin(UltrasonicMonitor())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
