import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/kylewong/Senior_Design/Vehicle/ros2_ws/install/car_description'
