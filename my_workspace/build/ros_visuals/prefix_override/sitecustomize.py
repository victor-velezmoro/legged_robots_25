import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/workspaces/ros2_ws/my_workspace/install/ros_visuals'
