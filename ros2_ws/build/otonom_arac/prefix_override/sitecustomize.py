import sys
if sys.prefix == '/Library/Frameworks/Python.framework/Versions/3.14':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/Applications/PROJELERİM/ISA_TEAM_2025-2026_ROS2/ros2_ws/install/otonom_arac'
