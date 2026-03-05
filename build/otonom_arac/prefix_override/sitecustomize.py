import sys
if sys.prefix == '/opt/homebrew/opt/python@3.11/Frameworks/Python.framework/Versions/3.11':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/Users/faikaktss/ISA_TEAM_2025-2026_ROS2/install/otonom_arac'
