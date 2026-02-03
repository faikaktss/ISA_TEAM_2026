import sys
if sys.prefix == '/opt/homebrew/Caskroom/miniforge/base/envs/isa_env':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/Users/faikaktss/ISA_TEAMM_2025-2026/ros2_ws/install/otonom_arac'
