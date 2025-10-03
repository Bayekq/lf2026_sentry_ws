import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/luojiafox/workspace/lf2026_sentry_ws/install/publish_anything_ros2_gui'
