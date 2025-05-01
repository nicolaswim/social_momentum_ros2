import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/wim/Documents/social_momentum_ros2/install/my_social_nav_pkg'
