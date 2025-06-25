import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rokey/github_package/turtlebot4-slam-nav/install/rokey_pjt'
