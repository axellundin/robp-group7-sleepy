import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sleepy/robp-group7-sleepy/src/core/install/odometry2'
