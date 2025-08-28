import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/arthurficial-intelligence/arthur_mecanum_ws/install/pid_tuner'
