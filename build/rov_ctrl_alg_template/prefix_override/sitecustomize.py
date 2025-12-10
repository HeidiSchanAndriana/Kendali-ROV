import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/heidischan/rov_ws/install/rov_ctrl_alg_template'
