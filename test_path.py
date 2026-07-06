import sys
sys.path.insert(0, '/home/nvidia/gb_ws/sdk/genisom_l1_sdk-main/lib/zsl-1w/aarch64')

import gb_base_driver.real_base_adapter as ra
import os

print("_ws_root:", ra._ws_root)
print("_SDK_LIB_PATH:", ra._SDK_LIB_PATH)
print("SDK exists:", os.path.isdir(ra._SDK_LIB_PATH))
print("SO files:", os.listdir(ra._SDK_LIB_PATH))
