#! /usr/bin/env python3

import pylink
import time
import sys
if len(sys.argv) != 2:
	print("Usage: python script.py <file_path>")
file_path = sys.argv[1]
# 在这里可以使用 file_path 进行后续操作
print("文件路径为:", file_path)

jlink = pylink.JLink()
jlink.open()
rst = jlink.set_tif(pylink.enums.JLinkInterfaces.SWD)

if (rst == True):
	print("Set Jlink SWD success\r")
else:
	print("Set Jlink SWD fail\r")
	jlink.close()

jlink.connect('ING9168xx')

rst = jlink.target_connected()

if (rst == True):
	print("Target connect success\r")
else:
	print("Target connect fail\r")
	jlink.close()

jlink.flash_file(file_path, 0x2027000)
jlink.reset()

print(time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()), end=" ")
print(file_path, "Flash done, close Jlink")
jlink.close()