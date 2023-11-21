#! /usr/bin/env python3

import pylink
import time

jlink = pylink.JLink()
jlink.open()
rst = jlink.set_tif(pylink.enums.JLinkInterfaces.SWD)

if (rst == True):
	print("Set Jlink SWD success\r")
else:
	print("Set Jlink SWD fail\r")
	jlink.close()

jlink.connect('ING9188xx')

rst = jlink.target_connected()

if (rst == True):
	print("Target connect success\r")
else:
	print("Target connect fail\r")
	jlink.close()

file_path = './build/zephyr/zephyr.hex'

jlink.flash_file(file_path, 0x24000)
jlink.reset()

print(time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()), end=" ")
print(file_path, "Flash done, close Jlink")
jlink.close()