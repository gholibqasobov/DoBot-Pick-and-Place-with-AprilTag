#!/usr/bin/env python3

from serial.tools import list_ports
import time
from pydobot import Dobot


port = list_ports.comports()[0].device

device = Dobot(port=port)

pose = device.get_pose()

# print(pose)

position = pose.position
print(position)
# device.conveyor_belt_distance(50, 100, 1, 0)
# print(help(device))
device.move_to(120, 0, 0, 0) # -150 to 150
# device.suck(True)
# time.sleep(3)
# device.suck(False)
# device.move_to(208, 150, 17.1, 0)
device.close()



