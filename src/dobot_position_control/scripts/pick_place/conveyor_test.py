from serial.tools import list_ports

from dobot_extensions import Dobot

port = list_ports.comports()[0].device # connect to the device
device = Dobot(port=port)

device.conveyor_belt_distance(50, 100, 1, 0) # move the conveyer

device.close()