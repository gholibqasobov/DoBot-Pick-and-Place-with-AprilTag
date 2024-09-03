#!/usr/bin/env python3
import rospy
from tf2_msgs.msg import TFMessage
import numpy as np
from serial.tools import list_ports
import time
from pydobot import Dobot


def callback(data, device, button, event):

    if not event.is_set():
        return


    # elevator buttons
    elevator_buttons_offset = {
        "none": [0, 0],
        "down": [33, 58],
        "up": [78, 58]
    }


    # Rotation Matrix - Robot to Camera frames
    R_m = np.array(
        [
            [0, -1, 0],
            [-1, 0, 0],
            [0, 0, -1]
        ]
    )

    # Origin Vector from Dobot to Camera
    P_robot_to_camera = np.array([12.5, 0, 44.5])

    # Extract AprilTag translation
    trans = data.transforms[0].transform.translation
    P_apriltag = np.array([trans.x, trans.y, trans.z]) * 100

    P_robot_to_apriltag = P_robot_to_camera + np.dot(R_m, P_apriltag) 
    P_robot_to_apriltag = P_robot_to_apriltag * 10  
    
    # Position of AprilTag with respect to Dobot
    print(P_robot_to_apriltag)
    move_to_x, move_to_y, move_to_z = P_robot_to_apriltag

    # Move Dobot to AprilTag

    pose = device.get_pose()
    position = pose.position

    dobot_x_offset, dobot_y_offset, dobot_z_offset = 120, 40, -50
    # dobot_x_offset, dobot_y_offset, dobot_z_offset = 120, 0, 0

    device.move_to(dobot_x_offset + move_to_x + elevator_buttons_offset[button][0], dobot_y_offset + move_to_y + elevator_buttons_offset[button][1], move_to_z + dobot_z_offset, position.r)
    time.sleep(1.5)
    device.move_to(120, 0, 0, position.r)
    
    # Signal that processing is complete
    event.clear()


def listener():
    rospy.init_node('tf_listener', anonymous=True)
    port = list_ports.comports()[0].device
    device = Dobot(port=port)
    from threading import Event
    event = Event()

    while not rospy.is_shutdown():
        button = input('Enter direction (up/down/none/quit): ')
        if button == "quit":
            print("Quitting")
            break
        elif button not in ["up", "down", "none"]:
            print("Invalid input. Please enter 'up', 'down', or 'none'.")
            continue

        event.set()

        # rospy.Subscriber("/tf", TFMessage, callback(data, device), queue_size=10)
        sub = rospy.Subscriber("/tf", TFMessage, lambda data: callback(data, device, button, event), queue_size=10)
        rospy.wait_for_message("/tf", TFMessage)
        sub.unregister()

if __name__ == '__main__':
    listener()

