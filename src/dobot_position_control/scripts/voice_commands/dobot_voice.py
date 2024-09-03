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
    


    # Rotation Matrix - Robot to Camera frames
    R_m = np.array(
        [
            [0, 0, 1],
            [1, 0, 0],
            [0, 1, 0]
        ]
    )

    # elevator offset
    elevator_buttons_offset = {
        "none": [0, 0],
        # "-1": [35, -80],
        "-1": [35, -40],
        "first": [35, 0],
        "second": [35, 40],
        "third": [35, 80],
        "fourth": [35, 120]
    }

    # Origin Vector from Dobot to Camera
    P_robot_to_camera = np.array([-8, -12, 9.5]) # y = -12

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

    dobot_x_offset, dobot_y_offset, dobot_z_offset = 45, -80, 0
    r_rot = ((dobot_y_offset + move_to_y + elevator_buttons_offset[button][0])/10) * 1.25
    
    print('rot', r_rot)
    r_rot *= -1
    print('rot2', r_rot)

    
    device.move_to(dobot_x_offset + move_to_x, dobot_y_offset + move_to_y + elevator_buttons_offset[button][0], move_to_z + dobot_z_offset + elevator_buttons_offset[button][1], r_rot)
    time.sleep(1.5) # delay time: 1.5 s
    device.move_to(120, 0, 0, 0)
    
    # Signal that processing is complete
    event.clear()


def listener():
    rospy.init_node('tf_listener', anonymous=True)
    port = list_ports.comports()[0].device
    device = Dobot(port=port)
    
    from threading import Event
    event = Event()
    last_action = None
    while not rospy.is_shutdown():
        device.suck(True)
        # button = input('Enter direction \n\t4\n\t3\n\t2\n\t1\n\t-1\nquit\n: ')
        with open("action.txt", "r") as f:
            action = f.readline().strip()
        
        if action != last_action:


            if "quit" in action:
                device.suck(False)
                print("Quitting")
                break
            elif "first" in action:
                button = "first"
            elif "second" in action:
                button = "second"
            elif button not in ["none", "-1", "1", "2", "3", "4", "first", "second", "third", "fourth"]:
                print("Invalid input. Please enter 'up', 'down', or 'none'.")
                continue

            event.set()

            # rospy.Subscriber("/tf", TFMessage, callback(data, device), queue_size=10)
            sub = rospy.Subscriber("/tf", TFMessage, lambda data: callback(data, device, button, event), queue_size=10)
            rospy.wait_for_message("/tf", TFMessage)
            sub.unregister()
            last_action = action
        else:
            continue

if __name__ == '__main__':
    listener()

