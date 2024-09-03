#!/usr/bin/env python3
import rospy
from tf2_msgs.msg import TFMessage
import numpy as np
from serial.tools import list_ports
import time
from dobot_extensions import Dobot


def callback(data, device, event):

    if not event.is_set():
        return

    """Rotation Matrix - Robot to Camera frames"""
    R_m = np.array(
        [
            [0, -1, 0],
            [-1, 0, 0],
            [0, 0, -1]
        ]
    )

    """Origin Vector from Dobot to Camera"""
    P_robot_to_camera = np.array([12.5, 0, 44.5]) # z = 44.5

    """Extract AprilTag translation"""
    trans = data.transforms[0].transform.translation
    P_apriltag = np.array([trans.x, trans.y, trans.z]) * 100


    """Extracts multiple AptilTag translations"""
    # for transform in data.transforms:
    #     # Extract AprilTag translation
    #     trans = transform.transform.translation

    #     # Position of AprilTag with respect to camera
    #     P_apriltag = np.array([trans.x, trans.y, trans.z]) * 100

    """Frame transformation: calculations"""
    P_robot_to_apriltag = P_robot_to_camera + np.dot(R_m, P_apriltag) 
    P_robot_to_apriltag = P_robot_to_apriltag * 10  
    
    """Position of AprilTag with respect to Dobot"""
    print(P_robot_to_apriltag)
    move_to_x, move_to_y, move_to_z = P_robot_to_apriltag

    """Move Dobot to AprilTag"""

    pose = device.get_pose()
    position = pose.position
    dobot_x_offset = 120 
    dobot_y_offset = 30
    dobot_z_offset = -52

    y_const = 0.115
    neg_y_const = 0.15
    if move_to_y < -70:
        dobot_z_offset = dobot_z_offset + y_const * abs(move_to_y)
    elif move_to_y > 0:
          dobot_z_offset = dobot_z_offset - neg_y_const * abs(move_to_y)

    if 70 > move_to_y < -70:
        if move_to_x < 80:
            x_const = 0.1
        elif move_to_x < 50:
            x_const = 0.18
        elif move_to_x < 35:
            x_const = 0.25
        else:
            x_const = 0
        dobot_z_offset = dobot_z_offset - x_const * move_to_x 

    # if move_to_x + dobot_x_offset < 165:
    #     dobot_z_offset -= 5

    # if move_to_y < -70:
    #     dobot_z_offset = -30
    # else:
    #     dobot_z_offset = -50

    # if move_to_x + dobot_x_offset < 175:
    #     dobot_z_offset -= 10
    # elif move_to_x + dobot_x_offset > 265:
    #     dobot_z_offset += 5

    # if move_to_y < 0:
    #     # dobot_z_offset -= 5
    #     if move_to_x + dobot_x_offset > 200:
    #         dobot_z_offset -= 15
    #     elif move_to_x + dobot_x_offset < 170:
    #         dobot_z_offset -= 25
    #     else:
    #         dobot_z_offset -= 15
    # elif move_to_y > 0:
    #     if move_to_x + dobot_x_offset> 200:
    #         dobot_z_offset -= 15
    #     elif move_to_x + dobot_x_offset < 170:
    #         dobot_z_offset -= 25

    """robot movement part"""
    device.move_to(dobot_x_offset + move_to_x, dobot_y_offset + move_to_y, move_to_z + dobot_z_offset, position.r)
    time.sleep(1.5)
    device.suck(True)
    time.sleep(1.5)
    device.move_to(dobot_x_offset + move_to_x, dobot_y_offset + move_to_y, 48, position.r)
    time.sleep(1.5)
    device.move_to(82.67, -203.839, 48, position.r)
    time.sleep(1.5)
    device.suck(False)
    # time.sleep(1.5)
    device.move_to(120, 0, 0, position.r)
    device.conveyor_belt_distance(50, 100, 1, 0)

    time.sleep(1.5)
    event.clear()

    



def listener():
    rospy.init_node('tf_listener', anonymous=True)

    """Connect the Dobot"""
    port = list_ports.comports()[0].device
    device = Dobot(port=port)

    from threading import Event
    event = Event()

    while not rospy.is_shutdown():
        event.set()
        sub = rospy.Subscriber("/tf", TFMessage, lambda data: callback(data, device, event), queue_size=10)
        rospy.wait_for_message("/tf", TFMessage)
        sub.unregister()


    # rospy.Subscriber("/tf", TFMessage, callback(data, device), queue_size=10)
    # rospy.Subscriber("/tf", TFMessage, lambda data: callback(data, device), queue_size=10)
    # rospy.spin()

if __name__ == '__main__':
    listener()

