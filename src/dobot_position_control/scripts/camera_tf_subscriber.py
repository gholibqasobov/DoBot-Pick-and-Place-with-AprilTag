#!/usr/bin/env python3
import rospy
from tf2_msgs.msg import TFMessage
import numpy as np
from serial.tools import list_ports
import time
from pydobot import Dobot


def callback(data, device):

    # Rotation Matrix - Robot to Camera frames
    R_m = np.array(
        [
            [0, -1, 0],
            [-1, 0, 0],
            [0, 0, -1]
        ]
    )

    # Origin Vector from Dobot to Camera
    P_robot_to_camera = np.array([12.5, 0, 44.5]) # z = 44.5

    # Extract AprilTag translation
    trans = data.transforms[0].transform.translation
    P_apriltag = np.array([trans.x, trans.y, trans.z]) * 100

    # for transform in data.transforms:
    #     # Extract AprilTag translation
    #     trans = transform.transform.translation

    #     # Position of AprilTag with respect to camera
    #     P_apriltag = np.array([trans.x, trans.y, trans.z]) * 100

    # Frame transformation: calculations
    P_robot_to_apriltag = P_robot_to_camera + np.dot(R_m, P_apriltag) 
    P_robot_to_apriltag = P_robot_to_apriltag * 10  
    
    # Position of AprilTag with respect to Dobot
    print(P_robot_to_apriltag)
    move_to_x, move_to_y, move_to_z = P_robot_to_apriltag

    # Move Dobot to AprilTag

    pose = device.get_pose()
    position = pose.position
    dobot_x_offset = 120 + 25
    dobot_y_offset = 40 + 10
    device.move_to(dobot_x_offset + move_to_x, dobot_y_offset + move_to_y, move_to_z, position.r)

    



def listener():
    rospy.init_node('tf_listener', anonymous=True)

    # Connect the Dobot
    port = list_ports.comports()[0].device
    device = Dobot(port=port)

    # rospy.Subscriber("/tf", TFMessage, callback(data, device), queue_size=10)
    rospy.Subscriber("/tf", TFMessage, lambda data: callback(data, device), queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    listener()

