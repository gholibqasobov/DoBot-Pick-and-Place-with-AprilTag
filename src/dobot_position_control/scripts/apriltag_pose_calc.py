# import numpy as np 

# R_m = np.array(
#     [
#         [0, -1, 0],
#         [-1, 0, 0],
#         [0, 0, -1]
#     ]
# )

# P_apriltag = np.array([-0.082, 0.022, 0.439]) * 100
# P_robot_to_camera = np.array([12.5, 0, 44.5])

# P_robot_to_apriltag = P_robot_to_camera + np.dot(R_m, P_apriltag) 
# P_robot_to_apriltag = P_robot_to_apriltag * 10
# print(P_robot_to_apriltag)

#!/usr/bin/env python3
import rospy
from tf2_msgs.msg import TFMessage
import numpy as np


def callback(data):

    # Rotation Matrix Robot to Camera frames
    R_m = np.array(
        [
            [0, 0, 1],
            [1, 0, 0],
            [0, 1, 0]
        ]
    )

    # Vector from Dobot to Camera
    P_robot_to_camera = np.array([-8, -12, 9.5])

    for transform in data.transforms:
        # Extract translation
        trans = transform.transform.translation

        # Position of AprilTag with respect to camera
        P_apriltag = np.array([trans.x, trans.y, trans.z]) * 100

    P_robot_to_apriltag = P_robot_to_camera + np.dot(R_m, P_apriltag) 
    P_robot_to_apriltag = P_robot_to_apriltag * 10  
    print(P_robot_to_apriltag)
    return P_robot_to_apriltag

def listener():
    rospy.init_node('tf_listener', anonymous=True)
    rospy.Subscriber("/tf", TFMessage, callback, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    listener()











