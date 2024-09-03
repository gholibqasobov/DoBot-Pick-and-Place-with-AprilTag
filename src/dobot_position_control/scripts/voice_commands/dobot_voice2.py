#!/usr/bin/env python3

"""Finished code"""

import rospy
from tf2_msgs.msg import TFMessage
import numpy as np
from serial.tools import list_ports
import time
from pydobot import Dobot
import speech_recognition as sr
from gtts import gTTS
import os
from playsound import playsound


def callback(data, device, button, event):
    print('received data')
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
        "1": [35, 0],
        "2": [35, 40],
        "3": [35, 80],
        "4": [35, 120]
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

language = 'en'

def speak(text, lang='en'):
    """Convert text to speech and play it."""
    # Check if the file exists and remove it before creating a new one
    filename = "dobot_voice.mp3"
    if os.path.exists(filename):
        os.remove(filename)
    
    tts = gTTS(text=text, lang=lang, slow=False)
    tts.save(filename)
    playsound(filename)
    os.remove(filename)

# Initialize the recognizer
r = sr.Recognizer()

def listener():
    rospy.init_node('tf_listener', anonymous=True)
    port = list_ports.comports()[0].device
    device = Dobot(port=port)
    
    from threading import Event
    event = Event()

    while not rospy.is_shutdown():
        device.suck(True)
        # button = input('Enter direction \n\t4\n\t3\n\t2\n\t1\n\t-1\nquit\n: ')

            # use microphone as source for input
        text = None
        with sr.Microphone() as source:
            # Adjust for ambient noise here
            r.adjust_for_ambient_noise(source, duration=0.2)
            print('Select floor')
            speak('select floor', language)
            audio = r.listen(source, timeout=7, phrase_time_limit=7)
            print('listented successfully')
            try:
                print('processing')
                text = r.recognize_google(audio)
                    

                # response_text = text 
                print('text: ', text)
                # speak(text, language) 


            except sr.UnknownValueError:
                speak("I could not understand you", language)
                continue
            except sr.RequestError as e:
                print("Could not request results from Google Speech Recognition service; {0}".format(e))
                continue

        
        
        if "quit" in text or "stop" in text:
            device.suck(False)
            print("Quitting")
            speak('Quitting program', language)
            break
        elif "first" in text:
            speak("Selecting first floor", language)
            button = "1"
        elif "second" in text:
            speak("Selecting second floor", language)
            button = "2"
        elif "third" in text:
            speak("Selecting third floor", language)
            button = "3"
        elif "fourth" in text:
            speak("Selecting fourth floor", language)
            button = "4"
        elif "basement" in text:
            speak("Selecting basement floor", language)
            button = "-1"
        else:
            # button = None
            speak("Please choose a valid floor", language)

        event.set()

        # rospy.Subscriber("/tf", TFMessage, callback(data, device), queue_size=10)
        sub = rospy.Subscriber("/tf", TFMessage, lambda data: callback(data, device, button, event), queue_size=10)
        rospy.wait_for_message("/tf", TFMessage)
        sub.unregister()

if __name__ == '__main__':
    listener()

