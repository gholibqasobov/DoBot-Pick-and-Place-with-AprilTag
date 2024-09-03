

"""Speech to text converter with ability to save in .txt file"""

import speech_recognition as sr
from gtts import gTTS
import os
from playsound import playsound
import time 

language = 'en'

def speak(text, lang='en'):
    """Convert text to speech and play it."""
    # Check if the file exists and remove it before creating a new one
    filename = "temp_voice.mp3"
    if os.path.exists(filename):
        os.remove(filename)
    
    tts = gTTS(text=text, lang=lang, slow=False)
    tts.save(filename)
    playsound(filename)
    os.remove(filename)



# Initialize the recognizer
r = sr.Recognizer()

# while True:
try:
    # use microphone as source for input
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
            if 'stop' in text.lower():
                speak('Quitting program', language)
                

            # response_text = text 
            print('text: ', text)
            speak(text, language) 

            with open("action.txt", "w") as f:
                f.writelines(text)


        except sr.UnknownValueError:
            speak("I could not understand you", language)
        except sr.RequestError as e:
            print("Could not request results from Google Speech Recognition service; {0}".format(e))

except Exception as e:
    print("[ERROR] An error occurred: {}".format(e))


# print('waiting')
# time.sleep(3)






"""-------------------------------------------------------------------------------------------------------------------"""



# import speech_recognition as sr
# import time
# # Initialize the recognizer
# r = sr.Recognizer()


# while True:
#     try:
#         # use microphone as source for input
#         with sr.Microphone() as source:
#             r.adjust_for_ambient_noise(source, duration=0.2)
#             print("Say something:")
#             audio = r.listen(source)
#             try:
#                 text = r.recognize_google(audio)
#                 if text == 'stop':
#                     print('Quitting')
#                     break
#                 print("You said: {}".format(text))
#                 time.sleep(3)
#             except sr.UnknownValueError:
#                 print("Google Speech Recognition could not understand audio")
#             except sr.RequestError as e:
#                 print("Could not request results from Google Speech Recognition service; {0}".format(e))

#     except sr.UnknownValueError:
#         print("[ERROR] Unknown error occured")


# import speech_recognition as sr
# from gtts import gTTS
# import os
# from playsound import playsound
# import time 

# language = 'en'

# def speak(text, lang='en'):
#     """Convert text to speech and play it."""
#     # Check if the file exists and remove it before creating a new one
#     filename = "temp_voice.mp3"
#     if os.path.exists(filename):
#         os.remove(filename)
    
#     tts = gTTS(text=text, lang=lang, slow=False)
#     tts.save(filename)
#     playsound(filename)
#     os.remove(filename)



# # Initialize the recognizer
# r = sr.Recognizer()

# while True:
#     try:
#         # use microphone as source for input
#         with sr.Microphone() as source:
#             # Adjust for ambient noise here
#             r.adjust_for_ambient_noise(source, duration=0.2)
#             print('Select floor')
#             speak('select floor', language)
#             audio = r.listen(source, timeout=7, phrase_time_limit=7)
#             print('listented successfully')
#             try:
#                 print('processing')
#                 text = r.recognize_google(audio)
#                 if 'stop' in text.lower():
#                     speak('Quitting program', language)
#                     break

#                 # response_text = text 
#                 print('text: ', text)
#                 speak(text, language) 

#                 with open("action.txt", "w") as f:
#                     f.writelines(text)


#             except sr.UnknownValueError:
#                 speak("I could not understand you", language)
#             except sr.RequestError as e:
#                 print("Could not request results from Google Speech Recognition service; {0}".format(e))

#     except Exception as e:
#         print("[ERROR] An error occurred: {}".format(e))


#     print('waiting')
#     time.sleep(3)


