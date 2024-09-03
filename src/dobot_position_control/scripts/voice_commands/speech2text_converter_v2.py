import speech_recognition as sr
import time
from gtts import gTTS
import os

language = 'en'

def speak(text, lang='en'):
    """Convert text to speech and play it"""
    tts = gTTS(text=text, lang=lang, slow=False)
    tts.save("temp_voice.mp3")
    os.system("mpg123 temp_voice.mp3")


# Initialize the recognizer
r = sr.Recognizer()


while True:
    try:
        # use microphone as source for input
        with sr.Microphone() as source:
            speak("Select floor:", language)
            audio = r.listen(source)
            r.adjust_for_ambient_noise(source, duration=0.2)
            try:
                text = r.recognize_google(audio)
                if text.lower() == 'stop':
                    speak('Quitting program', language)
                    break

                response_text = text 
                speak(response_text, language)                

                time.sleep(3)
            except sr.UnknownValueError:
                speak("I could not understand you", language)
            except sr.RequestError as e:
                print("Could not request results from Google Speech Recognition service; {0}".format(e))

    except sr.UnknownValueError:
        print("[ERROR] Unknown error occured")
