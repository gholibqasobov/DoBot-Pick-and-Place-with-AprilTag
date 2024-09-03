"""smart room voice assistant code as a sample"""

import pvporcupine
from pvrecorder import PvRecorder
import argparse
import queue
import sys
import sounddevice as sd
import asyncio 
import websockets 
from vosk import Model, KaldiRecognizer

# Открытие двери
async def hello(): 
    uri = "ws://192.168.0.200:1880/ws/testing" 
    async with websockets.connect(uri) as websocket: 
        await websocket.send("alibek") 

# Обработка ошибок при инициализации Porcupine
def initialize_porcupine():
    try:
        porcupine = pvporcupine.create(
            access_key='Q9WemUrswmo+nC+NUjX/S6+1OAXqrnnol1blHXVoPqw9CQ4mb5z8TA==',
            keyword_paths=["Jarvis_en_raspberry-pi_v3_0_0.ppn"],
            sensitivities=[1.0]
        )
        return porcupine
    except pvporcupine.PorcupineException as e:
        print("Error initializing Porcupine:", str(e))
        sys.exit(1)

# Инициализация парсера с комментариями и устранением магических чисел
def initialize_parser():
    parser = argparse.ArgumentParser(description="Voice assistant with Porcupine wake word detection")
    parser.add_argument("-l", "--list-devices", action="store_true", help="show list of audio devices and exit")
    args, remaining = parser.parse_known_args()
    if args.list_devices:
        print(sd.query_devices())
        sys.exit(0)
    parser.add_argument("-f", "--filename", type=str, metavar="FILENAME", help="audio file to store recording to")
    parser.add_argument("-d", "--device", type=int_or_str, help="input device (numeric ID or substring)")
    parser.add_argument("-r", "--samplerate", type=int, help="sampling rate")
    return parser, remaining

# Добавление документации к функции int_or_str
def int_or_str(text):
    """Convert text to int if possible, otherwise return as is."""
    try:
        return int(text)
    except ValueError:
        return text

# Обработка аудио с добавлением комментариев
def process_audio(model, args, q):
    try:
        # Инициализация параметров аудиозаписи
        if args.samplerate is None:
            device_info = sd.query_devices(args.device, "input")
            args.samplerate = int(device_info["default_samplerate"])
        if args.filename:
            dump_fn = open(args.filename, "wb")
        else:
            dump_fn = None
        
        # Начало записи с обработчиком колбэка
        with sd.RawInputStream(samplerate=args.samplerate, blocksize=8000, device=args.device,
                               dtype="int16", channels=1, callback=callback):
            print("#" * 80)
            print("Press Ctrl+C to stop the recording")
            print("#" * 80)

            rec = KaldiRecognizer(model, args.samplerate)
            while True:
                data = q.get()
                if rec.AcceptWaveform(data):
                    result = rec.Result()
                    if "включи свет" in result.lower():
                        print("Команда: Включи свет")
                        break
                    if "открой дверь" in result.lower():
                        print("Команда: Открой дверь")
                        asyncio.run(hello())
                        break
                if dump_fn is not None:
                    dump_fn.write(data)

    except KeyboardInterrupt:
        print("\nDone")
        sys.exit(0)
    except Exception as e:
        sys.exit(type(e).name + ": " + str(e))

# Инициализация модели распознавания речи
def initialize_recognizer(model_path):
    try:
        model = Model(model_path)
        return model
    except Exception as e:
        print("Error initializing the speech recognizer model:", str(e))
        sys.exit(1)

# Обработка аудио-блоков в колбэке
def callback(indata, frames, time, status):
    if status:
        print(status, file=sys.stderr)
    q.put(bytes(indata))

# Основная логика программы
if __name__ == "main":
    q = queue.Queue()
    parser, remaining = initialize_parser()
    args = parser.parse_args(remaining)
    model_path = "./small_model"

    # Инициализация Porcupine
    porcupine = initialize_porcupine()
# Инициализация модели распознавания речи
    model = initialize_recognizer(model_path)

    # Инициализация записывающего устройства
    recorder = PvRecorder(device_index=1, frame_length=porcupine.frame_length)

    # Запуск цикла обработки аудио
    recorder.start()
    try:
        while True:
            pcm = recorder.read()
            keyword_index = porcupine.process(pcm)
            if keyword_index == 0:
                recorder.stop()
                process_audio(model, args, q)
                recorder.start()
    finally:
        recorder.stop()
