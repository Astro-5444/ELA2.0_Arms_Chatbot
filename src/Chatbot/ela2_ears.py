#!/usr/bin/env python3
import socket
import speech_recognition as sr
import noisereduce as nr
import numpy as np
import soundfile as sf
import io

HOST = '127.0.0.1'  # Address of the chatbot server
PORT = 12345        # Must match the server's port

def reduce_noise(audio: sr.AudioData):
    # Convert AudioData to WAV bytes
    wav_bytes = audio.get_wav_data()
    
    # Load it using soundfile
    y, sr_rate = sf.read(io.BytesIO(wav_bytes))
    
    # Apply noise reduction
    reduced = nr.reduce_noise(y=y, sr=sr_rate)
    
    # Save cleaned audio to memory buffer
    buffer = io.BytesIO()
    sf.write(buffer, reduced, sr_rate, format='WAV')
    buffer.seek(0)
    
    return buffer

def main():
    recognizer = sr.Recognizer()
    mic = sr.Microphone()
    print("Microphone client started. Listening for speech...")

    while True:
        with mic as source:
            print("Adjusting for ambient noise...")
            recognizer.adjust_for_ambient_noise(source, duration=1)
            print("Listening...")
            audio = recognizer.listen(source)

        try:
            # Apply local noise reduction
            print("Reducing noise...")
            cleaned_audio = reduce_noise(audio)

            # Re-load the cleaned audio into recognizer
            with sr.AudioFile(cleaned_audio) as source:
                audio = recognizer.record(source)

            # Transcribe
            text = recognizer.recognize_google(audio)
            print("Recognized:", text)
        except sr.UnknownValueError:
            print("Could not understand audio, please try again.")
            continue
        except sr.RequestError as e:
            print(f"Speech recognition error: {e}")
            continue

        # Send to chatbot
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((HOST, PORT))
                s.sendall(text.encode('utf-8'))
                response = s.recv(4096)
                if response:
                    print("Chatbot Response:", response.decode('utf-8'))
        except Exception as e:
            print("Error connecting to chatbot server:", e)

if __name__ == "__main__":
    main()

