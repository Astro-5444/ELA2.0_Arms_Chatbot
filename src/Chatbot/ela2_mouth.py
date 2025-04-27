#!/usr/bin/env python3
import socket
import numpy as np
import sounddevice as sd
from nix.models.TTS import NixTTSInference
import re

# TTS parameters for Nix-TTS
tts_sample_rate = 22050  # Nix-TTS typically uses 22050 Hz

# Set the path to your downloaded Nix-TTS model
model_dir = "/home/ela2/nix-tts/nix-ljspeech-deterministic-v0.1"  # Replace with your model path

# Initialize the Nix-TTS inference engine
nix = NixTTSInference(model_dir=model_dir)

def remove_emoji(text):
    emoji_pattern = re.compile(
        "[" 
        u"\U0001F600-\U0001F64F"  # Emoticons
        u"\U0001F300-\U0001F5FF"  # Symbols & pictographs
        u"\U0001F680-\U0001F6FF"  # Transport & map symbols
        u"\U0001F1E0-\U0001F1FF"  # Flags
        u"\U00002700-\U000027BF"  # Dingbats
        u"\U0001F900-\U0001F9FF"  # Supplemental Symbols and Pictographs
        u"\U0001FA70-\U0001FAFF"  # Extended Symbols and Pictographs
        u"\U00002600-\U000026FF"  # Misc symbols
        "]+", flags=re.UNICODE)
    return emoji_pattern.sub(r'', text)

def tts_play(text):
    text = remove_emoji(text)
    print("TTS Service: Speaking ->", text)
    
    # Tokenize the input text using Nix-TTS
    c, c_length, phoneme = nix.tokenize(text)
    # Synthesize speech waveform
    xw = nix.vocalize(c, c_length)
    
    # xw is a NumPy array with shape (1, 1, n_samples)
    sd.play(xw[0, 0], tts_sample_rate)
    sd.wait()
    print("TTS Service: Done speaking.")

def main():
    host = '127.0.0.1'
    port = 65433  # Port dedicated to TTS communication
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((host, port))
    s.listen(5)
    print("TTS Service is listening on {}:{}".format(host, port))
    
    while True:
        conn, addr = s.accept()
        print("TTS Service: Connection from", addr)
        data = conn.recv(4096)
        if not data:
            conn.close()
            continue
        text = data.decode('utf-8')
        tts_play(text)
        conn.sendall(b"OK")  # Send back an acknowledgment
        conn.close()

if __name__ == "__main__":
    main()

