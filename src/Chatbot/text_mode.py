#!/usr/bin/env python3
import socket

HOST = '127.0.0.1'  # Address of the chatbot server
PORT = 12345        # Must match the server's port

def main():
    print("Text mode client started. Type your message and press Enter.")

    while True:
        try:
            text = input("You: ").strip()
            if not text:
                continue

            # Send to chatbot
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((HOST, PORT))
                s.sendall(text.encode('utf-8'))
                response = s.recv(4096)
                if response:
                    print("Chatbot:", response.decode('utf-8'))
        except KeyboardInterrupt:
            print("\n[Stopped by user]")
            break
        except Exception as e:
            print("Error communicating with chatbot:", e)

if __name__ == "__main__":
    main()

