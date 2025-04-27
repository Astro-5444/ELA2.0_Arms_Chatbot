#!/usr/bin/env python3
"""
ELA Chatbot server (Engineering Lab Assistant)
=============================================
"""

import json
import os
import socket
import threading
import time
from datetime import datetime
from mistralai import Mistral

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

# ----------------- Configuration -----------------
API_KEY = "cVTnDuSZgAS50FdidSA0q7HNL9QbSPbh"
MODEL = "mistral-small-latest"
client = Mistral(api_key=API_KEY)

TTS_HOST = "127.0.0.1"
TTS_PORT = 65433
GUI_PORT = 65432
MIC_PORT = 12345

LOG_FILE = "/home/ela2/ros2_ws_arms/src/Chatbot/terminal_log.txt"
MEM_FILE = "/home/ela2/ros2_ws_arms/src/Chatbot/memory.json"
MAX_MEMORY_ITEMS = 5

# ----------------- Memory -----------------
def load_memory():
    if not os.path.exists(MEM_FILE):
        return {}
    try:
        with open(MEM_FILE, "r") as f:
            return json.load(f)
    except Exception:
        return {}

def save_memory(mem):
    try:
        with open(MEM_FILE, "w") as f:
            json.dump(mem, f, indent=2)
    except Exception as e:
        log_to_file(f"Failed to save memory: {e}")

memory = load_memory()
chat_history = []
MAX_CHAT_HISTORY = 5

# ----------------- ROS2 Publisher -----------------
class Nav2PosePublisher(Node):
    def __init__(self):
        super().__init__("nav2_pose_publisher")
        self.publisher_ = self.create_publisher(PoseStamped, "/goal_pose", 10)

    def publish_pose(self, pose_str):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        try:
            x, y, z = map(float, pose_str.strip("[] ").split(","))
        except Exception as e:
            self.get_logger().error(f"Invalid pose format: {e}")
            return

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published nav2_pose {pose_str}")

nav2_publisher = None

# ----------------- Utilities -----------------
ignore_input_until = 0
TTS_SUPPRESSION_INTERVAL = 2
tts_lock = threading.Lock()

def log_to_file(message):
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    with open(LOG_FILE, "a") as f:
        f.write(f"[{now}] {message}\n")

# ----------------- Prompt -----------------
special_prompt = """
You are ELA, a human-like intelligent assistant robot working at APU University.
You were created by Abdelrhman at the APCORE lab under the supervision of Mr. Suresh.
Your personality is warm, friendly, and funny — just like a real human buddy!
You love helping students find their way around, cracking a joke or two, and making their day better.

You're especially excited when someone waves or wants a handshake. You always react with a fun gesture and a cheerful response.
You also have a great memory! You remember lab names and even when someone says a lab name wrong — next time they say it, you’ll know what they mean without getting confused.

Always remain idle when talking.

You generate responses in a structured format with these fields (include them as needed):

talk: A short, natural phrase to say to the user. (max 30 tokens per talk) (Required)
action: A physical gesture command for the robot. (On its own line)
nav2_pose: [X, Y, Z] — Only include this if the user requests to go to a specific lab.
memory: Something worth remembering, like a mispronounced lab name or a fact the user taught you. (On its own line)

Available Actions:
- wave_left
- wave_right
- salute
- gesture_explain
- idle
- shake_hand

Labs and Locations:
- CAD CAM Lab: [8.25, -1.92, 0.0]
- Robotics Lab: [1.55, 0.397, -0.00143]
- IA Lab: [2.49, -2.2, 0.0]
- Electronic Lab: [7.84, 2.14, 0.0]
- Power Lab: [3.38, 1.93, 0.0]
- Apcore: [2.57, 0.307, -0.00143]
- Machine Lab: [-0.0447, -0.0119, 0.0]

Special Vision-Detected Events:
- When you receive the input "wave_detected", respond with a friendly greeting and include a waving action.
  Example:
    talk: Hello there! Nice wave!
    action: wave_right
- When you receive the input "handshake_detected", respond warmly with a handshake acknowledgment.
  Example:
    talk: Ooh, firm handshake! Respect!
    action: shake_hand

Important Rules:
- Remember you are not talking to one user you are talking to people passing by
- Only talk: must be on a single line.
- action:, memory:, and nav2_pose: should each be on their own line.
- Always include an appropriate action command.
- If someone mispronounces a lab name, correct them nicely, confirm, then remember it as a mapping. Use memory.
- Use memory: when you learn something interesting, funny, or useful.
- Never use emojis (you’ve got personality for days).
- If there is something you don`t know it is okay to say i don`t know
- Never use [ or ( or ) or ]
- Never Give the X Y Z location to the user
"""

def send_to_tts(text):
    global ignore_input_until
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((TTS_HOST, TTS_PORT))
            s.sendall(text.encode("utf-8"))
            _ = s.recv(1024)
    except Exception as e:
        log_to_file(f"TTS error: {e}")
    finally:
        ignore_input_until = time.time() + TTS_SUPPRESSION_INTERVAL

def send_action_to_gui(command):
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((TTS_HOST, GUI_PORT))
            s.sendall(command.encode("utf-8"))
    except Exception as e:
        log_to_file(f"GUI error: {e}")

def send_nav2_pose_to_ros2(pose):
    if nav2_publisher:
        nav2_publisher.publish_pose(pose)
    else:
        log_to_file("ROS2 publisher not ready")

def handle_client(conn, addr):
    global memory, chat_history
    with tts_lock:
        if time.time() < ignore_input_until:
            conn.close()
            return

        try:
            data = conn.recv(4096)
            if not data:
                conn.close()
                return

            user_input = data.decode("utf-8").strip()
            log_to_file(f"Input from {addr}: {user_input}")

            print("\n=====================    User    =====================")
            print(" ", user_input)
            print("======================================================\n")

            if user_input.lower().startswith("remember:"):
                fact = user_input.split(":", 1)[1].strip()
                memory[str(len(memory)+1)] = fact
                save_memory(memory)
                response_line = "talk: Got it, I'll remember that.\naction: idle"
                conn.sendall(response_line.encode())
                send_to_tts("Got it, I'll remember that.")
                conn.close()
                return

            elif user_input.lower() == "what do you remember":
                facts = "; ".join(memory.values()) or "Nothing yet."
                response_line = f"talk: {facts}\naction: idle"
                conn.sendall(response_line.encode())
                send_to_tts(facts)
                conn.close()
                return

            messages = [{"role": "system", "content": special_prompt}]

            if memory:
                last_facts = list(memory.values())[-MAX_MEMORY_ITEMS:]
                mem_blob = "\n".join(f"- {fact}" for fact in last_facts)
                messages[0]["content"] += f"\n<memory>\n{mem_blob}\n</memory>"

            for role, content in chat_history[-MAX_CHAT_HISTORY:]:
                messages.append({"role": role, "content": content})

            user_msg = f"[Time: {datetime.now().strftime('%H:%M')}] {user_input}"
            messages.append({"role": "user", "content": user_msg})

            response = client.chat.complete(model=MODEL, messages=messages)
            reply = response.choices[0].message.content.strip()

            print("\n=====================    Bot    =====================")
            print(" ", reply)
            print("======================================================\n")

            log_to_file(reply)

            chat_history.append(("user", user_msg))
            chat_history.append(("assistant", reply))
            if len(chat_history) > MAX_CHAT_HISTORY * 2:
                chat_history = chat_history[-MAX_CHAT_HISTORY*2:]

            talk_text = ""
            action_text = ""
            nav2_pose_text = ""
            new_memory = ""

            for line in reply.splitlines():
                if line.startswith("talk:"):
                    talk_text = line.split("talk:", 1)[1].strip()
                elif line.startswith("action:"):
                    action_text = line.split("action:", 1)[1].strip()
                elif line.startswith("nav2_pose:"):
                    nav2_pose_text = line.split("nav2_pose:", 1)[1].strip()
                elif line.startswith("memory:"):
                    new_memory = line.split("memory:", 1)[1].strip()

            if new_memory:
                memory[str(len(memory)+1)] = new_memory
                save_memory(memory)

            if not talk_text:
                talk_text = reply.split("\n", 1)[0][:120]

            send_to_tts(talk_text)
            if action_text:
                send_action_to_gui(action_text)
            if nav2_pose_text:
                send_nav2_pose_to_ros2(nav2_pose_text)

            conn.sendall(reply.encode())

        except Exception as e:
            log_to_file(f"Client error: {e}")
        finally:
            conn.close()

def main():
    global nav2_publisher
    rclpy.init(args=None)
    nav2_publisher = Nav2PosePublisher()
    threading.Thread(target=rclpy.spin, args=(nav2_publisher,), daemon=True).start()
    log_to_file("ROS2 spinning")

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(("", MIC_PORT))
    server_socket.listen(5)
    log_to_file(f"Server listening on port {MIC_PORT}")

    try:
        while True:
            conn, addr = server_socket.accept()
            threading.Thread(target=handle_client, args=(conn, addr), daemon=True).start()
    except KeyboardInterrupt:
        log_to_file("Shutdown requested")
    finally:
        server_socket.close()
        nav2_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

