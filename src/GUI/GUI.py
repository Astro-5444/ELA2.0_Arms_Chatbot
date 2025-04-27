#!/usr/bin/env python3
"""
Multi‑page Arm Control GUI with a Separate Center Position Page and Emergency Stop.

Pages:
  1. StartPage – Displays action buttons (for left, right, and both arms) and a “Stop”
     button (for emergency stop), plus navigation to the Center Position page,
     Action Configuration, and a toggle to enable/disable a default action.
  2. ActionConfigPage – Lists actions (with options to create, edit, or delete).
     Also allows editing the global speed factor, which applies to all actions.
  3. CreateActionPage – Lets you create (or edit) an action. In addition to the action name, arm,
     and steps (one JSON object per line), you can specify:
       • Motion Mode ("single" or "continuous")
       • Delay between steps (sec)
     Also, a button is added to open manual control so you can move the arm(s) manually and then
     extract the positions into the steps textbox.
  4. CenterPositionPage – Displays the center position configuration for both arms. When opened,
     you can view/edit the center positions and press “Set Center” to command the arms to move there.
  5. ManualControlPage – Allows live manual control of each joint using sliders.

All configuration data is saved to JSON files, including a global speed factor
that applies to all arm motions (actions, stop, center, and manual control).
"""

import tkinter as tk
from tkinter import ttk, messagebox
import json
import os
import threading
import time
import socket

import rclpy  # Make sure your ROS2 environment is sourced!
from rclpy.node import Node
from sensor_msgs.msg import JointState

# ===============================
# Global Definitions & Defaults
# ===============================

ACTIONS_CONFIG_FILE = "/home/ela2/ros2_ws_arms/src/GUI/actions_config.json"
CENTER_CONFIG_FILE = "/home/ela2/ros2_ws_arms/src/GUI/center_config.json"

GLOBAL_SPEED_FACTOR = 1.0
stop_event = threading.Event()

ARM_JOINT_NAMES = {
    "left": [
        "left_upper_arm_y_joint",
        "left_upper_arm_x_joint",
        "left_forearm_z_joint",
        "left_forearm_x_joint",
        "left_finger_joint"
    ],
    "right": [
        "right_upper_arm_y_joint",
        "right_upper_arm_x_joint",
        "right_forearm_z_joint",
        "right_forearm_x_joint",
        "right_finger_joint"
    ]
}

JOINT_LIMITS = {
    "left_upper_arm_y_joint": (0.0, 1.5708),
    "left_upper_arm_x_joint": (-1.197, 0.704),
    "left_forearm_z_joint": (-1.5708, 1.5708),
    "left_forearm_x_joint": (-1.5708, 1.5708),
    "left_finger_joint": (-0.4, 0.3),
    "right_upper_arm_y_joint": (-1.5708, -0.12),
    "right_upper_arm_x_joint": (-1.197, 0.704),
    "right_forearm_z_joint": (-1.5708, 1.5708),
    "right_forearm_x_joint": (-1.5708, 1.5708),
    "right_finger_joint": (-0.4, 0.3)
}

DEFAULT_CONFIG = {
    "global_speed_factor": 1.0,
    "actions": {
        "Waving": {
            "name": "Waving",
            "arm": "left",
            "steps": [
                {"positions": [1.0, 0.5, -1.0, -0.5, 0.0], "duration": 1.0, "type": "moving"},
                {"positions": [1.2, 0.7, -0.8, -0.3, 0.0], "duration": 1.0, "type": "moving"}
            ],
            "motion_mode": "continuous",
            "step_delay": 0.0
        },
        "Hello": {
            "name": "Hello",
            "arm": "left",
            "steps": [
                {"positions": [0.5, 0.5, 0.0, 0.0, 0.0], "duration": 1.0, "type": "moving"}
            ],
            "motion_mode": "single",
            "step_delay": 0.0
        }
    }
}

DEFAULT_CENTER_CONFIG = {
    "left": [0.0, 0.0, 0.0, 0.0, 0.0],
    "right": [0.0, 0.0, 0.0, 0.0, 0.0]
}

# Global variables for default action management.
current_action_thread = None      # Currently running (user or default) action thread.
# The default action thread (if enabled) runs independently.
default_action_thread = None
is_user_action_running = False      # True when a user action is active.
default_action_enabled = False      # Global flag: whether the default action loop is active.

# Define a default action (you may adjust positions/durations as needed).
DEFAULT_ACTION = {
    "name": "DefaultAction",
    "arm": "both",
    "steps": [
        {
            "left_positions": [0.5, 0.5, 0.5, 0.5, 0.5],
            "right_positions": [-0.5, -0.5, -0.5, -0.5, -0.5],
            "duration": 1.0,
            "type": "moving"
        }
    ],
    "motion_mode": "single",
    "step_delay": 0.0
}

# ===============================
# Configuration Load/Save Helpers
# ===============================

def load_actions_config():
    global GLOBAL_SPEED_FACTOR
    if os.path.exists(ACTIONS_CONFIG_FILE):
        try:
            with open(ACTIONS_CONFIG_FILE, "r") as f:
                data = json.load(f)
                GLOBAL_SPEED_FACTOR = data.get("global_speed_factor", 1.0)
                actions = data.get("actions", {})
                return actions
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load actions config:\n{e}")
            GLOBAL_SPEED_FACTOR = DEFAULT_CONFIG["global_speed_factor"]
            return DEFAULT_CONFIG["actions"].copy()
    else:
        GLOBAL_SPEED_FACTOR = DEFAULT_CONFIG["global_speed_factor"]
        return DEFAULT_CONFIG["actions"].copy()

def save_actions_config(actions):
    global GLOBAL_SPEED_FACTOR
    data = {
        "global_speed_factor": GLOBAL_SPEED_FACTOR,
        "actions": actions
    }
    try:
        with open(ACTIONS_CONFIG_FILE, "w") as f:
            json.dump(data, f, indent=4)
    except Exception as e:
        messagebox.showerror("Error", f"Failed to save actions config:\n{e}")

def load_center_config():
    if os.path.exists(CENTER_CONFIG_FILE):
        try:
            with open(CENTER_CONFIG_FILE, "r") as f:
                return json.load(f)
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load center config:\n{e}")
            return DEFAULT_CENTER_CONFIG.copy()
    else:
        return DEFAULT_CENTER_CONFIG.copy()

def save_center_config(center_config):
    try:
        with open(CENTER_CONFIG_FILE, "w") as f:
            json.dump(center_config, f, indent=4)
    except Exception as e:
        messagebox.showerror("Error", f"Failed to save center config:\n{e}")

# ===============================
# ROS Publisher Helper Functions
# ===============================

def publish_center_position(arm, positions):
    node = Node(f"center_{arm}")
    publisher = node.create_publisher(JointState, "/joint_states", 10)
    msg = JointState()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.name = ARM_JOINT_NAMES[arm]
    msg.position = positions
    publisher.publish(msg)
    node.get_logger().info(f"Published center position for {arm} arm: {positions}")
    time.sleep(1)
    node.destroy_node()

current_positions = {
    "left": DEFAULT_CENTER_CONFIG["left"].copy(),
    "right": DEFAULT_CENTER_CONFIG["right"].copy()
}

def publish_action_step(arm, joint_names, target_positions, duration, step_type, speed_factor=1.0):
    global current_positions
    final_duration = duration * speed_factor
    if step_type == "moving" and final_duration > 0:
        start_positions = current_positions.get(arm, [0.0] * len(target_positions))
        dt = 0.05
        n_steps = max(1, int(final_duration / dt))
        node = Node(f"action_step_{arm}")
        publisher = node.create_publisher(JointState, "/joint_states", 10)
        for i in range(1, n_steps + 1):
            intermediate_positions = [
                start + (target - start) * i / n_steps
                for start, target in zip(start_positions, target_positions)
            ]
            msg = JointState()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.name = joint_names
            msg.position = intermediate_positions
            publisher.publish(msg)
            time.sleep(dt)
        current_positions[arm] = target_positions.copy()
        node.destroy_node()
    else:
        node = Node(f"action_step_{arm}")
        publisher = node.create_publisher(JointState, "/joint_states", 10)
        msg = JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.name = joint_names
        msg.position = target_positions
        publisher.publish(msg)
        time.sleep(duration + 0.1)
        current_positions[arm] = target_positions.copy()
        node.destroy_node()

def run_center_ros(arm, positions):
    publish_center_position(arm, positions)

def run_action_ros(action):
    arm = action["arm"]
    motion_mode = action.get("motion_mode", "single")
    step_delay = action.get("step_delay", 0.0)
    global GLOBAL_SPEED_FACTOR
    speed_factor = GLOBAL_SPEED_FACTOR
    steps = action["steps"]

    if arm == "both":
        def execute_steps_for_arm(target_arm):
            for step in steps:
                if stop_event.is_set():
                    return
                if "left_positions" in step and "right_positions" in step:
                    positions = step["left_positions"] if target_arm == "left" else step["right_positions"]
                else:
                    positions = step.get("positions", [])
                duration = step.get("duration", 1.0)
                step_type = step.get("type", "moving")
                publish_action_step(target_arm, ARM_JOINT_NAMES[target_arm], positions, duration, step_type, speed_factor)
                time.sleep(step_delay)
        if motion_mode == "continuous":
            while not stop_event.is_set():
                t1 = threading.Thread(target=execute_steps_for_arm, args=("left",))
                t2 = threading.Thread(target=execute_steps_for_arm, args=("right",))
                t1.start(); t2.start()
                t1.join(); t2.join()
        else:
            t1 = threading.Thread(target=execute_steps_for_arm, args=("left",))
            t2 = threading.Thread(target=execute_steps_for_arm, args=("right",))
            t1.start(); t2.start()
            t1.join(); t2.join()
    else:
        def execute_steps_for_arm():
            for step in steps:
                if stop_event.is_set():
                    return
                positions = step.get("positions", [])
                duration = step.get("duration", 1.0)
                step_type = step.get("type", "moving")
                publish_action_step(arm, ARM_JOINT_NAMES[arm], positions, duration, step_type, speed_factor)
                time.sleep(step_delay)
        if motion_mode == "continuous":
            while not stop_event.is_set():
                execute_steps_for_arm()
        else:
            execute_steps_for_arm()

def run_stop_ros(arm):
    node = Node(f"emergency_stop_{arm}")
    publisher = node.create_publisher(JointState, "/joint_states", 10)
    msg = JointState()
    msg.header.stamp = node.get_clock().now().to_msg()
    center_config = load_center_config()
    positions = center_config.get(arm, [0.0] * 5)
    msg.name = ARM_JOINT_NAMES[arm]
    msg.position = positions
    publisher.publish(msg)
    node.get_logger().info(f"Published emergency stop for {arm} arm: {positions}")
    time.sleep(0.5)
    node.destroy_node()

# ===============================
# Default Action Management
# ===============================

def run_default_action():
    global default_action_enabled
    while default_action_enabled and not stop_event.is_set():
        run_action_ros(DEFAULT_ACTION)
        time.sleep(0.1)

def start_default_action():
    global default_action_thread, default_action_enabled
    if not default_action_enabled:
        return
    if default_action_thread and default_action_thread.is_alive():
        return
    stop_event.clear()
    default_action_thread = threading.Thread(target=run_default_action, daemon=True)
    default_action_thread.start()

def stop_default_action():
    global default_action_enabled
    default_action_enabled = False
    stop_event.set()

def stop_current_action():
    global current_action_thread
    stop_event.set()
    if current_action_thread and current_action_thread.is_alive():
        current_action_thread.join(timeout=5)
    current_action_thread = None

def run_action_in_thread(action, is_user_action=True):
    global current_action_thread, is_user_action_running
    stop_current_action()
    stop_event.clear()
    if is_user_action:
        is_user_action_running = True
    t = threading.Thread(target=run_action_ros, args=(action,), daemon=True)
    t.start()
    current_action_thread = t
    if is_user_action:
        threading.Thread(target=watch_user_action_and_restart_default, daemon=True).start()

def watch_user_action_and_restart_default():
    global is_user_action_running
    if current_action_thread:
        current_action_thread.join()
    is_user_action_running = False
    if default_action_enabled:
        start_default_action()

def run_stop_in_thread():
    stop_event.set()
    threading.Thread(target=run_stop_ros, args=("left",), daemon=True).start()
    threading.Thread(target=run_stop_ros, args=("right",), daemon=True).start()

# ===============================
# TCP Socket Command Listener
# ===============================

def start_command_listener(gui_app, host='127.0.0.1', port=65432):
    def listen():
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((host, port))
            s.listen()
            print(f"Command listener started on {host}:{port}")
            while True:
                conn, addr = s.accept()
                with conn:
                    data = conn.recv(1024)
                    if data:
                        command = data.decode('utf-8').strip()
                        print(f"Received command: {command}")
                        actions = gui_app.actions_config
                        if command in actions:
                            run_action_in_thread(actions[command], is_user_action=True)
                        else:
                            print(f"Action '{command}' not found in actions configuration.")
    threading.Thread(target=listen, daemon=True).start()

# ===============================
# Tkinter Application Classes
# ===============================

class ArmControlApp(tk.Tk):
    ARM_JOINT_NAMES = {
        "left": ["left_upper_arm_y_joint", "left_upper_arm_x_joint", "left_forearm_z_joint", "left_forearm_x_joint", "left_finger_joint"],
        "right": ["right_upper_arm_y_joint", "right_upper_arm_x_joint", "right_forearm_z_joint", "right_forearm_x_joint", "right_finger_joint"]
    }
    
    def __init__(self):
        super().__init__()
        self.title("Arm Control GUI")
        self.geometry("900x650")
        self.actions_config = load_actions_config()  # Loads actions and sets GLOBAL_SPEED_FACTOR.
        self.center_config = load_center_config()
        self.default_action_enabled = tk.BooleanVar(value=False)  # Default action off on startup.
        
        container = tk.Frame(self)
        container.pack(side="top", fill="both", expand=True)
        container.grid_rowconfigure(0, weight=1)
        container.grid_columnconfigure(0, weight=1)
        self.frames = {}
        for PageClass in (StartPage, ActionConfigPage, CreateActionPage, CenterPositionPage, ManualControlPage):
            page_name = PageClass.__name__
            frame = PageClass(parent=container, controller=self)
            self.frames[page_name] = frame
            frame.grid(row=0, column=0, sticky="nsew")
        self.show_frame("StartPage")
        
        # Add navigation to ManualControlPage.
        start_page = self.frames["StartPage"]
        tk.Button(start_page, text="Manual Control", command=lambda: self.show_frame("ManualControlPage")).pack(pady=10)
        
        # Automatically publish center positions on startup.
        run_center_in_thread(self.center_config)
    
    def show_frame(self, page_name):
        frame = self.frames[page_name]
        frame.tkraise()
        if hasattr(frame, "refresh"):
            frame.refresh()
    
    def update_actions_config(self, new_config):
        self.actions_config = new_config
        save_actions_config(self.actions_config)
        if "StartPage" in self.frames:
            self.frames["StartPage"].refresh()
        if "ActionConfigPage" in self.frames:
            self.frames["ActionConfigPage"].refresh()

    def update_center_config(self, new_center):
        self.center_config = new_center
        save_center_config(self.center_config)
        if "CenterPositionPage" in self.frames:
            self.frames["CenterPositionPage"].refresh()
    
    def toggle_default_action(self):
        global default_action_enabled, stop_event
        if self.default_action_enabled.get():
            default_action_enabled = True
            stop_event.clear()
            start_default_action()
        else:
            stop_default_action()

class StartPage(tk.Frame):
    """
    Displays action buttons, emergency stop, navigation buttons, and the default action toggle.
    """
    def __init__(self, parent, controller):
        super().__init__(parent)
        self.controller = controller

        title = tk.Label(self, text="Select Action", font=("Arial", 18))
        title.pack(pady=10)

        stop_btn = tk.Button(self, text="STOP", fg="white", bg="red", font=("Arial", 14, "bold"),
                             command=run_stop_in_thread)
        stop_btn.pack(pady=5)

        nav_frame = tk.Frame(self)
        nav_frame.pack(pady=5)
        tk.Button(nav_frame, text="Center Position", command=lambda: controller.show_frame("CenterPositionPage")).pack(side="left", padx=10)
        tk.Button(nav_frame, text="Configure Actions", command=lambda: controller.show_frame("ActionConfigPage")).pack(side="left", padx=10)

        # Add Default Action toggle.
        toggle_frame = tk.Frame(self)
        toggle_frame.pack(pady=5)
        chk = tk.Checkbutton(toggle_frame, text="Enable Default Action", variable=controller.default_action_enabled,
                               command=controller.toggle_default_action)
        chk.pack()

        actions_frame = tk.Frame(self)
        actions_frame.pack(fill="both", expand=True, padx=20, pady=10)
        self.left_frame = tk.LabelFrame(actions_frame, text="Left Arm Actions", padx=10, pady=10)
        self.left_frame.pack(side="left", fill="both", expand=True, padx=10, pady=10)
        self.both_frame = tk.LabelFrame(actions_frame, text="Both Arms Actions", padx=10, pady=10)
        self.both_frame.pack(side="left", fill="both", expand=True, padx=10, pady=10)
        self.right_frame = tk.LabelFrame(actions_frame, text="Right Arm Actions", padx=10, pady=10)
        self.right_frame.pack(side="left", fill="both", expand=True, padx=10, pady=10)
        self.refresh()

    def refresh(self):
        for frame in (self.left_frame, self.both_frame, self.right_frame):
            for widget in frame.winfo_children():
                widget.destroy()
        actions = self.controller.actions_config
        for action_name, action in actions.items():
            arm = action.get("arm", "left")
            if arm == "left":
                parent_frame = self.left_frame
            elif arm == "right":
                parent_frame = self.right_frame
            else:
                parent_frame = self.both_frame
            btn = tk.Button(parent_frame, text=action_name,
                            command=lambda act=action: run_action_in_thread(act, is_user_action=True))
            btn.pack(pady=5, fill="x", padx=5)

class ActionConfigPage(tk.Frame):
    """
    Lists actions with options to create, edit, delete, plus adjust the global speed factor.
    """
    def __init__(self, parent, controller):
        super().__init__(parent)
        self.controller = controller

        title = tk.Label(self, text="Action Configuration", font=("Arial", 18))
        title.pack(pady=10)

        speed_frame = tk.Frame(self)
        speed_frame.pack(pady=5)
        tk.Label(speed_frame, text="Global Speed Factor (1.0 = normal):").pack(side="left", padx=5)
        self.global_speed_var = tk.StringVar(value=str(GLOBAL_SPEED_FACTOR))
        self.global_speed_entry = tk.Entry(speed_frame, textvariable=self.global_speed_var, width=10)
        self.global_speed_entry.pack(side="left", padx=5)
        tk.Button(speed_frame, text="Save Speed Factor", command=self.save_global_speed).pack(side="left", padx=5)

        self.listbox = tk.Listbox(self, width=50, height=15)
        self.listbox.pack(pady=10)

        btn_frame = tk.Frame(self)
        btn_frame.pack(pady=10)
        tk.Button(btn_frame, text="Create New Action", command=self.create_action).pack(side="left", padx=5)
        tk.Button(btn_frame, text="Edit Action", command=self.edit_action).pack(side="left", padx=5)
        tk.Button(btn_frame, text="Delete Action", command=self.delete_action).pack(side="left", padx=5)
        tk.Button(btn_frame, text="Back", command=lambda: controller.show_frame("StartPage")).pack(side="left", padx=5)
        self.refresh()
    
    def refresh(self):
        self.listbox.delete(0, tk.END)
        actions = self.controller.actions_config
        for action_name in actions:
            self.listbox.insert(tk.END, action_name)
        self.global_speed_var.set(str(GLOBAL_SPEED_FACTOR))

    def save_global_speed(self):
        global GLOBAL_SPEED_FACTOR
        try:
            new_speed = float(self.global_speed_var.get())
            GLOBAL_SPEED_FACTOR = new_speed
            self.controller.update_actions_config(self.controller.actions_config)
            messagebox.showinfo("Saved", f"Global Speed Factor set to {new_speed}")
        except ValueError:
            messagebox.showerror("Error", "Please enter a valid number for the speed factor.")

    def create_action(self):
        self.controller.show_frame("CreateActionPage")
        create_page = self.controller.frames["CreateActionPage"]
        create_page.load_action(None)
    
    def edit_action(self):
        selection = self.listbox.curselection()
        if not selection:
            messagebox.showwarning("No selection", "Please select an action to edit.")
            return
        action_name = self.listbox.get(selection[0])
        action = self.controller.actions_config.get(action_name)
        self.controller.show_frame("CreateActionPage")
        create_page = self.controller.frames["CreateActionPage"]
        create_page.load_action(action)
    
    def delete_action(self):
        selection = self.listbox.curselection()
        if not selection:
            messagebox.showwarning("No selection", "Please select an action to delete.")
            return
        action_name = self.listbox.get(selection[0])
        if messagebox.askyesno("Delete Action", f"Are you sure you want to delete '{action_name}'?"):
            actions = self.controller.actions_config
            if action_name in actions:
                del actions[action_name]
                self.controller.update_actions_config(actions)
                self.refresh()

class CreateActionPage(tk.Frame):
    """
    Allows creating or editing an action, with an option to extract positions from manual control.
    """
    def __init__(self, parent, controller):
        super().__init__(parent)
        self.controller = controller
        self.current_action = None

        title = tk.Label(self, text="Create / Edit Action", font=("Arial", 18))
        title.pack(pady=10)
        
        form_frame = tk.Frame(self)
        form_frame.pack(pady=10)
        tk.Label(form_frame, text="Action Name:").grid(row=0, column=0, sticky="e", padx=5, pady=5)
        self.name_entry = tk.Entry(form_frame, width=30)
        self.name_entry.grid(row=0, column=1, padx=5, pady=5)
        
        tk.Label(form_frame, text="Arm:").grid(row=1, column=0, sticky="e", padx=5, pady=5)
        self.arm_var = tk.StringVar(value="left")
        arm_menu = ttk.Combobox(form_frame, textvariable=self.arm_var,
                                values=["left", "right", "both"], state="readonly", width=28)
        arm_menu.grid(row=1, column=1, padx=5, pady=5)
        
        tk.Label(form_frame, text="Motion Mode:").grid(row=2, column=0, sticky="e", padx=5, pady=5)
        self.motion_mode_var = tk.StringVar(value="single")
        mode_frame = tk.Frame(form_frame)
        mode_frame.grid(row=2, column=1, padx=5, pady=5)
        tk.Radiobutton(mode_frame, text="Single", variable=self.motion_mode_var, value="single").pack(side="left")
        tk.Radiobutton(mode_frame, text="Continuous", variable=self.motion_mode_var, value="continuous").pack(side="left")
        
        tk.Label(form_frame, text="Delay Between Steps (sec):").grid(row=3, column=0, sticky="e", padx=5, pady=5)
        self.delay_entry = tk.Entry(form_frame, width=30)
        self.delay_entry.grid(row=3, column=1, padx=5, pady=5)
        self.delay_entry.insert(0, "0.0")
        
        tk.Label(self, text="Steps (one JSON object per line):").pack(pady=5)
        self.steps_text = tk.Text(self, width=80, height=10)
        self.steps_text.pack(pady=5)
        tk.Label(self, text='Example: {"positions": [1.0, 0.5, -1.0, -0.5, 0.0], "duration": 1.0, "type": "moving"}').pack(pady=5)
        
        tk.Button(self, text="Open Manual Control", command=self.open_manual_control).pack(pady=5)
        
        btn_frame = tk.Frame(self)
        btn_frame.pack(pady=10)
        tk.Button(btn_frame, text="Save Action", command=self.save_action).pack(side="left", padx=5)
        tk.Button(btn_frame, text="Cancel", command=lambda: controller.show_frame("ActionConfigPage")).pack(side="left", padx=5)
    
    def load_action(self, action):
        self.current_action = action
        if action:
            self.name_entry.delete(0, tk.END)
            self.name_entry.insert(0, action.get("name", ""))
            self.arm_var.set(action.get("arm", "left"))
            self.motion_mode_var.set(action.get("motion_mode", "single"))
            self.delay_entry.delete(0, tk.END)
            self.delay_entry.insert(0, str(action.get("step_delay", 0.0)))
            self.steps_text.delete("1.0", tk.END)
            for step in action.get("steps", []):
                self.steps_text.insert(tk.END, json.dumps(step) + "\n")
        else:
            self.name_entry.delete(0, tk.END)
            self.arm_var.set("left")
            self.motion_mode_var.set("single")
            self.delay_entry.delete(0, tk.END)
            self.delay_entry.insert(0, "0.0")
            self.steps_text.delete("1.0", tk.END)
            example = '{"positions": [1.2, 0.7, -0.8, -0.3, 0.0], "duration": 1.0, "type": "moving"}\n'
            self.steps_text.insert("1.0", example)
    
    def open_manual_control(self):
        selected_arm = self.arm_var.get()
        ManualControlDialog(self, selected_arm, self.receive_manual_positions)
    
    def receive_manual_positions(self, positions):
        arm = self.arm_var.get()
        if arm == "both" and isinstance(positions, dict):
            json_obj = {"left_positions": positions["left"], "right_positions": positions["right"],
                        "duration": 1.0, "type": "moving"}
        else:
            json_obj = {"positions": positions, "duration": 1.0, "type": "moving"}
        current_text = self.steps_text.get("1.0", tk.END).strip()
        if current_text:
            new_text = current_text + "\n" + json.dumps(json_obj)
        else:
            new_text = json.dumps(json_obj)
        self.steps_text.delete("1.0", tk.END)
        self.steps_text.insert(tk.END, new_text)
    
    def save_action(self):
        name = self.name_entry.get().strip()
        if not name:
            messagebox.showerror("Error", "Action name cannot be empty.")
            return
        arm = self.arm_var.get()
        try:
            step_delay = float(self.delay_entry.get())
        except ValueError:
            messagebox.showerror("Error", "Invalid delay value.")
            return

        steps_str = self.steps_text.get("1.0", tk.END).strip()
        steps = []
        if steps_str:
            for line in steps_str.splitlines():
                try:
                    step = json.loads(line)
                    if arm == "both":
                        if not (("left_positions" in step and "right_positions" in step) or "positions" in step):
                            raise ValueError("Each step must include 'positions' or both 'left_positions' and 'right_positions'.")
                    else:
                        if "positions" not in step or "duration" not in step or "type" not in step:
                            raise ValueError("Each step must include 'positions', 'duration', and 'type'.")
                    steps.append(step)
                except Exception as e:
                    messagebox.showerror("Error", f"Invalid step format:\n{line}\nError: {e}")
                    return
        new_action = {
            "name": name,
            "arm": arm,
            "steps": steps,
            "motion_mode": self.motion_mode_var.get(),
            "step_delay": step_delay
        }
        actions = self.controller.actions_config
        actions[name] = new_action
        self.controller.update_actions_config(actions)
        messagebox.showinfo("Saved", f"Action '{name}' saved.")
        self.controller.show_frame("ActionConfigPage")

class CenterPositionPage(tk.Frame):
    """
    Displays center position configuration for both arms; users can adjust values and command the arms.
    """
    def __init__(self, parent, controller):
        super().__init__(parent)
        self.controller = controller
        title = tk.Label(self, text="Center Position Configuration", font=("Arial", 18))
        title.pack(pady=10)
        
        center_frame = tk.Frame(self)
        center_frame.pack(pady=10, fill="x", padx=20)
        self.center_entries = {"left": [], "right": []}
        for arm in ["left", "right"]:
            arm_frame = tk.LabelFrame(center_frame, text=f"{arm.capitalize()} Arm", padx=5, pady=5)
            arm_frame.pack(side="left", expand=True, fill="both", padx=10, pady=5)
            for i in range(5):
                lbl = tk.Label(arm_frame, text=f"Joint {i+1}:")
                lbl.grid(row=i, column=0, sticky="e", padx=3, pady=2)
                ent = tk.Entry(arm_frame, width=8)
                ent.grid(row=i, column=1, padx=3, pady=2)
                self.center_entries[arm].append(ent)
        btn_frame = tk.Frame(self)
        btn_frame.pack(pady=10)
        tk.Button(btn_frame, text="Set Center", command=self.set_center).pack(side="left", padx=10)
        tk.Button(btn_frame, text="Save Center", command=self.save_center).pack(side="left", padx=10)
        tk.Button(btn_frame, text="Back", command=lambda: controller.show_frame("StartPage")).pack(side="left", padx=10)
        self.refresh()
    
    def refresh(self):
        for arm in ["left", "right"]:
            positions = self.controller.center_config.get(arm, [0.0]*5)
            for i, ent in enumerate(self.center_entries[arm]):
                ent.delete(0, tk.END)
                ent.insert(0, str(positions[i]))
    
    def set_center(self):
        new_center = {"left": [], "right": []}
        try:
            for arm in ["left", "right"]:
                for ent in self.center_entries[arm]:
                    new_center[arm].append(float(ent.get()))
        except ValueError:
            messagebox.showerror("Invalid Input", "Please enter valid float numbers for all center positions.")
            return
        self.controller.update_center_config(new_center)
        run_center_in_thread(new_center)
        messagebox.showinfo("Center Set", "Center positions commanded.")
    
    def save_center(self):
        new_center = {"left": [], "right": []}
        try:
            for arm in ["left", "right"]:
                for ent in self.center_entries[arm]:
                    new_center[arm].append(float(ent.get()))
        except ValueError:
            messagebox.showerror("Invalid Input", "Please enter valid float numbers for all center positions.")
            return
        self.controller.update_center_config(new_center)
        messagebox.showinfo("Center Saved", "Center positions saved.")

class ManualControlPage(tk.Frame):
    """
    Allows live manual control of each joint using sliders.
    """
    def __init__(self, parent, controller):
        super().__init__(parent)
        self.controller = controller
        self.joint_positions = {joint: tk.DoubleVar(value=0.0) for arm in controller.ARM_JOINT_NAMES for joint in controller.ARM_JOINT_NAMES[arm]}
        title = tk.Label(self, text="Joint State Publisher", font=("Arial", 16, "bold"))
        title.pack(pady=10)
        button_frame = tk.Frame(self)
        button_frame.pack(pady=5)
        tk.Button(button_frame, text="Randomize", command=self.randomize_positions).pack(side="left", padx=5)
        tk.Button(button_frame, text="Center", command=self.set_to_center).pack(side="left", padx=5)
        tk.Button(button_frame, text="Back", command=lambda: controller.show_frame("StartPage")).pack(side="left", padx=5)
        self.joint_sliders = {}
        self.create_sliders()
    
    def create_sliders(self):
        for arm in ["left", "right"]:
            arm_frame = tk.LabelFrame(self, text=f"{arm.capitalize()} Arm", padx=10, pady=10)
            arm_frame.pack(fill="both", expand=True, padx=10, pady=10)
            for joint in self.controller.ARM_JOINT_NAMES[arm]:
                frame = tk.Frame(arm_frame)
                frame.pack(fill="x", pady=3)
                label = tk.Label(frame, text=joint, width=20, anchor="w")
                label.pack(side="left")
                lower, upper = JOINT_LIMITS.get(joint, (-2.0, 2.0))
                slider = tk.Scale(frame, from_=lower, to=upper, resolution=0.01, orient="horizontal", length=200,
                                  variable=self.joint_positions[joint],
                                  command=lambda val, j=joint: self.publish_joint_state(j))
                slider.pack(side="left", padx=5)
                value_label = tk.Label(frame, textvariable=self.joint_positions[joint], width=5)
                value_label.pack(side="left")
                self.joint_sliders[joint] = slider
    
    def publish_joint_state(self, joint):
        positions = [self.joint_positions[j].get() for j in self.controller.ARM_JOINT_NAMES["left"] + self.controller.ARM_JOINT_NAMES["right"]]
        threading.Thread(target=self._ros_publish, args=(positions,), daemon=True).start()
    
    def _ros_publish(self, positions):
        node = Node("manual_control")
        publisher = node.create_publisher(JointState, "/joint_states", 10)
        msg = JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.name = self.controller.ARM_JOINT_NAMES["left"] + self.controller.ARM_JOINT_NAMES["right"]
        msg.position = positions
        publisher.publish(msg)
        time.sleep(0.1)
        node.destroy_node()
    
    def randomize_positions(self):
        import random
        for joint in self.joint_positions:
            lower, upper = JOINT_LIMITS.get(joint, (-1.5, 1.5))
            self.joint_positions[joint].set(round(random.uniform(lower, upper), 2))
        self.publish_joint_state(None)
    
    def set_to_center(self):
        center_config = self.controller.center_config
        for arm in ["left", "right"]:
            for i, joint in enumerate(self.controller.ARM_JOINT_NAMES[arm]):
                self.joint_positions[joint].set(center_config[arm][i])
        self.publish_joint_state(None)

class ManualControlDialog(tk.Toplevel):
    """
    A modal dialog providing manual control sliders; supports single or both arms.
    """
    def __init__(self, parent, arm, callback):
        super().__init__(parent)
        self.title("Manual Control - " + (arm.capitalize() if arm != "both" else "Both Arms"))
        self.callback = callback
        self.arm = arm
        self.slider_vars = {}
        if arm == "both":
            frame_left = tk.LabelFrame(self, text="Left Arm", padx=10, pady=10)
            frame_left.pack(side="left", fill="both", expand=True, padx=10, pady=10)
            frame_right = tk.LabelFrame(self, text="Right Arm", padx=10, pady=10)
            frame_right.pack(side="right", fill="both", expand=True, padx=10, pady=10)
            self.create_sliders(frame_left, "left")
            self.create_sliders(frame_right, "right")
        else:
            frame_arm = tk.LabelFrame(self, text=arm.capitalize() + " Arm", padx=10, pady=10)
            frame_arm.pack(fill="both", expand=True, padx=10, pady=10)
            self.create_sliders(frame_arm, arm)
        tk.Button(self, text="Extract Position", command=self.extract_position).pack(pady=10)
        tk.Button(self, text="Close", command=self.destroy).pack(pady=5)
    
    def create_sliders(self, parent_frame, arm):
        self.slider_vars[arm] = {}
        for joint in ARM_JOINT_NAMES[arm]:
            frame = tk.Frame(parent_frame)
            frame.pack(fill="x", pady=3)
            label = tk.Label(frame, text=joint, width=20, anchor="w")
            label.pack(side="left")
            var = tk.DoubleVar(value=0.0)
            lower, upper = JOINT_LIMITS.get(joint, (-2.0, 2.0))
            slider = tk.Scale(frame, from_=lower, to=upper, resolution=0.01, orient="horizontal",
                              variable=var, length=200, command=lambda val: self.publish_joint_state())
            slider.pack(side="left", padx=5)
            self.slider_vars[arm][joint] = var
    
    def publish_joint_state(self):
        if self.arm == "both":
            left_positions = [self.slider_vars["left"][joint].get() for joint in ARM_JOINT_NAMES["left"]]
            right_positions = [self.slider_vars["right"][joint].get() for joint in ARM_JOINT_NAMES["right"]]
            positions = left_positions + right_positions
            names = ARM_JOINT_NAMES["left"] + ARM_JOINT_NAMES["right"]
        else:
            positions = [self.slider_vars[self.arm][joint].get() for joint in ARM_JOINT_NAMES[self.arm]]
            names = ARM_JOINT_NAMES[self.arm]
        threading.Thread(target=self._ros_publish, args=(names, positions), daemon=True).start()
    
    def _ros_publish(self, names, positions):
        node = Node("manual_control_dialog")
        publisher = node.create_publisher(JointState, "/joint_states", 10)
        msg = JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.name = names
        msg.position = positions
        publisher.publish(msg)
        time.sleep(0.1)
        node.destroy_node()
    
    def extract_position(self):
        if self.arm == "both":
            left_positions = [self.slider_vars["left"][joint].get() for joint in ARM_JOINT_NAMES["left"]]
            right_positions = [self.slider_vars["right"][joint].get() for joint in ARM_JOINT_NAMES["right"]]
            positions = {"left": left_positions, "right": right_positions}
        else:
            positions = [self.slider_vars[self.arm][joint].get() for joint in ARM_JOINT_NAMES[self.arm]]
        self.callback(positions)

# ===============================
# Main and Helper Functions
# ===============================

def run_center_in_thread(center_config):
    threading.Thread(target=run_center_ros, args=("left", center_config["left"]), daemon=True).start()
    threading.Thread(target=run_center_ros, args=("right", center_config["right"]), daemon=True).start()

if __name__ == "__main__":
    rclpy.init(args=None)
    app = ArmControlApp()
    # Start the TCP socket command listener.
    start_command_listener(app)
    # Do NOT automatically start the default action loop.
    app.mainloop()
    rclpy.shutdown()

