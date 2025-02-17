import tkinter as tk
from tkinter import messagebox
import time
import rclpy
from rclpy.node import Node
import yaml
import os
import subprocess
import re
from termcolor import colored

class RosBagRecorder(Node):
  def __init__(self, master):
    super().__init__('rosbag_recorder')
    self.master = master
    self.master.title("ROS Bag Recorder")
    self.master.geometry("300x200")
    
    self.record_button = tk.Button(master, text="Record", command=self.start_recording)
    self.record_button.pack(pady=10)
    
    self.stop_button = tk.Button(master, text="Stop", command=self.stop_recording)
    self.stop_button.pack(pady=10)
    
    self.time_label = tk.Label(master, text="Time: 0 ms")
    self.time_label.pack(pady=10)
    
    self.recording = False
    self.start_time = None
    self.process = None

    # Load configuration
    with open('/root/ros2-ws/src/koch_ros2_wrapper/config/record.yaml', 'r') as file:
      self.config = yaml.safe_load(file)
    
    self.record_time_s = self.config['record_settings']['record_time_s']
    self.bags_folder_path = self.config['record_settings']['bags_folder_path']
    self.topics = [
      "/left_follower/joint_states",
      "/right_follower/joint_states",
      "/left_leader/joint_states",
      "/right_leader/joint_states"
    ]

    # Add camera topics if enabled
    if 'enable_cameras' in self.config:
      for camera_name in self.config['enable_cameras']['name']:
        self.topics.append(f"/{camera_name}/camera/color/image_raw/compressed")
        self.topics.append(f"/{camera_name}/camera/color/camera_info")

    # Print topics that will be recorded
    print(colored("[Topics Recorded]", "green"))
    for topic in self.topics:
      print(colored('\t- ' + topic, "green"))

  def get_next_bag_index(self):
    existing_files = os.listdir(self.bags_folder_path)
    max_index = 0
    for filename in existing_files:
      match = re.match(r"(\d+)_", filename)
      if match:
        index = int(match.group(1))
        if index > max_index:
          max_index = index
    return max_index + 1

  def start_recording(self):
    if not self.recording:
      self.recording = True
      self.start_time = time.time()
      self.bag_index = self.get_next_bag_index()
      bag_filename = os.path.join(self.bags_folder_path, f"{self.bag_index}_{time.strftime('%y-%m-%d')}")
      command = ['ros2', 'bag', 'record', '-o', bag_filename] + self.topics
      self.process = subprocess.Popen(command)
      print(colored("\n[ROS bag file path]", "green"))
      print(colored('\t- ' + bag_filename + '\n', "green"))
      self.update_time()

  def stop_recording(self):
    if self.recording:
      self.recording = False
      if self.process:
        self.process.terminate()
        self.process.wait()
      self.time_label.config(text="Time: 0 ms")
      print(colored("[Episode Done] Recording stopped and bag file saved.", "light_cyan"))
      messagebox.showinfo("Info", "Recording stopped and bag file saved.")
    else:
      self.master.quit()

  def update_time(self):
    if self.recording:
      elapsed_time = int((time.time() - self.start_time) * 1000)
      self.time_label.config(text=f"Time: {elapsed_time} ms")
      if elapsed_time >= self.record_time_s * 1000:
        self.stop_recording()
      else:
        self.master.after(10, self.update_time)

if __name__ == "__main__":
    rclpy.init()
    root = tk.Tk()
    app = RosBagRecorder(root)
    root.mainloop()
    rclpy.shutdown()
