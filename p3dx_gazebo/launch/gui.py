import os
import tkinter as tk
from tkinter import messagebox
import math
import subprocess

MAX_ROBOTS = 200

# Function to create the launch file
def create_launch_file(num_robots, radius):
    with open("dynamic_launch.launch", "w") as f:
        f.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        f.write('<launch>\n')
        f.write('    <arg name="radius" default="{}"/>\n'.format(radius))
        f.write('    <arg name="yaw_offset" default="0.0"/>\n')
        f.write('    <param name="use_sim_time" value="true"/>\n')

        for i in range(num_robots):
            ns = f"robot{i+1}"
            angle = (2 * math.pi / num_robots) * i
            x = math.cos(angle) * radius
            y = math.sin(angle) * radius
            yaw = angle + math.pi / 2

            x_str = "{:.6f}".format(x)
            y_str = "{:.6f}".format(y)
            yaw_str = "{:.6f}".format(yaw)

            f.write('    <!-- Robot {} -->\n'.format(i+1))
            f.write('    <arg name="ns{}" default="{}"/>\n'.format(i+1, ns))
            f.write('    <arg name="x{}" value="{}" />\n'.format(i+1, x_str))
            f.write('    <arg name="y{}" value="{}" />\n'.format(i+1, y_str))
            f.write('    <arg name="z{}" default="0.0" />\n'.format(i+1))
            f.write('    <arg name="yaw{}" value="{}" />\n'.format(i+1, yaw_str))
          
            f.write('    <arg name="robot_model{}" default="$(find p3dx_description)/urdf/pioneer3dx.xacro"/>\n'.format(i+1))
            f.write('    <group ns="$(arg ns{})">\n'.format(i+1))
            f.write('        <param name="tf_prefix" value="$(arg ns{})_tf" />\n'.format(i+1))
            f.write('        <include file="$(find p3dx_gazebo)/launch/spawn.launch">\n')
            f.write('            <arg name="name" value="$(arg ns{})" />\n'.format(i+1))
            f.write('            <arg name="x" value="$(arg x{})" />\n'.format(i+1))
            f.write('            <arg name="y" value="$(arg y{})" />\n'.format(i+1))
            f.write('            <arg name="z" value="$(arg z{})" />\n'.format(i+1))
            f.write('            <arg name="yaw" value="$(arg yaw{})" />\n'.format(i+1))
            f.write('            <arg name="namespace_arg" value="$(arg ns{})" />\n'.format(i+1))
            f.write('            <arg name="robot_model" value="$(arg robot_model{})"/>\n'.format(i+1))
            f.write('        </include>\n')
            f.write('    </group>\n')

        f.write('</launch>\n')

# Function to handle the submit button
def submit():
    num_robots = int(entry_num_robots.get())
    if num_robots > MAX_ROBOTS:
        messagebox.showerror("Error", f"The maximum number of robots is {MAX_ROBOTS}.")
        return
    
    radius = float(entry_radius.get())
    create_launch_file(num_robots, radius)
    messagebox.showinfo("Success", "Launch file created successfully!")
    start_launch_file()

# Function to start the launch file
def start_launch_file():
    global launch_process
    launch_process = subprocess.Popen(['roslaunch', 'dynamic_launch.launch'])

# Function to stop the launch file
def stop_launch_file():
    if 'launch_process' in globals():
        launch_process.terminate()
        messagebox.showinfo("Stopped", "Launch file stopped.")

# Function to start the circle movement script
def start_circle_movement():
    global movement_process
    num_robots = int(entry_num_robots.get())
    radius = float(entry_radius.get())
    script_path = os.path.join(os.path.dirname(__file__), 'circle_movement.py')
    movement_process = subprocess.Popen(['python3', script_path, str(num_robots), str(radius)])

# Function to stop the circle movement script
def stop_circle_movement():
    if 'movement_process' in globals():
        movement_process.terminate()
        messagebox.showinfo("Stopped", "Robot motion stopped.")

# Create the main window
root = tk.Tk()
root.title("ROS Robot Launcher")

# Set window size
root.geometry("400x300")

# Create and place the labels and entries
header = tk.Label(root, text="ROS Robot Launcher", font=("Helvetica", 16))
header.pack(pady=10)

frame = tk.Frame(root)
frame.pack(pady=10)

tk.Label(frame, text="Enter the Number of Robots to be launched:").grid(row=0, column=0, padx=10, pady=5, sticky="e")
entry_num_robots = tk.Entry(frame)
entry_num_robots.grid(row=0, column=1, pady=5)

tk.Label(frame, text="Radius(in m):").grid(row=1, column=0, padx=10, pady=5, sticky="e")
entry_radius = tk.Entry(frame)
entry_radius.grid(row=1, column=1, pady=5)

# Create and place the buttons
button_frame = tk.Frame(root)
button_frame.pack(pady=20)

submit_button = tk.Button(button_frame, text="Create and Launch", command=submit, width=25, bg="lightblue")
submit_button.grid(row=0, column=0, padx=5, pady=5)

start_motion_button = tk.Button(button_frame, text="Start Motion in circular trajectory", command=start_circle_movement, width=25, bg="lightgreen")
start_motion_button.grid(row=1, column=0, padx=5, pady=5)

stop_motion_button = tk.Button(button_frame, text="Stop Motion", command=stop_circle_movement, width=25, bg="lightcoral")
stop_motion_button.grid(row=2, column=0, padx=5, pady=5)

# Start the main loop
root.mainloop()

