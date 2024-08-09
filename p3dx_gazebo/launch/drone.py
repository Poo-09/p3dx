import os
import tkinter as tk
from tkinter import messagebox
import math
import subprocess

MAX_ROBOTS = 200

# Function to create the launch file
def create_launch_file(num_ground_robots, ground_radius, num_ardrones, ardrone_radius):
    with open("dynamic_launch.launch", "w") as f:
        f.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        f.write('<launch>\n')
        f.write('    <param name="use_sim_time" value="true"/>\n')

        # Ground robots configuration
        for i in range(num_ground_robots):
            ns = f"ground_robot{i+1}"
            angle = (2 * math.pi / num_ground_robots) * i
            x = math.cos(angle) * ground_radius
            y = math.sin(angle) * ground_radius
            yaw = angle + math.pi / 2

            x_str = "{:.6f}".format(x)
            y_str = "{:.6f}".format(y)
            yaw_str = "{:.6f}".format(yaw)

            f.write('    <!-- Ground Robot {} -->\n'.format(i+1))
            f.write('    <arg name="ground_ns{}" default="{}"/>\n'.format(i+1, ns))
            f.write('    <arg name="ground_x{}" value="{}" />\n'.format(i+1, x_str))
            f.write('    <arg name="ground_y{}" value="{}" />\n'.format(i+1, y_str))
            f.write('    <arg name="ground_z{}" default="0.0" />\n'.format(i+1))
            f.write('    <arg name="ground_yaw{}" value="{}" />\n'.format(i+1, yaw_str))
            f.write('    <arg name="ground_include_aruco_marker{}" default="true"/>\n'.format(i+1))
            f.write('    <arg name="ground_robot_model{}" default="$(find p3dx_description)/urdf/pioneer3dx.xacro"/>\n'.format(i+1))
            f.write('    <group ns="$(arg ground_ns{})">\n'.format(i+1))
            f.write('        <param name="tf_prefix" value="$(arg ground_ns{})_tf" />\n'.format(i+1))
            f.write('        <include file="$(find p3dx_gazebo)/launch/spawn.launch">\n')
            f.write('            <arg name="name" value="$(arg ground_ns{})" />\n'.format(i+1))
            f.write('            <arg name="x" value="$(arg ground_x{})" />\n'.format(i+1))
            f.write('            <arg name="y" value="$(arg ground_y{})" />\n'.format(i+1))
            f.write('            <arg name="z" value="$(arg ground_z{})" />\n'.format(i+1))
            f.write('            <arg name="yaw" value="$(arg ground_yaw{})" />\n'.format(i+1))
            f.write('            <arg name="namespace_arg" value="$(arg ground_ns{})" />\n'.format(i+1))
            f.write('            <arg name="robot_model" value="$(arg ground_robot_model{})"/>\n'.format(i+1))
            f.write('        </include>\n')
            f.write('    </group>\n')

        # AR.Drone configuration
        for i in range(num_ardrones):
            ns = f"ardrone{i+1}"
            angle = (2 * math.pi / num_ardrones) * i
            x = math.cos(angle) * ardrone_radius
            y = math.sin(angle) * ardrone_radius

            x_str = "{:.6f}".format(x)
            y_str = "{:.6f}".format(y)

            f.write('    <!-- AR.Drone {} -->\n'.format(i+1))
            f.write('    <arg name="ardrone_ns{}" default="{}"/>\n'.format(i+1, ns))
            f.write('    <arg name="ardrone_x{}" value="{}" />\n'.format(i+1, x_str))
            f.write('    <arg name="ardrone_y{}" value="{}" />\n'.format(i+1, y_str))
            f.write('    <arg name="ardrone_z{}" default="1.0" />\n'.format(i+1))
            f.write('    <arg name="ardrone_roll{}" default="0.0"/>\n'.format(i+1))
            f.write('    <arg name="ardrone_pitch{}" default="0.0"/>\n'.format(i+1))
            f.write('    <arg name="ardrone_yaw{}" default="0.0"/>\n'.format(i+1))
            f.write('    <group ns="$(arg ardrone_ns{})">\n'.format(i+1))
            f.write('        <param name="tf_prefix" value="$(arg ardrone_ns{})_tf" />\n'.format(i+1))
            f.write('        <include file="$(find ardrone_gazebo)/launch/single_ardrone.launch">\n')
            f.write('            <arg name="namespace_arg" value="$(arg ardrone_ns{})" />\n'.format(i+1))
            f.write('            <arg name="x" value="$(arg ardrone_x{})" />\n'.format(i+1))
            f.write('            <arg name="y" value="$(arg ardrone_y{})" />\n'.format(i+1))
            f.write('            <arg name="z" value="$(arg ardrone_z{})" />\n'.format(i+1))
            f.write('            <arg name="roll" value="$(arg ardrone_roll{})" />\n'.format(i+1))
            f.write('            <arg name="pitch" value="$(arg ardrone_pitch{})" />\n'.format(i+1))
            f.write('            <arg name="yaw" value="$(arg ardrone_yaw{})" />\n'.format(i+1))
            f.write('        </include>\n')
            f.write('    </group>\n')

        f.write('</launch>\n')

# Function to handle the submit button
def submit():
    num_ground_robots = int(entry_num_ground_robots.get())
    if num_ground_robots > MAX_ROBOTS:
        messagebox.showerror("Error", f"The maximum number of ground robots is {MAX_ROBOTS}.")
        return
    
    ground_radius = float(entry_ground_radius.get())
    num_ardrones = int(entry_num_ardrones.get())
    if num_ardrones > MAX_ROBOTS:
        messagebox.showerror("Error", f"The maximum number of AR.Drones is {MAX_ROBOTS}.")
        return
    
    ardrone_radius = float(entry_ardrone_radius.get())
    create_launch_file(num_ground_robots, ground_radius, num_ardrones, ardrone_radius)
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
    num_ground_robots = int(entry_num_ground_robots.get())
    ground_radius = float(entry_ground_radius.get())
    num_ardrones = int(entry_num_ardrones.get())
    ardrone_radius = float(entry_ardrone_radius.get())
    script_path = os.path.join(os.path.dirname(__file__), 'circle_movement.py')
    movement_process = subprocess.Popen(['python3', script_path, str(num_ground_robots), str(ground_radius), str(num_ardrones), str(ardrone_radius)])

# Function to stop the circle movement script
def stop_circle_movement():
    if 'movement_process' in globals():
        movement_process.terminate()
        messagebox.showinfo("Stopped", "Robot motion stopped.")
# Create the main window
root = tk.Tk()
root.title("ROS Robot Launcher")

# Set window size
root.geometry("500x400")

# Create and place the labels and entries
header = tk.Label(root, text="ROS Robot Launcher", font=("Helvetica", 16))
header.pack(pady=10)

frame = tk.Frame(root)
frame.pack(pady=10)

# Ground Robots
tk.Label(frame, text="Enter the Number of Ground Robots to be launched:").grid(row=0, column=0, padx=10, pady=5, sticky="e")
entry_num_ground_robots = tk.Entry(frame)
entry_num_ground_robots.grid(row=0, column=1, pady=5)

tk.Label(frame, text="Ground Robots Radius (in m):").grid(row=1, column=0, padx=10, pady=5, sticky="e")
entry_ground_radius = tk.Entry(frame)
entry_ground_radius.grid(row=1, column=1, pady=5)

# AR.Drones
tk.Label(frame, text="Enter the Number of AR.Drones to be launched:").grid(row=2, column=0, padx=10, pady=5, sticky="e")
entry_num_ardrones = tk.Entry(frame)
entry_num_ardrones.grid(row=2, column=1, pady=5)

tk.Label(frame, text="AR.Drones Radius (in m):").grid(row=3, column=0, padx=10, pady=5, sticky="e")
entry_ardrone_radius = tk.Entry(frame)
entry_ardrone_radius.grid(row=3, column=1, pady=5)

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

