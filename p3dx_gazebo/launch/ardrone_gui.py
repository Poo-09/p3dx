import os
import tkinter as tk
from tkinter import messagebox
import math
import subprocess
import time

MAX_DRONES = 200

# Function to create the launch file
def create_launch_file(num_drones, radius):
    with open("dynamic_launch_ardrone.launch", "w") as f:
        f.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        f.write('<launch>\n')
        f.write('    <param name="/use_sim_time" value="true"/>\n')
        f.write('    <env name="GAZEBO_MODEL_PATH" value="$GAZEBO_MODEL_PATH:$(find ardrone_gazebo)/models"/>\n')
        f.write('    <env name="GAZEBO_PLUGIN_PATH" value="$GAZEBO_PLUGIN_PATH:$(find ardrone_gazebo)/plugins"/>\n')
        f.write('    <env name="GAZEBO_RESOURCE_PATH" value="$GAZEBO_RESOURCE_PATH:$(find ardrone_gazebo)/meshes"/>\n')
        f.write('    <arg name="world_name" default=""/>\n')
        f.write('    <arg name="paused" value="false"/>\n')
        f.write('    <arg name="verbose" value="true"/>\n')
        f.write('    <arg name="gui" value="true"/>\n')
        f.write('    <include file="$(find gazebo_ros)/launch/empty_world.launch">\n')
        f.write('        <arg name="paused" value="$(arg paused)"/>\n')
        f.write('        <arg name="world_name" value="$(arg world_name)"/>\n')
        f.write('        <arg name="verbose" value="$(arg verbose)"/>\n')
        f.write('        <arg name="gui" value="$(arg gui)"/>\n')
        f.write('    </include>\n')

        for i in range(num_drones):
            ns = f"ardrone{i+1}"
            angle = (2 * math.pi / num_drones) * i
            x = math.cos(angle) * radius
            y = math.sin(angle) * radius
            yaw = angle + math.pi / 2

            x_str = "{:.6f}".format(x)
            y_str = "{:.6f}".format(y)
            yaw_str = "{:.6f}".format(yaw)

            f.write('    <!-- AR.Drone {} -->\n'.format(i+1))
            f.write('    <arg name="ns{}" default="{}"/>\n'.format(i+1, ns))
            f.write('    <arg name="x{}" value="{}" />\n'.format(i+1, x_str))
            f.write('    <arg name="y{}" value="{}" />\n'.format(i+1, y_str))
            f.write('    <arg name="z{}" default="0.0" />\n'.format(i+1))
            f.write('    <arg name="yaw{}" value="{}" />\n'.format(i+1, yaw_str))
            f.write('    <arg name="sdf_robot_file{}" value="$(find ardrone_gazebo)/models/ardrone_gazebo/ardrone_gazebo.sdf"/>\n'.format(i+1))
            f.write('    <group ns="$(arg ns{})">\n'.format(i+1))
            f.write('        <node name="$(arg ns{})_spawn_urdf" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen"\n'.format(i+1))
            f.write('              args="-file $(arg sdf_robot_file{}) -sdf -x $(arg x{}) -y $(arg y{}) -z $(arg z{}) -Y $(arg yaw{}) -model $(arg ns{})"/>\n'.format(i+1, i+1, i+1, i+1, i+1, i+1))
            f.write('    </group>\n')

        f.write('</launch>\n')

# Function to handle the submit button
def submit():
    num_drones = int(entry_num_drones.get())
    if num_drones > MAX_DRONES:
        messagebox.showerror("Error", f"The maximum number of drones is {MAX_DRONES}.")
        return
    
    radius = float(entry_radius.get())
    create_launch_file(num_drones, radius)
    messagebox.showinfo("Success", "Launch file created successfully!")
    start_launch_file()

# Function to start the launch file
def start_launch_file():
    global launch_process
    launch_process = subprocess.Popen(['roslaunch', 'dynamic_launch_ardrone.launch'])
    time.sleep(10)  # Wait for the drones to be fully spawned in the simulation

# Function to stop the launch file
def stop_launch_file():
    if 'launch_process' in globals():
        launch_process.terminate()
        messagebox.showinfo("Stopped", "Launch file stopped.")

# Function to start the takeoff script
def start_takeoff():
    global takeoff_process
    num_drones = int(entry_num_drones.get())
    takeoff_height = float(entry_takeoff_height.get())
    script_path = os.path.join(os.path.dirname(__file__), 'takeoff.py')
    takeoff_process = subprocess.Popen(['python3', script_path, str(num_drones), str(takeoff_height)])

# Function to stop the takeoff process
def stop_takeoff():
    if 'takeoff_process' in globals():
        takeoff_process.terminate()

# Function to start the circle movement script
def start_circle_movement():
    global movement_process
    num_drones = int(entry_num_drones.get())
    radius = float(entry_radius.get())
    script_path = os.path.join(os.path.dirname(__file__), 'circle.py')
    movement_process = subprocess.Popen(['python3', script_path, str(num_drones), str(radius)])

# Function to stop the circle movement script
def stop_circle_movement():
    if 'movement_process' in globals():
        movement_process.terminate()
        messagebox.showinfo("Stopped", "Robot motion stopped.")

# Function to start the landing script
def start_landing():
    global landing_process
    num_drones = int(entry_num_drones.get())
    script_path = os.path.join(os.path.dirname(__file__), 'land.py')
    landing_process = subprocess.Popen(['python3', script_path, str(num_drones)])

# Create the main window
root = tk.Tk()
root.title("ROS Drone Launcher")

# Set window size
root.geometry("400x400")

# Create and place the labels and entries
header = tk.Label(root, text="ROS Drone Launcher", font=("Helvetica", 16))
header.pack(pady=10)

frame = tk.Frame(root)
frame.pack(pady=10)

tk.Label(frame, text="Enter the Number of Drones to be launched:").grid(row=0, column=0, padx=10, pady=5, sticky="e")
entry_num_drones = tk.Entry(frame)
entry_num_drones.grid(row=0, column=1, pady=5)

tk.Label(frame, text="Radius (in m):").grid(row=1, column=0, padx=10, pady=5, sticky="e")
entry_radius = tk.Entry(frame)
entry_radius.grid(row=1, column=1, pady=5)

tk.Label(frame, text="Takeoff Height (in m):").grid(row=2, column=0, padx=10, pady=5, sticky="e")
entry_takeoff_height = tk.Entry(frame)
entry_takeoff_height.grid(row=2, column=1, pady=5)

# Create and place the buttons
button_frame = tk.Frame(root)
button_frame.pack(pady=20)

submit_button = tk.Button(button_frame, text="Create and Launch", command=submit, width=25, bg="lightblue")
submit_button.grid(row=0, column=0, padx=5, pady=5)

takeoff_button = tk.Button(button_frame, text="Takeoff", command=start_takeoff, width=25, bg="lightgreen")
takeoff_button.grid(row=1, column=0, padx=5, pady=5)

start_motion_button = tk.Button(button_frame, text="Start Motion in Circular Trajectory", command=start_circle_movement, width=25, bg="lightgreen")
start_motion_button.grid(row=2, column=0, padx=5, pady=5)

stop_motion_button = tk.Button(button_frame, text="Stop Motion", command=stop_circle_movement, width=25, bg="lightcoral")
stop_motion_button.grid(row=3, column=0, padx=5, pady=5)

land_button = tk.Button(button_frame, text="Land", command=start_landing, width=25, bg="lightyellow")
land_button.grid(row=4, column=0, padx=5, pady=5)

# Start the main loop
root.mainloop()

