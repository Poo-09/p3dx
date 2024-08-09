import tkinter as tk
from tkinter import messagebox
import math
import os

def create_launch_file(num_robots, radius, num_drones, drone_radius):
    with open("dynamic_launch.launch", "w") as f:
        f.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        f.write('<launch>\n')
        f.write('    <arg name="radius" default="{}"/>\n'.format(radius))
        f.write('    <arg name="drone_radius" default="{}"/>\n'.format(drone_radius))
        f.write('    <arg name="yaw_offset" default="0.0"/>\n')
        f.write('    <param name="use_sim_time" value="true"/>\n')

        for i in range(num_robots):
            ns = f"robot{i+1}"
            angle = (2 * math.pi / num_robots) * i
            x = math.cos(angle) * radius
            y = math.sin(angle) * radius

            # Ensure x and y are formatted as strings without scientific notation
            x_str = "{:.6f}".format(x)
            y_str = "{:.6f}".format(y)

            f.write('    <!-- Robot {} -->\n'.format(i+1))
            f.write('    <arg name="ns_robot{}" default="{}"/>\n'.format(i+1, ns))
            f.write('    <arg name="x_robot{}" value="{}" />\n'.format(i+1, x_str))
            f.write('    <arg name="y_robot{}" value="{}" />\n'.format(i+1, y_str))
            f.write('    <arg name="z_robot{}" default="0.0" />\n'.format(i+1))
            f.write('    <arg name="yaw_robot{}" value="$(eval pi/2 + arg(\'yaw_offset\'))" />\n'.format(i+1))
            f.write('    <arg name="include_aruco_marker_robot{}" default="true"/>\n'.format(i+1))
            f.write('    <arg name="robot_model_robot{}" default="$(find p3dx_description)/urdf/pioneer3dx.xacro"/>\n'.format(i+1))
            f.write('    <group ns="$(arg ns_robot{})">\n'.format(i+1))
            f.write('        <param name="tf_prefix" value="$(arg ns_robot{})_tf" />\n'.format(i+1))
            f.write('        <include file="$(find p3dx_gazebo)/launch/spawn.launch">\n')
            f.write('            <arg name="name" value="$(arg ns_robot{})" />\n'.format(i+1))
            f.write('            <arg name="x" value="$(arg x_robot{})" />\n'.format(i+1))
            f.write('            <arg name="y" value="$(arg y_robot{})" />\n'.format(i+1))
            f.write('            <arg name="z" value="$(arg z_robot{})" />\n'.format(i+1))
            f.write('            <arg name="yaw" value="$(arg yaw_robot{})" />\n'.format(i+1))
            f.write('            <arg name="namespace_arg" value="$(arg ns_robot{})" />\n'.format(i+1))
            f.write('            <arg name="robot_model" value="$(arg robot_model_robot{})"/>\n'.format(i+1))
            f.write('        </include>\n')
            f.write('    </group>\n')

        for i in range(num_drones):
            ns = f"drone{i+1}"
            angle = (2 * math.pi / num_drones) * i
            x = math.cos(angle) * drone_radius
            y = math.sin(angle) * drone_radius

            # Ensure x and y are formatted as strings without scientific notation
            x_str = "{:.6f}".format(x)
            y_str = "{:.6f}".format(y)

            f.write('    <!-- Drone {} -->\n'.format(i+1))
            f.write('    <arg name="ns_drone{}" default="{}"/>\n'.format(i+1, ns))
            f.write('    <arg name="x_drone{}" value="{}" />\n'.format(i+1, x_str))
            f.write('    <arg name="y_drone{}" value="{}" />\n'.format(i+1, y_str))
            f.write('    <arg name="z_drone{}" default="1.0" />\n'.format(i+1))
            f.write('    <arg name="yaw_drone{}" value="0.0" />\n'.format(i+1))
            f.write('    <arg name="robot_model_drone{}" default="$(find ardrone_gazebo)/models/ardrone_gazebo/ardrone_gazebo.sdf"/>\n'.format(i+1))
            f.write('    <group ns="$(arg ns_drone{})">\n'.format(i+1))
            f.write('        <param name="tf_prefix" value="$(arg ns_drone{})_tf" />\n'.format(i+1))
            f.write('        <include file="$(find ardrone_gazebo)/launch/single_ardrone.launch">\n')  # Make sure ardrone.launch exists
            f.write('            <arg name="name" value="$(arg ns_drone{})" />\n'.format(i+1))
            f.write('            <arg name="x" value="$(arg x_drone{})" />\n'.format(i+1))
            f.write('            <arg name="y" value="$(arg y_drone{})" />\n'.format(i+1))
            f.write('            <arg name="z" value="$(arg z_drone{})" />\n'.format(i+1))
            f.write('            <arg name="yaw" value="$(arg yaw_drone{})" />\n'.format(i+1))
            f.write('            <arg name="namespace_arg" value="$(arg ns_drone{})" />\n'.format(i+1))
            f.write('            <arg name="robot_model" value="$(arg robot_model_drone{})"/>\n'.format(i+1))
            f.write('        </include>\n')
            f.write('    </group>\n')

        f.write('</launch>\n')

def submit():
    num_robots = int(entry_num_robots.get())
    radius = float(entry_radius.get())
    num_drones = int(entry_num_drones.get())
    drone_radius = float(entry_drone_radius.get())
    create_launch_file(num_robots, radius, num_drones, drone_radius)
    messagebox.showinfo("Success", "Launch file created successfully!")

def launch_file():
    os.system("roslaunch dynamic_launch.launch")

# Create the main window
root = tk.Tk()
root.title("ROS Robot Launcher")

# Create and place the labels and entries
tk.Label(root, text="Number of Robots:").grid(row=0, column=0)
entry_num_robots = tk.Entry(root)
entry_num_robots.grid(row=0, column=1)

tk.Label(root, text="Robot Radius:").grid(row=1, column=0)
entry_radius = tk.Entry(root)
entry_radius.grid(row=1, column=1)

tk.Label(root, text="Number of Drones:").grid(row=2, column=0)
entry_num_drones = tk.Entry(root)
entry_num_drones.grid(row=2, column=1)

tk.Label(root, text="Drone Radius:").grid(row=3, column=0)
entry_drone_radius = tk.Entry(root)
entry_drone_radius.grid(row=3, column=1)

# Create and place the submit button
submit_button = tk.Button(root, text="Create Launch File", command=submit)
submit_button.grid(row=4, column=0, columnspan=2)

# Create and place the launch button
launch_button = tk.Button(root, text="Launch", command=launch_file)
launch_button.grid(row=5, column=0, columnspan=2)

# Start the main loop
root.mainloop()

