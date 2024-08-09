#!/usr/bin/env python3

import sys
import rospy
from PyQt5 import QtWidgets
from robot_launcher_ui import Ui_MainWindow
import os
import math

class RobotLauncher(QtWidgets.QMainWindow):
    def __init__(self):
        super(RobotLauncher, self).__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.ui.launchButton_2.clicked.connect(self.launch_robots)
        self.ui.startMotionButton.clicked.connect(self.start_motion)

    def launch_robots(self):
        num_robots = int(self.ui.numRobotsSpinBox_2.value())
        radius = float(self.ui.doubleSpinBox.value())
        print(f"Launching {num_robots} robots with a radius of {radius}...")
        launch_file_path = self.generate_launch_file(num_robots, radius)
        os.system(f'roslaunch {launch_file_path}')
        self.ui.logTextEdit.append("Launch complete.")


    def generate_launch_file(self, num_robots, radius):
        launch_content = '<?xml version="1.0" encoding="UTF-8"?>\n<launch>\n'
        launch_content += f'<arg name="radius" default="{radius}"/>\n'
        launch_content += '<arg name="yaw_offset" default="0.0"/>\n'

        for i in range(1, num_robots + 1):
            ns = f'robot{i}'
            angle=(i-1)*(2*math.pi/num_robots)
            x = f'$(eval cos({(i-1) * (2 * 3.14 / num_robots)}) * arg(\'radius\'))'
            y = f'$(eval sin({(i-1) * (2 * 3.14 / num_robots)}) * arg(\'radius\'))'
            yaw = f'$(eval {angle} + pi/2 + arg(\'yaw_offset\'))' 
            launch_content += f'''
            <arg name="ns{i}" default="{ns}"/>
            <arg name="x{i}" value="{x}" />
            <arg name="y{i}" value="{y}" />
            <arg name="z{i}" default="0.0" />
            <arg name="yaw{i}" value="{yaw}" />

            <group ns="$(arg ns{i})">
                <param name="tf_prefix" value="$(arg ns{i})_tf" />
                <include file="$(find p3dx_gazebo)/launch/spawn.launch">
                    <arg name="name" value="$(arg ns{i})" />
                    <arg name="x" value="$(arg x{i})" />
                    <arg name="y" value="$(arg y{i})" />
                    <arg name="z" value="$(arg z{i})" />
                    <arg name="yaw" value="$(arg yaw{i})" />
                    <arg name="namespace_arg" value="$(arg ns{i})" />
                </include>
            </group>
            '''
        launch_content += '</launch>\n'
        with open('/tmp/generated_launch_file.launch', 'w') as f:
            f.write(launch_content)
        return '/tmp/generated_launch_file.launch'
    def start_motion(self):
        print("Start Motion button clicked!")
        num_robots = int(self.ui.numRobotsSpinBox_2.value())
        radius = float(self.ui.doubleSpinBox.value())
        self.ui.logTextEdit.append("Starting circular motion for robots...")
        # Launch the circle_movement.py script as a separate ROS node
        os.system(f'rosrun p3dx_gazebo circle_movement.py {num_robots} {radius}')
        print("Circle movement script executed.")

if __name__ == '__main__':
    rospy.init_node('robot_launcher')
    app = QtWidgets.QApplication(sys.argv)
    launcher = RobotLauncher()
    launcher.show()
    sys.exit(app.exec_())
