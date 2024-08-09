import sys
import xml.etree.ElementTree as ET
from math import cos, sin, pi

def create_robot_launch(num_robots, radius, yaw_offset):
    launch = ET.Element("launch")

    for i in range(num_robots):
        ns = f"robot{i+1}"
        angle = (2 * pi / num_robots) * i
        x = cos(angle) * radius
        y = sin(angle) * radius
        yaw = (pi / 2) + yaw_offset + angle

        group = ET.SubElement(launch, "group", ns=ns)
        param = ET.SubElement(group, "param", name="tf_prefix", value=f"{ns}_tf")
        include = ET.SubElement(group, "include", file="$(find p3dx_gazebo)/launch/spawn.launch")
        ET.SubElement(include, "arg", name="name", value=ns)
        ET.SubElement(include, "arg", name="x", value=str(x))
        ET.SubElement(include, "arg", name="y", value=str(y))
        ET.SubElement(include, "arg", name="z", value="0.0")
        ET.SubElement(include, "arg", name="yaw", value=str(yaw))
        ET.SubElement(include, "arg", name="namespace_arg", value=ns)

    tree = ET.ElementTree(launch)
    tree.write(sys.stdout, encoding='unicode')

if __name__ == "__main__":
    num_robots = int(sys.argv[1])
    radius = float(sys.argv[2])
    yaw_offset = float(sys.argv[3])
    output_file=sys.argv[4]
    create_robot_launch(num_robots, radius, yaw_offset,output_file)
