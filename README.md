# p3dx

Task 1: Task is to launch 4 Pioneer P3-Dx robots in sim env (Ros Gazebo) . After that, you need to write a publisher node to run all these robots in a circular trajectory.
Ins to follow to achieve Task1.
To launch empty world in gazebo
$ roslaunch gazebo_ros empty_world.launch
To launch the four robots in gazebo world
$ roslaunch p3dx_gazebo p3dx.launch
To run all the four robots in a single circle
$ rosrun p3dx_gazebo motion.py

Task 2: To obtain the 2d plots to visualise the path followed by each robot, i.e., x vs t, y vs t, theta vs t and x vs y. 
            Script file to run to obtain the required plots:
            $ rosrun p3dx_gazebo subscriber.py

Task 3: To create the GUI to launch multiple robots in the simulation environment.
 
 Script file to run to achieve the task:
            $ roslaunch gazebo_ros empty_world.launch
            $ python3 gui.py ( by navigating to the folder where the script is located.)
            $ cd src/pioneer_p3dx_model/p3dx_gazebo/launch/














