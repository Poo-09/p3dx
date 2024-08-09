#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import numpy as np
from threading import Lock, Thread
import sys
import select
from tf.transformations import euler_from_quaternion

class RobotOdomSubscriber:
    def __init__(self, namespace):
        self.namespace = namespace
        self.x_data = []
        self.y_data = []
        self.theta_data = []
        self.time_data = []
        self.lock = Lock()
        self.subscriber = rospy.Subscriber(f'/{namespace}/odom', Odometry, self.callback)
        
    def callback(self, data):
        with self.lock:
            current_time = rospy.Time.now().to_sec()
            self.time_data.append(current_time)
            self.x_data.append(data.pose.pose.position.x)
            self.y_data.append(data.pose.pose.position.y)
            
            orientation = data.pose.pose.orientation
            quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
            _, _, yaw = euler_from_quaternion(quaternion)
            self.theta_data.append(yaw)
    def get_data(self):
        with self.lock:
            return {
                "time": np.array(self.time_data),
                "x": np.array(self.x_data),
                "y": np.array(self.y_data),
                "theta": np.array(self.theta_data)
            }

def plot_data(robots_data):
    
    for namespace, data in robots_data.items():
        fig, axs = plt.subplots(4, 1, figsize=(10, 12), sharex=True)
        time = data['time'] - data['time'][0]  
        axs[0].plot(time, data['x'], label=f'{namespace} x vs t')
        axs[1].plot(time, data['y'], label=f'{namespace} y vs t')
        axs[2].plot(time, data['theta'], label=f'{namespace} theta vs t')
        axs[3].plot(data['x'], data['y'], label=f'{namespace} x vs y')
    
    axs[0].set_ylabel('x (m)')
    axs[0].set_xlabel('time (s)')
    axs[1].set_ylabel('y (m)')
    axs[1].set_xlabel('time (s)')
    axs[2].set_ylabel('theta (radian)')
    axs[2].set_xlabel('time (s)')
    axs[3].set_xlabel('x (m)')
    axs[3].set_ylabel('y (m)')
    
    for ax in axs:
        ax.legend()
        ax.grid(True)
    
    plt.tight_layout()
    plt.suptitle(f'Data for {namespace}', y=1.02, fontsize=16)
    plt.show()

def wait_for_key(key):
    print(f"Press '{key}' to plot the data.")
    while not rospy.is_shutdown():
        if select.select([sys.stdin], [], [], 0)[0]:
            input_key = sys.stdin.read(1)
            if input_key == key:
                return

if __name__ == '__main__':
    try:
        rospy.init_node('multi_robot_odom_subscriber', anonymous=True)
        namespaces = ['robot1', 'robot2', 'robot3', 'robot4']
        
        robot_odom_subscribers = {ns: RobotOdomSubscriber(ns) for ns in namespaces}
        
        key_thread = Thread(target=wait_for_key, args=('p',))
        key_thread.start()
        key_thread.join()

        rospy.loginfo("Key 'p' pressed. Plotting data.")
        robots_data = {ns: sub.get_data() for ns, sub in robot_odom_subscribers.items()}
        plot_data(robots_data)
    except rospy.ROSInterruptException:
        pass

