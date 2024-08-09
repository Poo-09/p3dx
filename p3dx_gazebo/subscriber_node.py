#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import numpy as np
from threading import Lock
import keyboard

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
            theta = 2 * np.arctan2(orientation.z, orientation.w)
            self.theta_data.append(theta)
            
    def get_data(self):
        with self.lock:
            return {
                "time": np.array(self.time_data),
                "x": np.array(self.x_data),
                "y": np.array(self.y_data),
                "theta": np.array(self.theta_data)
            }

def plot_data(robots_data):
    fig, axs = plt.subplots(4, 1, figsize=(10, 8), sharex=True)
    for namespace, data in robots_data.items():
        time = data['time'] - data['time'][0]  
        axs[0].plot(time, data['x'], label=f'{namespace} x vs t')
        axs[1].plot(time, data['y'], label=f'{namespace} y vs t')
        axs[2].plot(time, data['theta'], label=f'{namespace} theta vs t')
        axs[3].plot(data['x'], data['y'], label=f'{namespace} x vs y')
    
    
    axs[0].set_ylabel('x')
    axs[0].set_xlabel('time (s)')
    axs[1].set_ylabel('y')
    axs[1].set_xlabel('time (s)')
    axs[2].set_ylabel('theta')
    axs[2].set_xlabel('time (s)')
    axs[3].set_xlabel('time (s)')
    axs[3].set_ylabel('y')
    
    for ax in axs:
        ax.legend()
        ax.grid(True)
    
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    try:
        
        rospy.init_node('multi_robot_odom_subscriber', anonymous=True)
        namespaces = ['robot1', 'robot2', 'robot3', 'robot4']
        

        robot_odom_subscribers = {ns: RobotOdomSubscriber(ns) for ns in namespaces}
        
        rospy.sleep(35)
        robots_data = {ns: sub.get_data() for ns, sub in robot_odom_subscribers.items()}
        plot_data(robots_data)
    except rospy.ROSInterruptException:
        pass
