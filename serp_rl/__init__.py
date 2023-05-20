#!/usr/bin/env python3
import rclpy
from rclpy.publisher import Publisher
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import LaserScan
from flatland_msgs.srv import MoveModel
from flatland_msgs.msg import Collisions

from gym import Env
from gym.spaces import Discrete, Box

from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env

import numpy as np
import time
import threading

class SerpControllerEnv(Node, Env):
    def __init__(self) -> None:
        super().__init__("SerpControllerEnv")

        # Predefined speed for the robot
        linear_speed = 0.5
        angular_speed = 1.57079632679

        self.actions = [(0.0, 0.0), 
                        (linear_speed, 0.0), 
                        (-linear_speed, 0.0), 
                        (0.0, angular_speed), 
                        (0.0, -angular_speed), 
                        (linear_speed, angular_speed), 
                        (linear_speed, -angular_speed)]

        self.end_range = 0.2

        self.n_lidar_sections = 9
        self.lidar_sample = []

        self.distance_to_end = 10.0
        self.collision = False

        self.position = 0
        self.start_positions = [(0.0, 0.0, 1.57079632679), (1.6, 1.6, 3.14159265359)]

        self.step_number = 0
        self.max_steps = 200
                                    
        # **** Create publishers ****
        self.pub:Publisher = self.create_publisher(Twist, "/cmd_vel", 1)
        # ***************************

        # **** Create subscriptions ****
        self.create_subscription(LaserScan, "/static_laser", self.processLiDAR, 1)

        self.create_subscription(LaserScan, "/end_beacon_laser", self.processEndLiDAR, 1)

        self.create_subscription(Collisions, "/collisions", self.processCollisions, 1)
        # ******************************

        self.action_space = Discrete(len(self.actions))

        self.observation_space = Box(0, 2, shape=(self.n_lidar_sections,), dtype=np.float64)

        self.state = np.array(self.lidar_sample)

    def reset(self):
        self.change_speed(self.pub, 0.0, 0.0)

        start_pos = self.start_positions[self.position]
        self.position = 1 - self.position
        end_pos = self.start_positions[self.position]
        
        self.move_model('serp', start_pos[0], start_pos[1], start_pos[2])
        self.move_model('end_beacon', end_pos[0], end_pos[1], 0.0)

        self.wait_lidar_reading()
        self.state = np.array(self.lidar_sample)

        time.sleep(0.1)

        self.lidar_sample = []
        self.distance_to_end = 10.0
        self.collision = False
        self.step_number = 0

        return self.state


    def step(self, action): 
        self.change_speed(self.pub, self.actions[action][0], self.actions[action][1])
        self.step_number += 1

        self.lidar_sample = []
        self.wait_lidar_reading()
        self.change_speed(self.pub, 0.0, 0.0)

        done = False

        if self.collision:
            self.get_logger().info("colision")
            reward = -200
            done = True
        elif self.distance_to_end < self.end_range:
            self.get_logger().info("end")
            reward = 400 
            done = True
        elif self.step_number >= self.max_steps:
            self.get_logger().info("timeout")
            reward = -300 
            done = True
        else:
            reward = 0

        info = {}

        return self.state, reward, done, info

    def render(self): pass

    def close(self): pass

    # Change the speed of the robot
    def change_speed(self, publisher, linear, angular):
        twist_msg = Twist()
        twist_msg.linear.x = linear
        twist_msg.angular.z = angular
        publisher.publish(twist_msg)

    def wait_lidar_reading(self):
        while len(self.lidar_sample) == 0:pass

    # Send a request to move a model
    def move_model(self, model_name, x, y, theta):
        client = self.create_client(MoveModel, "/move_model")
        client.wait_for_service()
        request = MoveModel.Request()
        request.name = model_name
        request.pose = Pose2D()
        request.pose.x = x
        request.pose.y = y
        request.pose.theta = theta
        client.call_async(request)
    
    # Handle LiDAR data
    def processLiDAR(self, data):
        self.lidar_sample = []

        rays = data.ranges
        rays_per_section = len(rays) // self.n_lidar_sections

        for i in range(self.n_lidar_sections - 1):
            self.lidar_sample.append(min(rays[rays_per_section * i:rays_per_section * (i + 1)]))
        self.lidar_sample.append(min(rays[(self.n_lidar_sections - 1) * rays_per_section:]))

    
    # Handle end beacon LiDAR data
    def processEndLiDAR(self, data):
        clean_data = [x for x in data.ranges if str(x) != 'nan']
        self.distance_to_end = min(clean_data)
    
    # Process collisions
    def processCollisions(self, data):
        if len(data.collisions) > 0:
            self.collision = True

    def run_rl_alg(self):
        
        check_env(self)
        self.wait_lidar_reading()

        model = PPO("MlpPolicy", self, verbose=1)
        model.learn(total_timesteps=25000)
        model.save("ppo")

        del model 

        model = PPO.load("ppo")


        obs = self.reset()
        while True:
            action, _states = model.predict(obs)
            obs, rewards, dones, info = self.step(action)

def main(args = None):
    rclpy.init()
    
    serp = SerpControllerEnv()

    thread = threading.Thread(target=serp.run_rl_alg)
    thread.start()

    rclpy.spin(serp)



if __name__ == "__main__":
    main()
