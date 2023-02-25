#!/usr/bin/env python3

import gym

class ApogeeGym(gym.Env):
    def __init__(self, ros_interface):
        self.ros_interface = ros_interface

    def step(self, action):
        reward, move_success, info = self.ros_interface.perform_action(action)
        observation = self.ros_interface.perform_observation()

        self.current_step += 1

        if self.current_step > MAX_EPISODE_LENGTH:
            done = True
        else:
            done = move_success

        return observation, reward, done, info

    def reset(self):
        self.ros_interface.reset_simulation()
        self.current_step = 0
        
        observation = self.ros_interface.perform_observation()
        return observation

    def render(self, mode='', close=False):
        pass

        