#!/usr/bin/env python3

import gym

MAX_EPISODE_LENGTH = 20

class ApogeeGym(gym.Env):
    def __init__(self, ros_interface):
        self.ros_interface = ros_interface

        # action space: (Trajectory slice, Latent Space)
        self.action_space = gym.spaces.Box(low=0, high=1, shape=(ros_interface.model_params['latent_dim']+1,))

        # Observation space: (Position, Orientation Quaternion, Velocity, Angular Velocity)
        self.observation_space = gym.spaces.Box(low=-1, high=1, shape=(13,))

    def step(self, action):
        print("========= Step " + str(self.current_step) + " ==========")
        reward, grasp_attempt, info = self.ros_interface.perform_action(action)
        observation = self.ros_interface.perform_observation()
        self.current_step += 1

        if self.current_step > MAX_EPISODE_LENGTH:
            done = True
            print("Restarting: Reached max episode!")
        else:
            done = grasp_attempt
            if done == True:
                print("Restarting: Grasp Attempted!")

        return observation, reward, done, info

    def reset(self):
        self.ros_interface.reset_simulation()
        self.current_step = 0
        
        observation = self.ros_interface.perform_observation()
        return observation

    def render(self, mode='', close=False):
        pass

        