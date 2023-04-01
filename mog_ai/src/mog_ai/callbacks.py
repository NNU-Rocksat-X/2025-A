from stable_baselines3.common.callbacks import BaseCallback

class TensorboardCallback(BaseCallback):
    def __init__(self, verbose=1):
        super(TensorboardCallback, self).__init__(verbose)
        self.num_catches = 0

    def _on_step(self) -> bool:
        #print(self.locals)
        reward = self.locals['rewards'][0]
        if self.locals['infos'][0]['catch_success']:
            self.num_catches += 1    


        #info = self.locals['infos'][0]
        #print(self.locals)
        #print("Object position:", self.locals["new_obs"])

        self.logger.record('Reward', reward)
        self.logger.record('motion_punishment', self.locals['infos'][0]['motion_punishment'])
        self.logger.record('Catches', self.num_catches)
        self.logger.record('out_of_reach', self.locals['infos'][0]['out_of_reach'])
        self.logger.record('pre_grasp_plan', self.locals['infos'][0]['pre_grasp_plan'])
        self.logger.record('grasp_plan', self.locals['infos'][0]['grasp_plan'])
        self.logger.record("pointing_reward", self.locals['infos'][0]["pointing_reward"])

        #self.logger.record('Time: ' + info['timing']['func_name'], info['timing']['elapsed_time'])
        #self.logger.record('ADR Difficulty', info['difficulty'])
        return True

