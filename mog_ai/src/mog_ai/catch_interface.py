import rospy
import numpy as np

from mog_ai.ros_interface import RosInterface
from mog_ai.grasp_vae import GraspVae

from std_srvs.srv import Trigger, TriggerRequest
from daedalus_msgs.srv import ObjectObservation
from daedalus_msgs.srv import GraspDetect
from daedalus_msgs.srv import PredictPosition
from daedalus_msgs.srv import MoveCmd
from mog_ai.srv import GenerateGrasp

# only for testing with gripper
from std_msgs.msg import Float32 
from mujoco_ros.srv import SetBody

class CatchInterface(RosInterface):
    """
    CatchInterface provides an interface between the OpenAI gym environment and ROS.
    """

    def __init__(self):
        super(CatchInterface, self).__init__()

        self.model_params = rospy.get_param("/model_config")

        self.setup_services()

    def setup_services(self):
        rospy.loginfo("Setting up services...")
        rospy.loginfo("Reset")
        rospy.wait_for_service("/mujoco/reset")
        self.reset = rospy.ServiceProxy("/mujoco/reset", Trigger)

        rospy.loginfo("Trajectory predictor")
        rospy.wait_for_service("/trajectory_predictor/ready")
        self.prediction_ready = rospy.ServiceProxy('/trajectory_predictor/ready', Trigger)

        rospy.wait_for_service("/trajectory_predictor/observe")
        self.observe = rospy.ServiceProxy('/trajectory_predictor/observe', ObjectObservation)

        rospy.wait_for_service("/trajectory_predictor/predict")
        self.predict = rospy.ServiceProxy('/trajectory_predictor/predict', PredictPosition)
    
        rospy.loginfo("grasp")
        #rospy.wait_for_service("/ARM1/grasp_cmd")
        #self.grasp_cmd = rospy.ServiceProxy("/ARM1/grasp_cmd", MoveCmd)

        # Allows for testing with just gripper without the rest of arm
        self.grasp_pub = rospy.Publisher("/ARM1/grip_position_cmd", Float32, queue_size=1)
        rospy.wait_for_service("/mujoco/set_body")
        self.set_body = rospy.ServiceProxy("/mujoco/set_body", SetBody)

        rospy.wait_for_service("/ARM1/is_grasped")
        self.is_grasped = rospy.ServiceProxy("/ARM1/is_grasped", GraspDetect)

        rospy.wait_for_service("/mog_ai/generate_grasp")
        self.generate_grasp = rospy.ServiceProxy("/mog_ai/generate_grasp", GenerateGrasp)

    # temporary replacement for grasp_cmd service
    def grasp_cmd(self, cmd):
        if cmd == "close":
            self.grasp_pub.publish(-0.9)
        elif cmd == "open":
            self.grasp_pub.publish(0.8)
        else:
            rospy.logerr("Grasp cmd: " + str(cmd) + " not known.")
        

    # action space (time slice, latent grasp)
    def perform_action(self, action):
        reward = 0
        rospy.loginfo("Action: " +  str(action))

        # Generate grasp in object coordinate frame
        grasp = self.generate_grasp(1, action[1:]).grasp
        rospy.loginfo("grasp: " + str(grasp))

        self.set_body('gripper', grasp)

        rospy.sleep(0.5)

        # reward, move_success, info
        self.grasp_cmd("close") # TODO make this command block
        rospy.sleep(1.5)
        status = self.is_grasped()
        if status.is_grasped:

            reward += 10

        self.grasp_cmd("open")
        rospy.sleep(1)
        info = {}
        return reward, 0, info


    def perform_observation(self):
        poll_rate = rospy.Rate(75)
        attempts = 0

        while not self.prediction_ready().success:
            if attempts > 750:
                self.reset_simulation()
                attempts = 0
            poll_rate.sleep()
            attempts += 1

        raw_observation = self.observe()
        observation = np.empty(shape=(13,))
        rospy.loginfo("observation: " + str(raw_observation))

        observation[0] = raw_observation.pose.position.x
        observation[1] = raw_observation.pose.position.y 
        observation[2] = raw_observation.pose.position.z 
        observation[3] = raw_observation.pose.orientation.x 
        observation[4] = raw_observation.pose.orientation.y 
        observation[5] = raw_observation.pose.orientation.z 
        observation[6] = raw_observation.pose.orientation.w 
        observation[7] = raw_observation.velocity.linear.x
        observation[8] = raw_observation.velocity.linear.y 
        observation[9] = raw_observation.velocity.linear.z 
        observation[10] = raw_observation.velocity.angular.x 
        observation[11] = raw_observation.velocity.angular.y
        observation[12] = raw_observation.velocity.angular.z  

        return observation 


    def reset_simulation(self):
        self.reset()
        rospy.sleep(0.1)