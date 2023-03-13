import rospy
import numpy as np
import math

from mog_ai.ros_interface import RosInterface
from mog_ai.grasp_vae import GraspVae
from mog_ai.quaternion import *

from std_srvs.srv import Trigger, TriggerRequest
from geometry_msgs.msg import Pose
from daedalus_msgs.srv import ObjectObservation
from daedalus_msgs.srv import GraspDetect
from daedalus_msgs.srv import PredictPosition
from daedalus_msgs.srv import MoveCmd
from daedalus_msgs.srv import PlanGrasp, PlanGraspRequest
from mog_ai.srv import GenerateGrasp

# only for testing with gripper
from std_msgs.msg import Float32 
from mujoco_ros.srv import SetBody


def object_to_world(obj_grasp: Pose, obj_pose: Pose):
    world = Pose()
    world.position.x = obj_pose.position.x + obj_grasp.position.x 
    world.position.y = obj_pose.position.y + obj_grasp.position.y 
    world.position.z = obj_pose.position.z + obj_grasp.position.z 
    world.orientation = quaternion_multiply(obj_grasp.orientation, obj_pose.orientation)
    return world

def distance(pose1: Pose, pose2: Pose):
    dist = math.sqrt((pose1.position.x - pose2.position.x)**2 +
                     (pose1.position.y - pose2.position.y)**2 +
                     (pose1.position.z - pose2.position.z)**2)
    return dist

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
        rospy.wait_for_service("/random_reset")
        self.reset = rospy.ServiceProxy("/random_reset", Trigger)

        rospy.loginfo("Trajectory predictor")
        rospy.wait_for_service("/trajectory_predictor/ready")
        self.prediction_ready = rospy.ServiceProxy('/trajectory_predictor/ready', Trigger)

        rospy.wait_for_service("/trajectory_predictor/observe")
        self.observe = rospy.ServiceProxy('/trajectory_predictor/observe', ObjectObservation)

        rospy.wait_for_service("/trajectory_predictor/predict")
        self.predict = rospy.ServiceProxy('/trajectory_predictor/predict', PredictPosition)
    
        rospy.loginfo("grasp")
        rospy.wait_for_service("/ARM1/grasp_cmd")
        self.grasp_cmd = rospy.ServiceProxy("/ARM1/grasp_cmd", MoveCmd)

        rospy.wait_for_service("/ARM1/plan_grasp")
        self.plan_grasp = rospy.ServiceProxy("/ARM1/plan_grasp", PlanGrasp)

        # Allows for testing with just gripper without the rest of arm
        #self.grasp_pub = rospy.Publisher("/ARM1/grip_position_cmd", Float32, queue_size=1)
        #rospy.wait_for_service("/mujoco/set_body")
        #self.set_body = rospy.ServiceProxy("/mujoco/set_body", SetBody)

        #rospy.wait_for_service("/ARM1/is_grasped")
        self.is_grasped = rospy.ServiceProxy("/ARM1/is_grasped", GraspDetect)

        #rospy.wait_for_service("/mog_ai/generate_grasp")
        self.generate_grasp = rospy.ServiceProxy("/mog_ai/generate_grasp", GenerateGrasp)

    # temporary replacement for grasp_cmd service
    """
    def grasp_cmd(self, cmd):
        if cmd == "close":
            self.grasp_pub.publish(-0.9)
        elif cmd == "open":
            self.grasp_pub.publish(0.8)
        else:
            rospy.logerr("Grasp cmd: " + str(cmd) + " not known.")
    """
        

    # action space (time slice, latent grasp)
    def perform_action(self, action):
        reward = 0
        #rospy.loginfo("Action: " +  str(action))

        prediction = self.predict(action[0])
        rospy.loginfo("Prediction: " + str(prediction))
        if prediction.delta_time < 0:
            rospy.loginfo("Object out of reach.")
            return 0, 0, {'catch_success': False}

        # Generate grasp in object coordinate frame
        object_grasp = self.generate_grasp(1, action[1:]).grasp
        rospy.loginfo("obj grasp: " + str(object_grasp))

        grasp = object_to_world(object_grasp, prediction)

        grasp_plan = PlanGraspRequest()
        grasp_plan.Grasp = grasp 
        grasp_plan.time_to_maneuver.data = rospy.Time.from_sec(rospy.get_time() + prediction.delta_time)
        grasp_plan.grasp_time = 1.0

        status = self.plan_grasp(grasp_plan)
        rospy.loginfo("Plan Status: " + str(status))


        # reward, move_success, info
        dist_at_grasp = distance(self.observe().pose, grasp)

        rospy.sleep(1.5)
        status = self.is_grasped()
        if status.is_grasped:
            rospy.logwarn("Successful Grasp!")
            reward += 10
        else:
            reward -= dist_at_grasp

        self.grasp_cmd("open")
        rospy.sleep(1)
        info = {'catch_success': status.is_grasped}
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
        status = self.reset()
        if status.success == False:
            rospy.logerr("Reset Failed: " + status.message)
        rospy.sleep(0.5)