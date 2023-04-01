import rospy
import numpy as np
import math

from mog_ai.ros_interface import RosInterface
from mog_ai.grasp_vae import GraspVae
from mog_ai.quaternion import *
from mog_ai.vector_util import (npPoint, 
                               place_point_on_vector, 
                               shift_point_to_frame,
                               time_to_reach_position,
                               set_frame)

from std_srvs.srv import Trigger, TriggerRequest
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point

from daedalus_msgs.srv import ObjectObservation
from daedalus_msgs.srv import GraspDetect
from daedalus_msgs.srv import PredictPosition
from daedalus_msgs.srv import MoveCmd
from daedalus_msgs.srv import PlanGrasp
from daedalus_msgs.srv import ExecuteGrasp

from mog_ai.srv import GenerateGrasp

# only for testing with gripper
from std_msgs.msg import Float32 
from mujoco_ros.srv import SetBody

GRASP_DISTANCE_THRESHOLD = 4


def object_to_world(obj_grasp: Pose, obj_pose: Pose):
    world = Pose()
    world.position.x = obj_pose.position.x + obj_grasp.position.x 
    world.position.y = obj_pose.position.y + obj_grasp.position.y 
    world.position.z = obj_pose.position.z + obj_grasp.position.z 

    norm_obj_orientation = normalize_quat(obj_pose.orientation)

    # Multiplying it adds complexity. Trying without for now
    #world.orientation = quaternion_multiply(norm_obj_orientation, obj_grasp.orientation)
    world.orientation = obj_grasp.orientation

    return world

def distance(pose1: Pose, pose2: Pose):
    dist = math.sqrt((pose1.position.x - pose2.position.x)**2 +
                     (pose1.position.y - pose2.position.y)**2 +
                     (pose1.position.z - pose2.position.z)**2)
    return dist

def relative_vector(p1: Point, p2: Point):
    p3 = [0, 0, 0]
    p3[0] = p1.x - p2.x
    p3[1] = p1.y - p2.y
    p3[2] = p1.z - p2.z 
    return p3

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2' """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

def calculate_pointing_reward(grasp: Pose, obj_pos: Pose):
    z = [0, 0, 1] # Vector of gripper opening without any rotation
    grasp_vector = rotate(z, grasp.orientation)
    grasp_vector = unit_vector(grasp_vector)
    obj_vec = [obj_pos.position.x, obj_pos.position.y, obj_pos.position.z]
    obj_vec = relative_vector(obj_pos.position, grasp.position)
    obj_vec = unit_vector(obj_vec)
    #rospy.logwarn("g vector: " + str(grasp_vector))
    #rospy.logwarn("obj_vec: " + str(obj_vec))

    diff_angle = angle_between(grasp_vector, obj_vec)

    # Increase reward the closer the angle is to zero
    #rospy.logwarn("diff angle: " + str(diff_angle))
    reward = 2 / diff_angle
    return reward

def adjust_grasp_to_trajectory(grasp: Pose, pos0: Point, pos1: Point):
    grasp_point = np.array([grasp.position.x, grasp.position.y, grasp.position.z])

    trajectory = np.array([[pos0.x, pos0.y, pos0.z], [pos1.x, pos1.y, pos1.z]])

    grasp_point = place_point_on_vector(grasp_point, trajectory)

    grasp.position = npPoint(grasp_point)

    grasp_offset = Point(0, 0, 0.1)

    grasp = shift_point_to_frame(grasp_offset, grasp)

    return grasp, set_frame(grasp, grasp_offset)

class CatchInterface(RosInterface):
    """
    CatchInterface provides an interface between the OpenAI gym environment and ROS.
    """

    def __init__(self):
        super(CatchInterface, self).__init__()

        self.model_params = rospy.get_param("/model_config")

        self.setup_services()

        self.previous_grasp = Pose()

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
    
        rospy.wait_for_service("/trajectory_predictor/reset")
        self.reset_trajectory_predictor = rospy.ServiceProxy('/trajectory_predictor/reset', Trigger)

        rospy.loginfo("grasp")
        rospy.wait_for_service("/ARM1/grasp_cmd")
        self.grasp_cmd = rospy.ServiceProxy("/ARM1/grasp_cmd", MoveCmd)

        rospy.wait_for_service("/ARM1/plan_grasp")
        self.plan_grasp = rospy.ServiceProxy("/ARM1/plan_grasp", PlanGrasp)

        rospy.wait_for_service("/ARM1/execute_grasp")
        self.execute_grasp = rospy.ServiceProxy("/ARM1/execute_grasp", ExecuteGrasp)
        

        rospy.wait_for_service("/ARM1/is_grasped")
        self.is_grasped = rospy.ServiceProxy("/ARM1/is_grasped", GraspDetect)

        rospy.wait_for_service("/mog_ai/generate_grasp")
        self.generate_grasp = rospy.ServiceProxy("/mog_ai/generate_grasp", GenerateGrasp)

        rospy.wait_for_service("/mujoco/reset")
        self.reset_mujoco = rospy.ServiceProxy("/mujoco/reset", Trigger)
    
    def calculate_large_motion_punishment(self, grasp: Pose):
        step_distance = distance(grasp, self.previous_grasp)
        punishment = 1 * step_distance
        self.previous_grasp = grasp
        return punishment

    def wait_for_grasp(self, prediction: Point):
        """
        Watches object until it is at predicted grasp position.
        Stops if object distance increases

        Returns: True if wait finished successfully
                 False if object went out of range
        """
        observation = self.observe()
        time_remaining = time_to_reach_position(observation.pose, prediction, observation.velocity)
        prev_time = GRASP_DISTANCE_THRESHOLD + 1
        distance_increase_persistence = 0

        while (time_remaining > 0.05):
            rospy.sleep(0.01)
            observation = self.observe()
            time_remaining = time_to_reach_position(observation.pose, prediction, observation.velocity)
            rospy.loginfo("Time remaining: " + str(time_remaining))

            if time_remaining > GRASP_DISTANCE_THRESHOLD:
                return False

            if time_remaining > prev_time:
                distance_increase_persistence += 1

            if distance_increase_persistence > 5:
                return False

            prev_time = time_remaining
        
        return True


    # action space (time slice, latent grasp)
    def perform_action(self, action):
        reward = 0
        info = {"catch_success": False,
                "out_of_reach": False,
                "pre_grasp_plan": False,
                "grasp_plan": False,
                "motion_punishment": 0,
                "pointing_reward": 0}
        #rospy.loginfo("Action: " +  str(action))

        prediction = self.predict(action[0])
        rospy.sleep(0.1)

        if not prediction.in_range:
            rospy.loginfo("Object out of reach.")
            info["out_of_reach"] = True
            return 0, True, info

        # Generate grasp in object coordinate frame
        object_grasp = self.generate_grasp(1, action[1:]).grasp
        #rospy.loginfo("Latent grasp: " + str(action[1:]))
        #rospy.loginfo("Grasp orientation: " + str(quat_to_euler(object_grasp.orientation)))

        # Motion punishment
        # Reduce reward for the amount of distance between steps
        grasp = object_to_world(object_grasp, prediction)
        large_motion_punishment = self.calculate_large_motion_punishment(grasp)
        info["motion_punishment"] = large_motion_punishment
        reward -= large_motion_punishment

        # Pointing reward
        # Increase reward as the gripper points closer to the object
        # Could try replacing observation with prediction!
        observation = self.observe()
        pointing_reward = calculate_pointing_reward(grasp, observation.pose)
        info["pointing_reward"] = pointing_reward
        #rospy.loginfo("Point reward: " + str(pointing_reward))
        reward += pointing_reward

        # Puts gripper opening on object path
        #grasp, aiming_point = adjust_grasp_to_trajectory(grasp, observation.pose.position, prediction.position)

        ### Debug
        """
        rospy.sleep(0.2)
        set_body_status = self.set_body("gripper", grasp_plan.Grasp)
        rospy.sleep(0.1)
        if not set_body_status.success:
            rospy.logwarn("Setting body failed!")
        rospy.loginfo("Moving, grasping in: " + str(prediction.prediction_time.data.to_sec() - rospy.get_rostime().to_sec()))
        status = PlanGraspResponse()
        status.pre_grasp_success = True
        status.grasp_success = True
        """

        status = self.plan_grasp(grasp)

        info["pre_grasp_plan"] = status.pre_grasp_success
        info["grasp_plan"] = status.grasp_plan_success

        rospy.loginfo("Pre grasp Plan: " + str(status.pre_grasp_success) + " Grasp Plan: " + str(status.grasp_plan_success))

        if not status.pre_grasp_success or not status.grasp_plan_success:
            reward -= 1
            return reward, False, info
        reward += 2


        rospy.loginfo("Time left: " + str((prediction.prediction_time.data - rospy.get_rostime()).to_sec()))
        if (prediction.prediction_time.data - rospy.get_rostime()).to_sec() < GRASP_DISTANCE_THRESHOLD:
            in_range = self.wait_for_grasp(prediction)

            if in_range:
                grasp_time = 1.0
                status = self.execute_grasp(grasp_time)
            else:
                reward, False, info


            # reward, move_success, info
            if status.grasp_attempted:
                dist_at_grasp = distance(self.observe().pose, grasp)

                rospy.sleep(1.5)
                status = self.is_grasped()
                if status.is_grasped:
                    rospy.logwarn("Successful Grasp!")
                    reward += 50
                else:
                    reward -= dist_at_grasp

                self.grasp_cmd("open")
                info['catch_success'] = status.is_grasped

                return reward, True, info
            else:
                return reward, False, info

        return reward, False, info


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
        #rospy.loginfo("observation: " + str(raw_observation))

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
        rospy.loginfo("Resetting!")
        status = self.reset()
        rospy.sleep(0.1)
        self.reset_trajectory_predictor()
        rospy.sleep(0.5)
        if status.success == False:
            rospy.logerr("Reset Failed: " + status.message)