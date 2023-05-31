#!/usr/bin/env python

import rospy

from daedalus_msgs.srv import MoveCmd, GraspDetect

print('waiting for services')

rospy.wait_for_service('joint_pose_cmd')
joint_pose_cmd = rospy.ServiceProxy('joint_pose_cmd', MoveCmd)
print("joint_pose_cmd found")

rospy.wait_for_service('async_joint_pose_cmd')
async_joint_pose_cmd = rospy.ServiceProxy('async_joint_pose_cmd', MoveCmd)
print("async_joint_pose_cmd found")

rospy.wait_for_service('pose_cmd')
pose_cmd = rospy.ServiceProxy('pose_cmd', MoveCmd)
print("pose_cmd found")

rospy.wait_for_service('async_pose_cmd')
async_pose_cmd = rospy.ServiceProxy('async_pose_cmd', MoveCmd)
print("async_pose_cmd found")

rospy.wait_for_service('grasp_cmd')
grasp_cmd = rospy.ServiceProxy('grasp_cmd', MoveCmd)
print("grasp_cmd found")

# rospy.wait_for_service('is_grasped')
# is_grasped = rospy.ServiceProxy('is_grasped', GraspDetect)
# print("is_grasped found")