#!/usr/bin/env python

import rospy
import smach
import smach_ros

from daedalus_core.daedalus_services.move_cmds import *

"""
Look at namespace for the other arm.
This is used to communicate with the other arm
"""
other_arm = None
if rospy.get_namespace() == "/ARM1/":
    other_arm = "/ARM2/"
else:
    other_arm = "/ARM1/"

# ---------------------------------------------------------------- #
#                      Move State Templates                        #
# ---------------------------------------------------------------- #

class Move_State(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Success', 'Fail'])

class Joint_Pose_State(smach.State):
    """
    Takes pose name as parameter and executes, transitions to next state when done.

    Usage example:
        smach.StateMachine.add('Pre_Throw', Joint_Pose_State("pre_throw"),
                  transitions={'Success': 'Jetson_Sync_1',
                               'Fail': 'Ball_Status'})
    """
    def __init__(self, pose):
        smach.State.__init__(self, outcomes=['Success', 'Fail'])
        self.pose = pose

    def execute(self, userdata):
        complete = False
        try:
            complete = joint_pose_cmd(self.pose).done
        except Exception as e:
            try:
                rospy.logwarn("Joint Pose Cmd Err: " + str(e))
                # try again
                complete = joint_pose_cmd(self.pose).done
            except Exception as e:
                rospy.logwarn("Retrying move failed.")

        if complete:
            return 'Success'
        else:
            return 'Fail'

class Grasp_Cmd_State(smach.State):
    """
    Takes grasp cmd ('open', 'close', etc) as input.
    Executes then transitions when done

    Usage Example:

    smach.StateMachine.add('Grasp_Ball', Grasp_Cmd_State("close"),
                        transitions={'Success': 'Unfold',
                                     'Fail': 'Fail'})
    """
    def __init__(self, pose):
        smach.State.__init__(self, outcomes=['Success', 'Fail'])
        self.pose = pose

    def execute(self, userdata):
        complete = grasp_cmd(self.pose).done

        if complete:
            return 'Success'
        else:
            return 'Fail'

class Is_Grasped(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Success', 'Fail'])
    
    def execute(self, userdata):
        status = is_grasped()

        if status.is_grasped:
            return 'Success'
        else:
            return 'Fail'

# ---------------------------------------------------------------- #
#                      Jetson Comm States                          #
# ---------------------------------------------------------------- #

class Jetson_Sync(smach.State):
    def __init__(self, sync_id, timeout=10, poll_period=0.5):
        smach.State.__init__(self, outcomes=['Ready', 'Timeout'])
    
    def execute(self, userdata):
        pass


# ---------------------------------------------------------------- #
#                      Maneuver State Machines                     #
# ---------------------------------------------------------------- #

"""
PICKUP CUBE
    Only one of the arms may be able to execute pickup cube,
    if the arm that isn't capable executes this state machine, it will fail immediately
    the arm must of the 'pre_pickup_cube' and 'pickup_cube' poses defined in poses.yaml
"""
Pickup_Cube_SM = smach.StateMachine(outcomes=["Success", "Fail"])

with Pickup_Cube_SM:
    smach.StateMachine.add('Pre_Pickup', Joint_Pose_State("pre_pickup_cube"),
            transitions={'Success': 'Pickup',
                         'Fail': 'Fail'})

    smach.StateMachine.add('Pickup', Joint_Pose_State("pickup_cube"),
            transitions={'Success': 'Grasp',
                         'Fail': 'Fail'})

    smach.StateMachine.add('Grasp', Grasp_Cmd_State("grasp_cube"),
            transitions={'Success': 'Check_Grasp',
                         'Fail': 'Fail'})

    smach.StateMachine.add('Check_Grasp', Is_Grasped(),
            transitions={'Success': 'Post_Pickup',
                         'Fail': 'Fail'})

    smach.StateMachine.add('Post_Pickup', Joint_Pose_State("pre_pickup_cube"),
            transitions={'Success': 'Success',
                         'Fail': 'Fail'})

"""
PICKUP_WRENCH
    Arm must have 'pre_pickup_wrench' and 'pickup_wrench' defined in poses.yaml
"""
Pickup_Wrench_SM = smach.StateMachine(outcomes=["Success", "Fail"])

with Pickup_Wrench_SM:
    smach.StateMachine.add('Pre_Pickup', Joint_Pose_State("pre_pickup_wrench"),
            transitions={'Success': 'Pickup',
                            'Fail': 'Fail'})

    smach.StateMachine.add('Pickup', Joint_Pose_State("pickup_wrench"),
            transitions={'Success': 'Grasp',
                            'Fail': 'Fail'})

    smach.StateMachine.add('Grasp', Grasp_Cmd_State("close"),
            transitions={'Success': 'Check_Grasp',
                            'Fail': 'Fail'})

    smach.StateMachine.add('Check_Grasp', Is_Grasped(),
            transitions={'Success': 'Post_Pickup',
                         'Fail': 'Fail'})

    smach.StateMachine.add('Post_Pickup', Joint_Pose_State("pre_pickup_wrench"),
            transitions={'Success': 'Success',
                            'Fail': 'Fail'})
