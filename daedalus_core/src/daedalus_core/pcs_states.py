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

"""
=============================================================================
                            MOVE STATE TEMPLATES
=============================================================================
"""

class Move_State(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Success', 'Fail'])

class Joint_Pose_State(smach.State):
    """
    Takes pose name as parameter and executes, transitions to next state when done.

    USAGE EXAMPLE
        smach.StateMachine.add('Pre_Throw', Joint_Pose_State("pre_throw"),
                  transitions={'Success': 'Jetson_Sync_1',
                               'Fail': 'Ball_Status'})
    """
    def __init__(self, pose, allowed_attempts=1):
        smach.State.__init__(self, outcomes=['Success', 'Fail'])
        self.pose = pose
        self.allowed_attempts = allowed_attempts

    def execute(self, userdata):
        complete = False
        attempts = 0

        while not complete and attempts < self.allowed_attempts:
            complete = joint_pose_cmd(self.pose).done

        if complete:
            return 'Success'
        else:
            return 'Fail'

class Grasp_Cmd_State(smach.State):
    """
    Executes grasp cmd then transitions when done

    INPUT
        grasp cmd ('open', 'close', etc)
    USAGE EXAMPLE
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

# TODO: Update mujoco ros, currently is_grasped returns true if either arm has object grasped
class Is_Grasped(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Success', 'Fail'])
    
    def execute(self, userdata):
        status = is_grasped()

        if status.is_grasped:
            return 'Success'
        else:
            return 'Fail'

"""
=============================================================================
                            JESTON COMM STATES
=============================================================================
"""

class ARM_Sync(smach.State):
    """
    Allows both arms to continue at the same time.
    Both arms wait until the other arm reaches a state with the same sync_id
    Transitions to ready if both arms reach state within timeout
    the sync_id is communicated through the rosparam server

    INPUT
        sync_id - a string or value matching the sync_id of the other arm
        timeout - Number of seconds before transitioning to timeout
        poll_period - The period in seconds to poll the state of the other arm

    USAGE EXAMPLE
        smach.StateMachine.add('Jetson_Sync_TE', Jetson_Sync("TE", timeout=60),
                                transitions={'Ready': 'Unlatch',
                                             'Timeout': 'Signal_Err'})
    """
    def __init__(self, sync_id, timeout=10, poll_period=0.1):
        smach.State.__init__(self, outcomes=['Ready', 'Timeout'])

        self.sync_id = sync_id
        self.timeout = timeout
        self.poll_period = poll_period
    
    
    def execute(self, userdata):
        rospy.loginfo("Syncing Jets: " + str(self.sync_id))

        time_elapsed = 0
        other_sync_param = other_arm + 'sync_id'

        rospy.set_param('sync_id', self.sync_id)

        # Wait until other arm has a sync_id set
        while not rospy.has_param(other_sync_param):
            rospy.logwarn("Waiting for other arm to init...")

            rospy.sleep(self.poll_period)
            time_elapsed += self.poll_period
            if time_elapsed > self.timeout:
                return 'Timeout'

        # Wait for other arm to reach corresponding sync state
        while rospy.get_param(other_sync_param) != self.sync_id:
            rospy.sleep(self.poll_period)
            time_elapsed += self.poll_period
            if time_elapsed > self.timeout:
                return 'Timeout'

        return 'Ready'



"""
=============================================================================
                            MANEUVER STATE MACHINES
=============================================================================
"""

"""
------------------------------------------------------------------------------
                                PICKUP CUBE
------------------------------------------------------------------------------

- Only one of the arms may be able to execute pickup cube,
  if the arm that isn't capable executes this state machine, it will fail immediately
- The arm must have the 'pre_pickup_cube' and 'pickup_cube' poses defined in poses.yaml

"""
Pickup_Cube_SM = smach.StateMachine(outcomes=["Success", "Fail"])

with Pickup_Cube_SM:
    smach.StateMachine.add('Pre_Pickup', Joint_Pose_State("pre_pickup_cube"),
            transitions={'Success': 'Open_Gripper',
                         'Fail': 'Fail'})

    smach.StateMachine.add('Open_Gripper', Grasp_Cmd_State("open"),
                            transitions={'Success': 'Arm_Sync_Grasp',
                                        'Fail': 'Fail'})

    # Just for demo, you probably don't want to always wait before grasping
    smach.StateMachine.add('Arm_Sync_Grasp', ARM_Sync("pickup"),
            transitions={'Ready': 'Pickup',
                         'Timeout': 'Fail'})

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
------------------------------------------------------------------------------
                                PICKUP_WRENCH
------------------------------------------------------------------------------

- Arm must have 'pre_pickup_wrench' and 'pickup_wrench' defined in poses.yaml
"""
Pickup_Wrench_SM = smach.StateMachine(outcomes=["Success", "Fail"])

with Pickup_Wrench_SM:
    smach.StateMachine.add('Pre_Pickup', Joint_Pose_State("pre_pickup_wrench"),
            transitions={'Success': 'Open_Gripper',
                            'Fail': 'Fail'})

    smach.StateMachine.add('Open_Gripper', Grasp_Cmd_State("open"),
                        transitions={'Success': 'Arm_Sync_Grasp',
                                    'Fail': 'Fail'})

    # Just for demo, you probably don't want to always wait before grasping
    smach.StateMachine.add('Arm_Sync_Grasp', ARM_Sync("pickup"),
            transitions={'Ready': 'Pickup',
                         'Timeout': 'Fail'})

    smach.StateMachine.add('Pickup', Joint_Pose_State("pickup_wrench"),
            transitions={'Success': 'Grasp',
                            'Fail': 'Fail'})

    smach.StateMachine.add('Grasp', Grasp_Cmd_State("grasp_cube"),
            transitions={'Success': 'Check_Grasp',
                            'Fail': 'Fail'})

    smach.StateMachine.add('Check_Grasp', Is_Grasped(),
            transitions={'Success': 'Post_Pickup',
                         'Fail': 'Fail'})

    smach.StateMachine.add('Post_Pickup', Joint_Pose_State("pre_pickup_wrench"),
            transitions={'Success': 'Success',
                            'Fail': 'Fail'})

"""
------------------------------------------------------------------------------
                                UNFOLDING
------------------------------------------------------------------------------

- Loops through however many steps are in folding section of poses.yaml
- Starts at step_0 and goes until step_n
- Uses allowed attempts to automatically retry a step
- It would take some effort, but may want to retry by going to previous step
  then continue to the next step if successful
"""

Unfold_SM = smach.StateMachine(outcomes=['Success', 'Fail'])
NUM_FOLDING_STEPS = len(rospy.get_param('joints/folding'))

with Unfold_SM:
    for i in range(0, NUM_FOLDING_STEPS):
        step_str = 'step_' + str(i)

        if i == NUM_FOLDING_STEPS - 1:
            smach.StateMachine.add(step_str, Joint_Pose_State('folding/' + step_str, allowed_attempts=2),
                    transitions={'Success': 'Success',
                                 'Fail': 'Fail'})

        else:
            smach.StateMachine.add(step_str, Joint_Pose_State('folding/' + step_str, allowed_attempts=2),
                    transitions={'Success': 'step_' + str(i+1),
                                 'Fail': 'Fail'})

"""
------------------------------------------------------------------------------
                                FOLDING
------------------------------------------------------------------------------

- Loops through however many steps are in folding section of poses.yaml
- Starts at step_n and goes in reverse until step_0
- Uses NUM_FOLDING_STEPS defined for unfolding
"""

Fold_SM = smach.StateMachine(outcomes=['Success', 'Fail'])

with Fold_SM:
    for i in range(NUM_FOLDING_STEPS-1, -1):
        step_str = 'step_' + str(i)

        if i == 0:
            smach.StateMachine.add(step_str, Joint_Pose_State('folding/' + step_str, allowed_attempts=2),
                    transitions={'Success': 'Success',
                                 'Fail': 'Fail'})

        else:
            smach.StateMachine.add(step_str, Joint_Pose_State('folding/' + step_str, allowed_attempts=2),
                    transitions={'Success': 'step_' + str(i-1),
                                 'Fail': 'Fail'})
