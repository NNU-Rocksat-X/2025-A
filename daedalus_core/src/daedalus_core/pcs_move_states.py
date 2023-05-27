#!/usr/bin/env python

import rospy
import smach
import smach_ros

from daedalus_core.daedalus_services.move_cmds import *

pickup_fail_1_flag = False


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



"""
--------------------------------------~+=|=+~- Pickup Obj 1 ~+=|=+~---------------------------------------

- ARM1 Picks up Ball 1
- ARM1 moves to the ball, checks to see if its grasped
- Test Ready
"""

pickup_obj_1_SM = smach.StateMachine(outcomes=['Success', 'Fail'])
NUM_PICKUP_OBJ_1_STEPS = len(rospy.get_param('joints/pickup_obj_1'))

with pickup_obj_1_SM:
    for i in range(0, NUM_PICKUP_OBJ_1_STEPS):
        step_str = 'step_' + str(i)

        if i == NUM_PICKUP_OBJ_1_STEPS - 1:
            smach.StateMachine.add(step_str, Joint_Pose_State('pickup_obj_1/' + step_str, allowed_attempts=2),
                    transitions={'Success': 'Success',
                                 'Fail': 'Fail'})

        else:
            smach.StateMachine.add(step_str, Joint_Pose_State('pickup_obj_1/' + step_str, allowed_attempts=2),
                    transitions={'Success': 'step_' + str(i+1),
                                 'Fail': 'Fail'})
            
    smach.StateMachine.add('Grasp', Joint_Pose_State('grasp_cube/'), 
                           transitions={'Success': 'Check_Grasp',
                                        'Fail': 'Try_Again'})

    smach.StateMachine.add('Check_Grasp', Is_Grasped(),             
                           transitions={'Success': 'Post_Pickup',
                                        'Fail': 'Try_Again'})
    
    smach.StateMachine.add('Grasp', Joint_Pose_State('grasp_cube/'), 
                           transitions={'Success': 'Check_Grasp',
                                        'Fail': 'Try_Again'})
    
    smach.StateMachine.add('Try_Again', Is_Grasped(), 
                           transitions={'Success': 'Post_Pickup',
                                        'Fail': 'Fail'})
                           
    smach.StateMachine.add('Post_Pickup', Joint_Pose_State('active_home/'),
                           transitions={'Success': 'Success',
                                        'Fail': 'Fail'})


"""
-------------------------------------~+=|=+~- Handoff Obj 1 ~+=|=+~---------------------------------------

- Moves to Handoff position and waits for Arm_Sync
"""

throw_1_SM = smach.StateMachine(outcomes=['Success', 'Fail'])
NUM_THROW_OBJ_1_STEPS = len(rospy.get_param('joints/throw_obj_1'))

with throw_1_SM:
    smach.StateMachine.add('Move_To_Handy', Joint_Pose_State(),
                           transitions={'Success': 'Wait_For_Sync',
                                        'Fail': 'Fail'})
    
    smach.StateMachine.add('Wait_For_Sync', ARM_Sync('handoff_1'),
                           transitions={'Success': 'Handoff',
                                        'Fail': 'Fail'})
    
    smach.StateMachine.add('Handoff')






"""
-------------------------------------~+=|=+~- Pickup Obj 2 ~+=|=+~---------------------------------------

- ARM2 Function
"""

pickup_obj_2_SM = smach.StateMachine(outcomes=['Success', 'Fail'])
NUM_PICKUP_OBJ_2_STEPS = len(rospy.get_param('joints/pickup_obj_2'))

with pickup_obj_2_SM:
    for i in range(0, NUM_PICKUP_OBJ_2_STEPS):
        step_str = 'step_' + str(i)

        if i == NUM_PICKUP_OBJ_2_STEPS - 1:
            smach.StateMachine.add(step_str, Joint_Pose_State('pickup_obj_2/' + step_str, allowed_attempts=2),
                    transitions={'Success': 'Success',
                                 'Fail': 'Fail'})

        else:
            smach.StateMachine.add(step_str, Joint_Pose_State('pickup_obj_2/' + step_str, allowed_attempts=2),
                    transitions={'Success': 'step_' + str(i+1),
                                 'Fail': 'Fail'})
            
    smach.StateMachine.add('Check_Grasp', Is_Grasped(),
                           transitions={'Success': 'Post_Pickup',
                                        'Fail': 'Fail'})

    smach.StateMachine.add('Post_Pickup', Joint_Pose_State('active_home/'),
                           transitions={'Success': 'Success',
                                        'Fail': 'Fail'})

                                 # same thing here, computer vision to add feedback to this movement


"""
-------------------------------------~+=|=+~- Pickup Obj 3 ~+=|=+~---------------------------------------

- ARM 2 Function
"""

pickup_obj_3_SM = smach.StateMachine(outcomes=['Success', 'Fail'])
NUM_PICKUP_OBJ_3_STEPS = len(rospy.get_param('joints/pickup_obj_2'))

with pickup_obj_3_SM:
    for i in range(0, NUM_PICKUP_OBJ_3_STEPS):
        step_str = 'step_' + str(i)

        if i == NUM_PICKUP_OBJ_3_STEPS - 1:
            smach.StateMachine.add(step_str, Joint_Pose_State('pickup_obj_3/' + step_str, allowed_attempts=2),
                    transitions={'Success': 'Success',
                                 'Fail': 'Fail'})

        else:
            smach.StateMachine.add(step_str, Joint_Pose_State('pickup_obj_3/' + step_str, allowed_attempts=2),
                    transitions={'Success': 'step_' + str(i+1),
                                 'Fail': 'Fail'})

     smach.StateMachine.add()                            


"""
-------------------------------------~+=|=+~- Throw Obj 1 ~+=|=+~---------------------------------------

- Exatly like the unfold, but references throw_obj_1 in poses.yaml
"""

throw_1_SM = smach.StateMachine(outcomes=['Success', 'Fail'])
NUM_THROW_OBJ_1_STEPS = len(rospy.get_param('joints/throw_obj_1'))

with throw_1_SM:
    for i in range(0, NUM_THROW_OBJ_1_STEPS):
        step_str = 'step_' + str(i)

        if i == NUM_THROW_OBJ_1_STEPS - 1:
            smach.StateMachine.add(step_str, Joint_Pose_State('throw_obj_1/' + step_str, allowed_attempts=2),
                    transitions={'Success': 'Success',
                                 'Fail': 'Fail'})

        else:
            smach.StateMachine.add(step_str, Joint_Pose_State('throw_obj_1/' + step_str, allowed_attempts=2),
                    transitions={'Success': 'step_' + str(i+1),
                                 'Fail': 'Fail'})


"""
-------------------------------------~+=|=+~- Throw Obj 2 ~+=|=+~---------------------------------------

- Exatly like the unfold, but references throw_obj_2 in poses.yaml
"""

throw_2_SM = smach.StateMachine(outcomes=['Success', 'Fail'])
NUM_THROW_OBJ_2_STEPS = len(rospy.get_param('joints/throw_obj_1'))

with throw_2_SM:
    for i in range(0, NUM_THROW_OBJ_2_STEPS):
        step_str = 'step_' + str(i)

        if i == NUM_THROW_OBJ_2_STEPS - 1:
            smach.StateMachine.add(step_str, Joint_Pose_State('throw_obj_2/' + step_str, allowed_attempts=2),
                    transitions={'Success': 'Success',
                                 'Fail': 'Fail'})

        else:
            smach.StateMachine.add(step_str, Joint_Pose_State('throw_obj_2/' + step_str, allowed_attempts=2),
                    transitions={'Success': 'step_' + str(i+1),
                                 'Fail': 'Fail'})


"""
-------------------------------------~+=|=+~- Throw Obj 3 ~+=|=+~---------------------------------------

- Exatly like the unfold, but references throw_obj_3 in poses.yaml
"""

throw_3_SM = smach.StateMachine(outcomes=['Success', 'Fail'])
NUM_THROW_OBJ_3_STEPS = len(rospy.get_param('joints/throw_obj_3'))

with throw_3_SM:
    for i in range(0, NUM_THROW_OBJ_3_STEPS):
        step_str = 'step_' + str(i)

        if i == NUM_THROW_OBJ_3_STEPS - 1:
            smach.StateMachine.add(step_str, Joint_Pose_State('throw_obj_3/' + step_str, allowed_attempts=2),
                    transitions={'Success': 'Success',
                                 'Fail': 'Fail'})

        else:
            smach.StateMachine.add(step_str, Joint_Pose_State('throw_obj_3/' + step_str, allowed_attempts=2),
                    transitions={'Success': 'step_' + str(i+1),
                                 'Fail': 'Fail'})


"""
--------------------------------------~+=|=+~- Pickup Obj 1 ~+=|=+~---------------------------------------

- Exatly like the unfold, but references pickup_obj_1 in poses.yaml
- ToDo: add computer vision feedback to this movement
    - only if we need it though. Im not convinced we need it yet but then again we don't know if we need it 
"""

pickup_obj_1_SM = smach.StateMachine(outcomes=['Success', 'Fail'])
NUM_PICKUP_OBJ_1_STEPS = len(rospy.get_param('joints/pickup_obj_1'))

with pickup_obj_1_SM:
    for i in range(0, NUM_PICKUP_OBJ_1_STEPS):
        step_str = 'step_' + str(i)

        if i == NUM_PICKUP_OBJ_1_STEPS - 1:
            smach.StateMachine.add(step_str, Joint_Pose_State('pickup_obj_1/' + step_str, allowed_attempts=2),
                    transitions={'Success': 'Success',
                                 'Fail': 'Fail'})

        else:
            smach.StateMachine.add(step_str, Joint_Pose_State('pickup_obj_1/' + step_str, allowed_attempts=2),
                    transitions={'Success': 'step_' + str(i+1),
                                 'Fail': 'Fail'})

    smach.StateMachine.add('')                             # we need to have something here to implement the computer vision seeing the ball in the gripper?


"""
-------------------------------------~+=|=+~- Pickup Obj 2 ~+=|=+~---------------------------------------

- Exatly like the unfold, but references pickup_obj_2 in poses.yaml
"""

pickup_obj_2_SM = smach.StateMachine(outcomes=['Success', 'Fail'])
NUM_PICKUP_OBJ_2_STEPS = len(rospy.get_param('joints/pickup_obj_2'))

with pickup_obj_2_SM:
    for i in range(0, NUM_PICKUP_OBJ_2_STEPS):
        step_str = 'step_' + str(i)

        if i == NUM_PICKUP_OBJ_2_STEPS - 1:
            smach.StateMachine.add(step_str, Joint_Pose_State('pickup_obj_2/' + step_str, allowed_attempts=2),
                    transitions={'Success': 'Success',
                                 'Fail': 'Fail'})

        else:
            smach.StateMachine.add(step_str, Joint_Pose_State('pickup_obj_2/' + step_str, allowed_attempts=2),
                    transitions={'Success': 'step_' + str(i+1),
                                 'Fail': 'Fail'})

                                 # same thing here, computer vision to add feedback to this movement


"""
-------------------------------------~+=|=+~- Pickup Obj 3 ~+=|=+~---------------------------------------

- Exatly like the unfold, but references pickup_obj_2 in poses.yaml
"""

pickup_obj_3_SM = smach.StateMachine(outcomes=['Success', 'Fail'])
NUM_PICKUP_OBJ_3_STEPS = len(rospy.get_param('joints/pickup_obj_2'))

with pickup_obj_3_SM:
    for i in range(0, NUM_PICKUP_OBJ_3_STEPS):
        step_str = 'step_' + str(i)

        if i == NUM_PICKUP_OBJ_3_STEPS - 1:
            smach.StateMachine.add(step_str, Joint_Pose_State('pickup_obj_3/' + step_str, allowed_attempts=2),
                    transitions={'Success': 'Success',
                                 'Fail': 'Fail'})

        else:
            smach.StateMachine.add(step_str, Joint_Pose_State('pickup_obj_3/' + step_str, allowed_attempts=2),
                    transitions={'Success': 'step_' + str(i+1),
                                 'Fail': 'Fail'})

                                 # Vision Feedback.. Although im starting to think we dont really need it if we can nail the movements here on earth


"""
-------------------------------------~+=|=+~- Throw Obj 1 ~+=|=+~---------------------------------------

- Exatly like the unfold, but references throw_obj_1 in poses.yaml
"""

throw_1_SM = smach.StateMachine(outcomes=['Success', 'Fail'])
NUM_THROW_OBJ_1_STEPS = len(rospy.get_param('joints/throw_obj_1'))

with throw_1_SM:
    for i in range(0, NUM_THROW_OBJ_1_STEPS):
        step_str = 'step_' + str(i)

        if i == NUM_THROW_OBJ_1_STEPS - 1:
            smach.StateMachine.add(step_str, Joint_Pose_State('throw_obj_1/' + step_str, allowed_attempts=2),
                    transitions={'Success': 'Success',
                                 'Fail': 'Fail'})

        else:
            smach.StateMachine.add(step_str, Joint_Pose_State('throw_obj_1/' + step_str, allowed_attempts=2),
                    transitions={'Success': 'step_' + str(i+1),
                                 'Fail': 'Fail'})


"""
-------------------------------------~+=|=+~- Throw Obj 2 ~+=|=+~---------------------------------------

- Exatly like the unfold, but references throw_obj_2 in poses.yaml
"""

throw_2_SM = smach.StateMachine(outcomes=['Success', 'Fail'])
NUM_THROW_OBJ_2_STEPS = len(rospy.get_param('joints/throw_obj_1'))

with throw_2_SM:
    for i in range(0, NUM_THROW_OBJ_2_STEPS):
        step_str = 'step_' + str(i)

        if i == NUM_THROW_OBJ_2_STEPS - 1:
            smach.StateMachine.add(step_str, Joint_Pose_State('throw_obj_2/' + step_str, allowed_attempts=2),
                    transitions={'Success': 'Success',
                                 'Fail': 'Fail'})

        else:
            smach.StateMachine.add(step_str, Joint_Pose_State('throw_obj_2/' + step_str, allowed_attempts=2),
                    transitions={'Success': 'step_' + str(i+1),
                                 'Fail': 'Fail'})


"""
-------------------------------------~+=|=+~- Throw Obj 3 ~+=|=+~---------------------------------------

- Exatly like the unfold, but references throw_obj_3 in poses.yaml
"""

throw_3_SM = smach.StateMachine(outcomes=['Success', 'Fail'])
NUM_THROW_OBJ_3_STEPS = len(rospy.get_param('joints/throw_obj_3'))

with throw_3_SM:
    for i in range(0, NUM_THROW_OBJ_3_STEPS):
        step_str = 'step_' + str(i)

        if i == NUM_THROW_OBJ_3_STEPS - 1:
            smach.StateMachine.add(step_str, Joint_Pose_State('throw_obj_3/' + step_str, allowed_attempts=2),
                    transitions={'Success': 'Success',
                                 'Fail': 'Fail'})

        else:
            smach.StateMachine.add(step_str, Joint_Pose_State('throw_obj_3/' + step_str, allowed_attempts=2),
                    transitions={'Success': 'step_' + str(i+1),
                                 'Fail': 'Fail'})


"""
-------------------------------------~+=|=+~- Discard Obj 1 ~+=|=+~---------------------------------------

- Exatly like the unfold, but references throw_obj_3 in poses.yaml
"""

discard_obj_SM = smach.StateMachine(outcomes=['Success', 'Fail'])
NUM_DISCARD_OBJ_1_STEPS = len(rospy.get_param('joints/discard_obj'))

with discard_obj_SM:
    for i in range(0, NUM_DISCARD_OBJ_1_STEPS):
        step_str = 'step_' + str(i)

        if i == NUM_DISCARD_OBJ_1_STEPS - 1:
            smach.StateMachine.add(step_str, Joint_Pose_State('discard_obj/' + step_str, allowed_attempts=2),
                    transitions={'Success': 'Success',
                                 'Fail': 'Fail'})

        else:
            smach.StateMachine.add(step_str, Joint_Pose_State('discard_obj/' + step_str, allowed_attempts=2),
                    transitions={'Success': 'step_' + str(i+1),
                                 'Fail': 'Fail'})
            



"""
============================================================================================================
                                                   H O M E                                                  
============================================================================================================

-------------------------------------~+=|=+~- Home Position 1 ~+=|=+~---------------------------------------

- Exatly like the unfold, but references throw_obj_3 in poses.yaml
"""

home_pos_1_SM = smach.StateMachine(outcomes=['Success', 'Fail'])

with home_pos_1_SM:
    if pickup_fail_1_flag == 1:
        smach.StateMachine.add('go_home', Joint_Pose_State('ARM1_Home')
                               transitions={'Success': 'Pickup_Fail_1', 
                                            'Fail': 'Fail'})
        
    elif pickup_fail_1_flag == 2:
        smach.StateMachine.add('go_home', Joint_Pose_State('ARM1_Home') # ToDo: Set ARM1_Home in ARM1_poses.yaml
                           transitions={'Success': 'Move_On',
                                        'Fail': 'Fail'}) 
    
    else:
        smach.StateMachine.add('go_home', Joint_Pose_State('ARM1_Home')
                               transition={'Success': 'home_pos_1_SM'})
        
    smach.StateMachine.add()