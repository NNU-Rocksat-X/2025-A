#!/usr/bin/env python

import rospy
import smach
import smach_ros

from daedalus_core.daedalus_services.move_cmds import *
from daedalus_core.pcs_util import *
from daedalus_core.pcs_states import *

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
    
    #Takes pose name as parameter and executes, transitions to next state when done.
    """
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
            #joint_pose_cmd(self.pose)
            complete = joint_pose_cmd(self.pose).done

        if complete:
            return 'Success'
        else:
            return 'Fail'

# class Ender_High_State(smach.State):
#     #Sends Ember a High using gpio

#     def __init__(self):
#         smach.State.__init__(self, outcomes=['Success', 'Fail'])
#         GPIO.setmode(GPIO.BCM)
#         output_pin = 16 # Physical Pin no 36
#         GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.LOW)
#         self.time = time.time()
 

#     def execute(self, userdata):
#         complete = False

#         GPIO.output(output_pin, GPIO.HIGH)  
#         time.sleep(5)
#         GPIO.output(output_pin, GPIO.LOW)

#         if complete:
#             return 'Success'
#         else:
#             return 'Fail'

#     start_time = time.time()
#     trigger_action()

#     GPIO.cleanup()


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
    smach.StateMachine.add('Arm_Sync_Grasp', ARM_Sync("pickup", timeout=120),
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
    smach.StateMachine.add('Arm_Sync_Grasp', ARM_Sync("pickup", timeout=120),
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

# ============================================================================================================
#                                                   ARM1                                                  
# ============================================================================================================

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
        delay_str = 'delay_' + str(i)

        if i == NUM_FOLDING_STEPS - 1:
            smach.StateMachine.add(step_str, Joint_Pose_State('folding/' + step_str, allowed_attempts=2),
                    transitions={'Success': 'Success',
                                 'Fail': 'Fail'})

        else:
            smach.StateMachine.add(step_str, Joint_Pose_State('folding/' + step_str, allowed_attempts=2),
                    transitions={'Success': delay_str,
                                 'Fail': 'Fail'})
            
            smach.StateMachine.add(delay_str, Wait_State(1),
                    transitions={'Complete': 'step_' + str(i+1)})



Fold_SM = smach.StateMachine(outcomes=['Success', 'Fail'])
NUM_REFOLDING_STEPS = len(rospy.get_param('joints/refolding'))

print(NUM_REFOLDING_STEPS)

with Fold_SM:
    for i in range(0, NUM_REFOLDING_STEPS):
        step_str = 'step_' + str(i)
        delay_str = 'delay_' + str(i)

        if i == NUM_REFOLDING_STEPS - 1:
            smach.StateMachine.add(step_str, Joint_Pose_State('refolding/' + step_str, allowed_attempts=2),
                    transitions={'Success': 'Success',
                                 'Fail': 'Fail'})

        else:
            smach.StateMachine.add(step_str, Joint_Pose_State('refolding/' + step_str, allowed_attempts=2),
                    transitions={'Success': delay_str,
                                 'Fail': 'Fail'})
            
            smach.StateMachine.add(delay_str, Wait_State(1),
                    transitions={'Complete': 'step_' + str(i+1)})

"""
------------------------------------------------------------------------------
                                FOLDING
------------------------------------------------------------------------------

- Loops through however many steps are in folding section of poses.yaml
- Starts at step_n and goes in reverse until step_0
- Uses NUM_FOLDING_STEPS defined for unfolding
"""

Fold_ARM_SM = smach.StateMachine(outcomes=['Success', 'Fail'])

with Fold_ARM_SM:
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
============================================================================================================
                                                  OBJECT 1                                                  
============================================================================================================


--------------------------------------~+=|=+~- Pickup Obj 1 ~+=|=+~---------------------------------------

- ARM1 Picks up Ball 1
- ARM1 moves to the ball, checks to see if its grasped
- Test Ready
"""

pickup_obj_1_SM_ARM1 = smach.StateMachine(outcomes=['Success', 'Fail'])
NUM_PICKUP_OBJ_1_STEPS = len(rospy.get_param('joints/pickup_obj_1'))

with pickup_obj_1_SM_ARM1:
    for i in range(0, NUM_PICKUP_OBJ_1_STEPS):
        step_str = 'step_' + str(i)
        delay_str = 'delay_' + str(i)

        if i == NUM_PICKUP_OBJ_1_STEPS - 1:
            smach.StateMachine.add(step_str, Joint_Pose_State('pickup_obj_1/' + step_str, allowed_attempts=2),
                    transitions={'Success': 'Success',
                                 'Fail': 'Fail'})

        else:
            smach.StateMachine.add(step_str, Joint_Pose_State('pickup_obj_1/' + step_str, allowed_attempts=2),
                    transitions={'Success': delay_str,
                                 'Fail': 'Fail'})
            
            smach.StateMachine.add(delay_str, Wait_State(0),
                    transitions={'Complete': 'step_' + str(i+1)})


"""
-------------------------------------~+=|=+~- Handoff Obj 1 ~+=|=+~---------------------------------------

- Moves to Handoff position and waits for Arm_Sync
- Move on to Home if any of the states fail
"""

handoff_1_SM_ARM1 = smach.StateMachine(outcomes=['Success', 'Fail'])
#NUM_THROW_OBJ_2_STEPS = len(rospy.get_param('joints/throw_obj_2'))

with handoff_1_SM_ARM1:
    smach.StateMachine.add('Active_Home', Joint_Pose_State('active_home', allowed_attempts = 2),
                           transitions={'Success': 'Move_To_Handy',
                                        'Fail': 'Fail'}) # temporary


    smach.StateMachine.add('Move_To_Handy', Joint_Pose_State('pre_handoff_1'), # TODO add handoff poses to poses.yaml
                           transitions={'Success': 'Wait_For_Sync',
                                        'Fail': 'Fail'})
    
    
    smach.StateMachine.add('Wait_For_Sync', ARM_Sync("handoff_1", timeout = 120), # TODO Sync all the sycn states so that they all match up
                           transitions={'Ready': 'Handoff',
                                        'Timeout': 'Fail'})
    
    smach.StateMachine.add('Handoff', Joint_Pose_State('handoff_1a'),
                           transitions={'Success': 'Sync_Grasps',
                                        'Fail': 'Fail'})

    smach.StateMachine.add('Sync_Grasps', ARM_Sync("grasp_ready_1", timeout = 120), 
                           transitions={'Ready': 'Open_Gripper',
                                        'Timeout': 'Fail'})
    
    smach.StateMachine.add('Open_Gripper', Joint_Pose_State('handoff_1b'), # Open gripper
                           transitions={'Success': 'Post_Handoff',
                                        'Fail': 'Fail'})
    
    smach.StateMachine.add('Post_Handoff', Joint_Pose_State('handoff_1c'),
                           transitions={'Success': 'Success',
                                        'Fail': 'Fail'})
    
"""
-------------------------------------~+=|=+~- Home Position 1 ~+=|=+~---------------------------------------

- ARM1
- Ready for testing
"""

home_pos_1_SM_ARM1 = smach.StateMachine(outcomes=['Success', 'Fail'])

with home_pos_1_SM_ARM1:
    smach.StateMachine.add('Go_Home', Joint_Pose_State("active_home"), # TODO add active home to the poses.yaml
                           transitions={'Success': 'Sync_Arms', 
                                        'Fail': 'Fail'})

    smach.StateMachine.add('Sync_Arms', ARM_Sync("handoff_2", timeout = 120), # wait here until arm 2 has gotten rid of the object
                           transitions={'Ready': 'Success',
                                        'Timeout': 'Fail'})        



"""
============================================================================================================
                                                  OBJECT 2                                                  
============================================================================================================

-------------------------------------~+=|=+~- Handoff Obj 2 ~+=|=+~---------------------------------------

- 
- 
"""

handoff_2_SM_ARM1 = smach.StateMachine(outcomes=['Success', 'Fail'])

with handoff_2_SM_ARM1:
    smach.StateMachine.add('Active_Home', Joint_Pose_State('active_home', allowed_attempts = 2),
                           transitions={'Success': 'Move_To_Handy',
                                        'Fail': 'Fail'}) # temporary


    smach.StateMachine.add('Move_To_Handy', Joint_Pose_State('active_home'), # TODO add handoff poses to poses.yaml
                           transitions={'Success': 'Wait_For_Sync',
                                        'Fail': 'Fail'})
    
    
    smach.StateMachine.add('Wait_For_Sync', ARM_Sync("handoff_2", timeout = 120), # TODO Sync all the sycn states so that they all match up
                           transitions={'Ready': 'Handoff',
                                        'Timeout': 'Fail'})
    
    smach.StateMachine.add('Handoff', Joint_Pose_State('handoff_2a'),
                           transitions={'Success': 'Sync_Grasps',
                                        'Fail': 'Fail'})

    smach.StateMachine.add('Sync_Grasps', ARM_Sync("grasp_ready_2", timeout = 120), 
                           transitions={'Ready': 'Wait_Again',
                                        'Timeout': 'Fail'})
    
    smach.StateMachine.add('Wait_Again', Wait_State(0),
                           transitions={'Complete': 'Final_Sync'})
    
    smach.StateMachine.add('Final_Sync', ARM_Sync('grasp_ready_2a', timeout=120),
                           transitions={'Ready': 'Last_Wait',
                                        'Timeout': 'Fail'})
    
    smach.StateMachine.add('Last_Wait', Wait_State(3),
                           transitions={'Complete': 'Open_Gripper'})
    
    smach.StateMachine.add('Open_Gripper', Joint_Pose_State('handoff_2b'), # Open gripper
                           transitions={'Success': 'Post_Handoff',
                                        'Fail': 'Fail'})
    
    smach.StateMachine.add('Post_Handoff', Joint_Pose_State('handoff_2c'),
                           transitions={'Success': 'Success',
                                        'Fail': 'Fail'})


"""
-------------------------------------~+=|=+~- Discard Obj 2 ~+=|=+~---------------------------------------

- Gets rid of whats in the gripper by iterating through the discard_obj_2 poses in the yaml
- TODO add a fail-safe state-state-machine if the gripper returns that there is something inside it
"""

discard_obj_2_SM_ARM1 = smach.StateMachine(outcomes=['Success', 'Fail'])
#NUM_DISCARD_OBJ_2_STEPS = len(rospy.get_param('joints/discard_obj'))

with discard_obj_2_SM_ARM1:
    smach.StateMachine.add('Discard_1', Joint_Pose_State('discard_0'),
                           transitions={'Success': 'Wait_1',
                                        'Fail': 'Fail'})
    
    smach.StateMachine.add('Wait_1', Wait_State(2),
                           transitions={'Complete': 'Discard_2'})
    
    smach.StateMachine.add('Discard_2', Joint_Pose_State('active_home'),
                           transitions={'Success': 'Success',
                                        'Fail': 'Fail'})


"""
-------------------------------------~+=|=+~- Home Position 2 ~+=|=+~---------------------------------------

-
-
"""
home_pos_2_SM_ARM1 = smach.StateMachine(outcomes=['Success', 'Fail'])

with home_pos_2_SM_ARM1:
    smach.StateMachine.add('Go_Home', Joint_Pose_State('active_home'), 
                           transitions={'Success': 'Sync_Up', 
                                        'Fail': 'Fail'})

    smach.StateMachine.add('Sync_Up', ARM_Sync("handoff_3", timeout = 120), # wait here until arm 2 is ready to recieve 
                           transitions={'Ready': 'Success',
                                        'Timeout': 'Fail'})        

"""
============================================================================================================
                                                  OBJECT 3                                                  
============================================================================================================

"""
"""
--------------------------------------~+=|=+~- Pickup Obj 3 ~+=|=+~---------------------------------------

- ARM1 Picks up Cube 1
- ARM1 moves to the cube, checks to see if its grasped
- Test Ready
"""

pickup_obj_3_SM_ARM1 = smach.StateMachine(outcomes=['Success', 'Fail'])
NUM_PICKUP_OBJ_3_STEPS = len(rospy.get_param('joints/pickup_obj_3'))

with pickup_obj_3_SM_ARM1:
    for i in range(0, NUM_PICKUP_OBJ_1_STEPS):
        step_str = 'step_' + str(i)
        delay_str = 'delay_' + str(i)

        if i == NUM_PICKUP_OBJ_1_STEPS - 1:
            smach.StateMachine.add(step_str, Joint_Pose_State('pickup_obj_3/' + step_str, allowed_attempts=2),
                    transitions={'Success': 'Success',
                                 'Fail': 'Fail'})

        else:
            smach.StateMachine.add(step_str, Joint_Pose_State('pickup_obj_3/' + step_str, allowed_attempts=2),
                    transitions={'Success': delay_str,
                                 'Fail': 'Fail'})

            smach.StateMachine.add(delay_str, Wait_State(0),
                                   transitions={'Complete': 'step_' + str(i+1)})



"""
-------------------------------------~+=|=+~- Handoff Obj 3 ~+=|=+~---------------------------------------

- Moves to Handoff position and waits for Arm_Sync
- Move on to Home if any of the states fail
"""

handoff_3_SM_ARM1 = smach.StateMachine(outcomes=['Success', 'Fail'])

with handoff_3_SM_ARM1:
    smach.StateMachine.add('Active_Home', Joint_Pose_State('active_home', allowed_attempts = 2),
                           transitions={'Success': 'Move_To_Handy',
                                        'Fail': 'Fail'}) # temporary


    smach.StateMachine.add('Move_To_Handy', Joint_Pose_State('pre_handoff_1'), # TODO add handoff poses to poses.yaml
                           transitions={'Success': 'Wait_For_Sync',
                                        'Fail': 'Fail'})
    
    
    smach.StateMachine.add('Wait_For_Sync', ARM_Sync("handoff_3a", timeout = 120), # TODO Sync all the sycn states so that they all match up
                           transitions={'Ready': 'Handoff',
                                        'Timeout': 'Fail'})
    
    smach.StateMachine.add('Handoff', Joint_Pose_State('handoff_1a'),
                           transitions={'Success': 'Sync_Grasps',
                                        'Fail': 'Fail'})

    smach.StateMachine.add('Sync_Grasps', ARM_Sync("grasp_ready_3", timeout = 120), 
                           transitions={'Ready': 'Open_Gripper',
                                        'Timeout': 'Fail'})
    
    smach.StateMachine.add('Open_Gripper', Joint_Pose_State('handoff_1b'), # Open gripper
                           transitions={'Success': 'Post_Handoff',
                                        'Fail': 'Fail'})
    
    smach.StateMachine.add('Post_Handoff', Joint_Pose_State('handoff_1c'),
                           transitions={'Success': 'Success',
                                        'Fail': 'Fail'})
    
"""
-------------------------------------~+=|=+~- Home Position 3 ~+=|=+~---------------------------------------

- ARM1
- Ready for testing
"""

home_pos_3_SM_ARM1 = smach.StateMachine(outcomes=['Success', 'Fail'])

with home_pos_3_SM_ARM1:
    smach.StateMachine.add('Go_Home', Joint_Pose_State("active_home"), 
                           transitions={'Success': 'Success', 
                                        'Fail': 'Fail'})
        


# ============================================================================================================
#                                                   ARM2                                                  
# ============================================================================================================


"""
============================================================================================================
                                                  OBJECT 1                                                  
============================================================================================================

-------------------------------------~+=|=+~- Handoff Obj 1 ~+=|=+~---------------------------------------

- 
- 
"""

# handoff_1_SM_ARM2 = smach.StateMachine(outcomes=['Success', 'Fail'])

# with handoff_1_SM_ARM2:
#     smach.StateMachine.add('Move_To_Handy', Joint_Pose_State('pre_handoff'), # move to general position w/ gripper open
#                            transitions={'Success': 'Wait_For_Sync',
#                                         'Fail': 'Fail'})
    
#     smach.StateMachine.add('Wait_For_Sync', ARM_Sync("handoff_1", timeout = 120), # wait for arm2 to be ready
#                            transitions={'Ready': 'Handoff',
#                                         'Timeout': 'Fail'})
    
#     smach.StateMachine.add('Handoff', Joint_Pose_State('handoff_1a'), # move closer to the target position
#                            transitions={'Success': 'Pre_Close_Gripper',
#                                         'Fail': 'Fail'})
    
#     smach.StateMachine.add('Pre_Close_Gripper', Joint_Pose_State('handoff_1b'), 
#                            transitions={'Success': 'Check_Gripper',
#                                         'Fail': 'Fail'})
    
#     smach.StateMachine.add('Sync grasps', ARM_Sync('grasp_ready_1', timeout = 120),
#                            transitions={'Ready': 'Close_Gripper',
#                                         'Timeout': 'Fail'})
    
#     smach.StateMachine.add('Close_Gripper', Joint_Pose_State('handoff_1c'),  # check the gripper to ensure we "caught" the ball
#                             transitions={'Success': 'Check_Gripper',
#                                         'Fail': 'Check_Gripper'})
    
#     smach.StateMachine.add('Check_Gripper', Is_Grasped(),  # check the gripper to ensure we "caught" the ball
#                             transitions={'Success': 'Post_Handoff',
#                                         'Fail': 'Try_Again_A'})
    
#     smach.StateMachine.add('Post_Handoff', Joint_Pose_State('handoff_1d'), # Move away from the other arm becasuse we got it yay
#                            transitions={'Success': 'Success',
#                                         'Fail': 'Fail'})

#     smach.StateMachine.add('Try_Again_A', Is_Grasped(), # Check again because we might not have got it the first time
#                            transitions={'Success': 'Post_Handoff',
#                                         'Fail': 'Fail'})


# """
# -------------------------------------~+=|=+~- Discard Obj 1 ~+=|=+~---------------------------------------

# - Gets rid of whats in the gripper by iterating through the discard_obj_2 poses in the yaml
# - TODO add a fail-safe state-state-machine if the gripper returns that there is something inside it
# """

# discard_obj_1_SM_ARM2 = smach.StateMachine(outcomes=['Success', 'Fail'])
# NUM_DISCARD_OBJ_1_STEPS = len(rospy.get_param('joints/discard_obj'))

# with discard_obj_1_SM_ARM2:
#     for i in range(0, NUM_DISCARD_OBJ_1_STEPS):
#         step_str = 'step_' + str(i)

#         if i == NUM_DISCARD_OBJ_1_STEPS - 1:
#             smach.StateMachine.add(step_str, Joint_Pose_State('discard_obj_1/' + step_str, allowed_attempts=2),
#                     transitions={'Success': 'Check_Gripper',
#                                  'Fail': 'Fail'})

#         else:
#             smach.StateMachine.add(step_str, Joint_Pose_State('discard_obj_1/' + step_str, allowed_attempts=2),
#                     transitions={'Success': 'step_' + str(i+1),
#                                  'Fail': 'Fail'})
   
#     # smach.StateMachine.add('Check_Gripper', Is_Grasped(),
#     #                        transitions={'Success': 'Fail',
#     #                                     'Fail': 'Success'}) # Check to see if there is anything in the gripper (empty is success)


# """
# -------------------------------------~+=|=+~- Home Position 1 ~+=|=+~---------------------------------------

# -
# -
# """
# home_pos_1_SM_ARM2 = smach.StateMachine(outcomes=['Success', 'Fail'])

# with home_pos_1_SM_ARM2:
#     smach.StateMachine.add('Go_Home', Joint_Pose_State('active_home'), 
#                            transitions={'Success': 'Sync_Up', 
#                                         'Fail': 'Fail'})

#     smach.StateMachine.add('Sync_Up', ARM_Sync("handoff_2", timeout=120), # wait here until arm 2 is ready to recieve 
#                            transitions={'Ready': 'Success',
#                                         'Timeout': 'Fail'})  

# """
# ============================================================================================================
#                                                   OBJECT 2                                                  
# ============================================================================================================


# --------------------------------------~+=|=+~- Pickup Obj 2 ~+=|=+~---------------------------------------

# - ARM1 Picks up Ball 1
# - ARM1 moves to the ball, checks to see if its grasped
# - Test Ready
# """

# pickup_obj_2_SM_ARM2 = smach.StateMachine(outcomes=['Success', 'Fail'])
# NUM_PICKUP_OBJ_2_STEPS = len(rospy.get_param('joints/pickup_obj_2'))

# with pickup_obj_2_SM_ARM2:
#     for i in range(0, NUM_PICKUP_OBJ_2_STEPS):
#         step_str = 'step_' + str(i)

#         if i == NUM_PICKUP_OBJ_2_STEPS - 1:
#             smach.StateMachine.add(step_str, Joint_Pose_State('pickup_obj_2/' + step_str, allowed_attempts=2),
#                     transitions={'Success': 'Check_Grasp',
#                                  'Fail': 'Fail'})

#         else:
#             smach.StateMachine.add(step_str, Joint_Pose_State('pickup_obj_2/' + step_str, allowed_attempts=2),
#                     transitions={'Success': 'step_' + str(i+1),
#                                  'Fail': 'Fail'})

#     smach.StateMachine.add('Check_Grasp', Is_Grasped(),
#                            transitions={'Success': 'Post_Pickup_A',
#                                         'Fail': 'Fail'})
    
#     smach.StateMachine.add('Post_Pickup_A', Joint_Pose_State('pre_active_home'),
#                            transitions={'Success': 'Post_Pickup_B',
#                                         'Fail': 'Fail'})
                           
#     smach.StateMachine.add('Post_Pickup_B', Joint_Pose_State('active_home'),
#                            transitions={'Success': 'Success',
#                                         'Fail': 'Fail'})


# """
# -------------------------------------~+=|=+~- Handoff Obj 2 ~+=|=+~---------------------------------------

# - Moves to Handoff position and waits for Arm_Sync
# - Move on to Home if any of the states fail
# """

# handoff_2_SM_ARM2 = smach.StateMachine(outcomes=['Success', 'Fail'])
# #NUM_THROW_OBJ_2_STEPS = len(rospy.get_param('joints/throw_obj_2'))

# with handoff_2_SM_ARM2:
#     smach.StateMachine.add('Move_To_Handy', Joint_Pose_State('pre_handoff'), # TODO add handoff poses to poses.yaml
#                            transitions={'Success': 'Wait_For_Sync',
#                                         'Fail': 'Fail'})
    
#     smach.StateMachine.add('Wait_For_Sync', ARM_Sync("handoff_2", timeout=120), # TODO Sync all the sycn states so that they all match up
#                            transitions={'Ready': 'Handoff',
#                                         'Timeout': 'Fail'})
    
#     smach.StateMachine.add('Handoff', Joint_Pose_State('handoff_2a'),
#                            transitions={'Success': 'Pre_Open_Gripper',
#                                         'Fail': 'Fail'})
    
#     smach.StateMachine.add('Pre_Open_Gripper', Joint_Pose_State('handoff_2b'),
#                            transitions={'Success': 'Sync_Grasps',
#                                         'Fail': 'Fail'})
    
#     smach.StateMachine.add('Sync_Grasps', ARM_Sync("grasp_ready_2", timeout=120), 
#                            transitions={'Ready': 'Open_Gripper',
#                                         'Timeout': 'Fail'})
    
#     smach.StateMachine.add('Open_Gripper', Joint_Pose_State('handoff_2c'), # Open gripper
#                            transitions={'Success': 'Post_Handoff',
#                                         'Fail': 'Fail'})
    
#     smach.StateMachine.add('Post_Handoff', Joint_Pose_State('handoff_2d'),
#                            transitions={'Success': 'Success',
#                                         'Fail': 'Fail'})
    
# """
# -------------------------------------~+=|=+~- Home Position 2 ~+=|=+~---------------------------------------

# - ARM1
# - Ready for testing
# """

# home_pos_2_SM_ARM2 = smach.StateMachine(outcomes=['Success', 'Fail'])

# with home_pos_2_SM_ARM2:
#     smach.StateMachine.add('Go_Home', Joint_Pose_State('active_home'), # TODO add active home to the poses.yaml
#                            transitions={'Success': 'Sync_Up', 
#                                         'Fail': 'Fail'})

#     smach.StateMachine.add('Sync_Up', ARM_Sync("handoff_3", timeout=20), # wait here until arm 2 has gotten rid of the object
#                            transitions={'Ready': 'Success',
#                                         'Timeout': 'Fail'})        

# """
# ============================================================================================================
#                                                   OBJECT 3                                                  
# ============================================================================================================

# -------------------------------------~+=|=+~- Handoff Obj 3 ~+=|=+~---------------------------------------

# - 
# - 
# """

# handoff_3_SM_ARM2 = smach.StateMachine(outcomes=['Success', 'Fail'])

# with handoff_3_SM_ARM2:
#     smach.StateMachine.add('Move_To_Handy', Joint_Pose_State('pre_handoff'), # move to general position w/ gripper open
#                            transitions={'Success': 'Handoff',
#                                         'Fail': 'Fail'})
    
#     smach.StateMachine.add('Handoff', Joint_Pose_State('handoff_3a'), # move closer to the target position
#                            transitions={'Success': 'Pre_Close_Gripper',
#                                         'Fail': 'Fail'})
    
#     smach.StateMachine.add('Pre_Close_Gripper', Joint_Pose_State('handoff_3b'), 
#                            transitions={'Success': 'Sync_Grasps',
#                                         'Fail': 'Fail'})
    
#     smach.StateMachine.add('Sync_Grasps', ARM_Sync('grasp_ready_3', timeout = 120),
#                            transitions={'Ready': 'Close_Gripper',
#                                         'Timeout': 'Fail'})
    
#     smach.StateMachine.add('Close_Gripper', Joint_Pose_State('handoff_3c'),  # check the gripper to ensure we "caught" the ball
#                             transitions={'Success': 'Check_Gripper',
#                                         'Fail': 'Check_Gripper'})
    
#     smach.StateMachine.add('Check_Gripper', Is_Grasped(),  # check the gripper to ensure we "caught" the ball
#                             transitions={'Success': 'Post_Handoff',
#                                         'Fail': 'Try_Again_A'})
    
#     smach.StateMachine.add('Post_Handoff', Joint_Pose_State('handoff_3d'), # Move away from the other arm becasuse we got it yay
#                            transitions={'Success': 'Success',
#                                         'Fail': 'Fail'})

#     smach.StateMachine.add('Try_Again_A', Is_Grasped(), # Check again because we might not have got it the first time
#                            transitions={'Success': 'Post_Handoff',
#                                         'Fail': 'Fail'})


# """
# -------------------------------------~+=|=+~- Discard Obj 3 ~+=|=+~---------------------------------------

# - Gets rid of whats in the gripper by iterating through the discard_obj_2 poses in the yaml
# - TODO add a fail-safe state-state-machine if the gripper returns that there is something inside it
# """

# discard_obj_3_SM_ARM2 = smach.StateMachine(outcomes=['Success', 'Fail'])
# NUM_DISCARD_OBJ_3_STEPS = len(rospy.get_param('joints/discard_obj'))

# with discard_obj_3_SM_ARM2:
#     for i in range(0, NUM_DISCARD_OBJ_3_STEPS):
#         step_str = 'step_' + str(i)

#         if i == NUM_DISCARD_OBJ_3_STEPS - 1:
#             smach.StateMachine.add(step_str, Joint_Pose_State('discard_obj_3/' + step_str, allowed_attempts=2),
#                     transitions={'Success': 'Check_Gripper',
#                                  'Fail': 'Fail'})

#         else:
#             smach.StateMachine.add(step_str, Joint_Pose_State('discard_obj_3/' + step_str, allowed_attempts=2),
#                     transitions={'Success': 'step_' + str(i+1),
#                                  'Fail': 'Fail'})
   
#     smach.StateMachine.add('Check_Gripper', Is_Grasped(),
#                            transitions={'Success': 'Fail',
#                                         'Fail': 'Success'}) # Check to see if there is anything in the gripper (empty is success)


# """
# -------------------------------------~+=|=+~- Home Position 3 ~+=|=+~---------------------------------------

# -
# -
# """
# home_pos_3_SM_ARM2 = smach.StateMachine(outcomes=['Success', 'Fail'])

# with home_pos_3_SM_ARM2:
#     smach.StateMachine.add('Go_Home', Joint_Pose_State('active_home'), 
#                            transitions={'Success': 'Success', 
#                                         'Fail': 'Fail'})

"""
============================================================================================================
                                            Partial-Inhibit Test                                                  
============================================================================================================

"""

Partial_Inhibit_SM = smach.StateMachine(outcomes=['Success', 'Fail'])

with Partial_Inhibit_SM:

    smach.StateMachine.add('Open_Gripper', Joint_Pose_State("open"),
                            transitions={'Success': 'Delay',
                                        'Fail': 'Fail'})
    
    smach.StateMachine.add('Delay', Wait_State(1),
                           transitions={'Complete': 'Close_Gripper'})
    
    smach.StateMachine.add('Close_Gripper', Joint_Pose_State("close"),
                            transitions={'Success': 'Success',
                                        'Fail': 'Fail'})







