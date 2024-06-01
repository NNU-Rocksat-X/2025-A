#!/usr/bin/env python3

import rospy
import smach
import smach_ros
import RPi.GPIO as GPIO
import time
from daedalus_msgs.srv import SignalStatus
from daedalus_msgs.srv import DeviceCmd
from daedalus_core.pcs_util import *


"""
=============================================================================
                            PERIPHERALS
=============================================================================
"""
class Detector_Status(smach.State):
    def __init__(self, detector):
        """
        Allows the state machine to make decisions based on the state of pins on the jetson

        INPUT
            detector - Either 'te_detection', 'partial_inhibit_detection', 'full_inhibit_detection'
        """

        smach.State.__init__(self, outcomes=['On', 'Off'])
        self.detector = detector
        rospy.wait_for_service(detector)
        self.detector_status = rospy.ServiceProxy(detector, SignalStatus)

    def execute(self, userdata):
        status = self.detector_status()

        if status.state:
            return 'On'
        else:
            return 'Off'


class Wait_State(smach.State):
    """
    Waits for the specified wait_time where wait_time is in seconds
    """
    
    def __init__(self, wait_time):
        smach.State.__init__(self, outcomes=['Complete'])
        self.wait_time = wait_time

    def execute(self, userdata):
        rospy.sleep(self.wait_time)

        return 'Complete'


class Check_Inhibit(smach.State):

    """
        Checks the status of the inhibit pin on the current Jetson the other Jetson
        If both inhibit pins return true, then Partial_Inhibit is returned
        If both inhibit pins are false, then No_Inhibit is returned
        If only one inhibit pin is true, then Full_Inhibit is returned
        If the other Jetson's pin state is not published, then eventually it will return timeout

        INPUT
            timeout - Number of seconds before transitioning to timeout
            poll_period - The period in seconds to poll the state of the other arm

        USAGE EXAMPLE
            smach.StateMachine.add('Check_Inhibit', Check_Inhibit(timeout=20),
                                    transitions={'Full_Inhibit': 'Do_Nothing',
                                                'Partial_Inhibit': 'Partial_Inhibit_Procedure',
                                                'No_Inhibit': 'Unfold',
                                                'Timeout': 'init'})
    """


    def __init__(self, timeout=10, poll_period=0.1):
        smach.State.__init__(self, outcomes=['Full_Inhibit', 'Partial_Inhibit', 'No_Inhibit', 'Timeout'])
        rospy.wait_for_service('full_inhibit_detection')
        # self.inhibit_status = rospy.ServiceProxy('full_inhibit_detection', SignalStatus)
        self.timeout = timeout
        self.poll_period = poll_period

    def execute(self, userdata):
        time_elapsed = 0
        other_arm_param = other_arm + 'inhibit_status'
        inhibit_status = self.inhibit_status()

        rospy.set_param('inhibit_status', inhibit_status.state)

        # Wait until other arm has a inhibit status set
        # while not rospy.has_param(other_arm_param):
        #     rospy.logwarn("Waiting for other arm's inhibit state")

        #     rospy.sleep(self.poll_period)
        #     time_elapsed += self.poll_period
        #     if time_elapsed > self.timeout:
        #         return 'Timeout'

        # return information based on both inhibits
        if inhibit_status.state: # Was if rospy.get_param(other_arm_param) and inhibit_status.state:
            return 'Partial_Inhibit'
        elif not inhibit_status.state: # Was elif not rospy.get_param(other_arm_param) and not inhibit_status.state: 
            return 'No_Inhibit'
        else:
            return 'Full_Inhibit'


class Delete_Param(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Done'])

    def execute(self, userdata):
        rospy.delete_param('inhibit_status')
        rospy.delete_param('sync_id')
        return 'Done'


class Ember_High_State(smach.State):
    #Sends Ember a High using gpio launching ball 

    def __init__(self):
        smach.State.__init__(self, outcomes=['Done'])
        GPIO.setmode(GPIO.BCM)
        self.output_pin = 16 # Physical Pin no 36
        GPIO.setup(self.output_pin, GPIO.OUT, initial=GPIO.LOW)
 
 

    def execute(self, userdata):
        GPIO.output(self.output_pin, GPIO.HIGH)  
        time.sleep(2)
        GPIO.output(self.output_pin, GPIO.LOW)

        return 'Done'
    
"""
=============================================================================
                    PAYLOAD CONTROL SYSTEM (PCS)
=============================================================================
"""

"""
The vision behind the PCS is that each device is a node which has a state it can be in
such as Enabled, Disabled, various fault states, etc
and there are states used to control the various devices from the main state machine
or sub state machines.

Marsha implemented this and it worked well, but it was complicated.
Example at https://github.com/aborger/Marsha/tree/flight_left/marsha_core/src/marsha_core

Due to schedule constraints this is the reduced functionality version


Use the activate and deactivate states to control nodes by their name.

For example:

smach.StateMachine.add('Success_led', PCS_Activate_State('led'),
                    transitions={'Complete': 'Mission_Success',
                                 'Error': 'Mission_Fail'})
"""

class PCS_State(smach.State):
    def __init__(self, device_name):
        smach.State.__init__(self, outcomes=['Complete', 'Error'])

        self.device_name = device_name
        rospy.wait_for_service(device_name)
        self.device_service = rospy.ServiceProxy(device_name, DeviceCmd)

class PCS_Activate_State(PCS_State):
    def execute(self, userdata):
        status = self.device_service(True)

        if status.done:
            return 'Complete'
        else:
            return 'Error'



class PCS_Deactivate_State(PCS_State):
    def execute(self, userdata):
        status = self.device_service(False)

        if status.done:
            return 'Complete'
        else:
            return 'Error'
            smach.StateMachine.add(step_str, Joint_Pose_State('folding/' + step_str, allowed_attempts=2),
                    transitions={'Success': 'step_' + str(i-1),
                                 'Fail': 'Fail'})



            
    


"""
============================================================================================================
                                            Partial-Inhibit lightbulb Test                                                  
============================================================================================================

"""

# Partial_Inhibit_LightBulb_Test_SM = smach.StateMachine(outcomes=['Success', 'Fail'])

# with Partial_Inhibit_LightBulb_Test_SM:
#     smach.StateMachine.add('Blink_on', PCS_Activate_State('led'),
#                             transitions={'Complete' : 'Blink_delay',
#                                         'Error' : 'Fail'})
#     smach.StateMachine.add('Blink_delay', Wait_State(0.5),
#                             transitions={'Complete': 'Blink_off'})

#     smach.StateMachine.add('Blink_off', PCS_Deactivate_State('led'),
#                             transitions={'Complete' : 'Blink_delay_2',
#                                         'Error' : 'Fail'})
    
#     smach.StateMachine.add('Blink_delay_2', Wait_State(0.5),
#                             transitions={'Complete': 'Blink_on_2'})

#     smach.StateMachine.add('Blink_on_2', PCS_Activate_State('led'),
#                             transitions={'Complete' : 'Blink_delay_3',
#                                         'Error' : 'Fail'})
    
#     smach.StateMachine.add('Blink_delay_3', Wait_State(0.5),
#                             transitions={'Complete': 'Blink_off_2'})

#     smach.StateMachine.add('Blink_off_2', PCS_Deactivate_State('led'),
#                             transitions={'Complete' : 'Success',
#                                         'Error' : 'Fail'})
    



