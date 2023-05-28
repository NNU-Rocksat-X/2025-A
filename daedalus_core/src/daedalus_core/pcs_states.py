#!/usr/bin/env python3

import rospy
import smach
import smach_ros
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






