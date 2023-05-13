#!/usr/bin/env python3

import rospy
import smach
import smach_ros
from daedalus_msgs.srv import SignalStatus

"""
=============================================================================
                            JESTON COMM STATES
=============================================================================
"""

"""
Look at namespace for the other arm.
This is used to communicate with the other arm
"""
other_arm = None
if rospy.get_namespace() == "/ARM1/":
    other_arm = "/ARM2/"
else:
    other_arm = "/ARM1/"

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


class PCS_Activate_State(smach.State):
    """
    """
    def __init__(self, activation):
        smach.State.__init__(self, outcomes=['Complete'])