#!/usr/bin/env python3

import rospy
import smach
import smach_ros
from daedalus_msgs.srv import SignalStatus

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