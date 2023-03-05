import rospy

from std_srvs.srv import Trigger, TriggerRequest

from daedalus_msgs.srv import ObjectObservation

class CatchInterface(RosInterface):
    """
    CatchInterface provides an interface between the OpenAI gym environment and ROS.
    """

    def __init__(self):
        super(CatchInterface, self).__init__()

    def setup_services(self):
        rospy.wait_for_service("/mujoco/reset")
        self.reset = rospy.ServiceProxy("/mujoco/reset", Trigger)

        rospy.wait_for_service("/trajectory_predictor/observe")
        self.observe = rospy.ServiceProxy('/trajectory_predictor/observe', ObjectObservation)

        

