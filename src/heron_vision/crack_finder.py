import rospy
from cv_bridge import CvBridge
from heron_msgs.srv import *
from robotnik_msgs.srv import *


class CrackFinder:
    def __init__(self, name):
        # Register a node with the ROS Master server and register a shutdown hook
        #    - only one node allowed per process
        rospy.init_node(name)

        self._cv_bridge = CvBridge()

        # Create a service server for the find_cracks service.
        self._crack_service = rospy.Service(
            "~find_cracks", FindCrack, self.find_crack_callback)
        self._get_bool_service = rospy.Service(
            "~get_bool", GetBool, self.get_bool_callback)

    # Define a callback for the cracks server.

    def find_crack_callback(self, req: FindCrackRequest):
        print("Finding some cracks")
        cv_image = self._cv_bridge.imgmsg_to_cv2(
            req.image_rgb, desired_encoding='passthrough')
        print(cv_image.shape)
        rospy.sleep(5.0)
        # TODO: Fill in the rest here!!!!
        return FindCrackResponse()

    def get_bool_callback(self, req: GetBoolRequest):
        print("Getting a bool")
        return GetBoolResponse()

    def start_spin(self):
        rospy.loginfo("Idling until exit")
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
