import rospy
from heron_msgs.srv import *
from robotnik_msgs.srv import *
from sensor_msgs.msg import Image, CameraInfo


class ServiceTester:
    def __init__(self, name):
        # Register a node with the ROS Master server and register a shutdown hook
        #    - only one node allowed per process
        rospy.init_node(name)

        # Cache the messages
        self._rgb_image_msg = None
        self._depth_image_msg = None
        self._camera_info_msg = None

        # Create subscribers for the RGB image, depth image, and camera_info.
        self._rgb_sub = rospy.Subscriber(
            "/front_rgbd_camera/rgb/image_raw", Image, self.rgb_image_callback)
        self._depth_sub = rospy.Subscriber(
            "/front_rgbd_camera/stereo/image", Image, self.depth_image_callback)
        self._camera_info_sub = rospy.Subscriber(
            "/front_rgbd_camera/stereo/camera_info", CameraInfo, self.camera_info_callback)

        # Create a service client for the cracks (to begin with)
        self._crack_service_client = rospy.ServiceProxy("find_cracks", FindCrack)

    # Callbacks:
    def rgb_image_callback(self, image_msg):
        rospy.loginfo("Got an RGB image.")
        self._rgb_image_msg = image_msg
        self.send_service_request()

    def depth_image_callback(self, image_msg):
        rospy.loginfo("Got an Depth image.")
        self._depth_image_msg = image_msg
        self.send_service_request()

    def camera_info_callback(self, camera_info_msg):
        rospy.loginfo("Got an Camera Info image.")
        self._camera_info_msg = camera_info_msg
        self.send_service_request()

    def send_service_request(self):
        # If all messages are there, send the service request
        if self._rgb_image_msg and self._depth_image_msg and self._camera_info_msg:
            rospy.loginfo("Calling service call.")

            service_request = FindCrackRequest()
            service_request.image_rgb = self._rgb_image_msg
            service_request.image_depth = self._depth_image_msg
            service_request.camera_info = self._camera_info_msg

            service_response = self._crack_service_client(service_request)

            # Then print the results
            rospy.loginfo(service_response)

            # And exit'
            rospy.signal_shutdown("we're finished")

    def start_spin(self):
        rospy.loginfo("Idling until exit")
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
