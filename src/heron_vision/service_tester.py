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
        self._is_right_side = True

        # Flag to prevent multiple service calls
        self._service_called = False

        # Create subscribers for the RGB image, depth image, and camera_info.
        self._rgb_sub = rospy.Subscriber(
            "/body_camera/body_camera/rgb/image_raw", Image, self.rgb_image_callback)
        self._depth_sub = rospy.Subscriber(
            "/body_camera/body_camera/stereo/image_raw", Image, self.depth_image_callback)
        self._camera_info_sub = rospy.Subscriber(
            "/body_camera/body_camera/rgb/camera_info", CameraInfo, self.camera_info_callback)

        # Create the service clients
        self._crack_service_client = rospy.ServiceProxy("find_cracks", FindCrack)
        self._pothole_service_client = rospy.ServiceProxy("find_potholes", FindPothole)
        self._lane_edge_service_client = rospy.ServiceProxy("find_lane_edges", FindLaneEdge)

    # Callbacks:
    def rgb_image_callback(self, image_msg):
        rospy.loginfo("Got an RGB image.")
        self._rgb_image_msg = image_msg
        self.send_service_request()

    def depth_image_callback(self, depth_image_msg):
        rospy.loginfo("Got Image depth.")
        self._depth_image_msg = depth_image_msg
        rospy.loginfo(depth_image_msg.encoding)
        self.send_service_request()

    def camera_info_callback(self, camera_info_msg):
        rospy.loginfo("Got Camera Info.")
        self._camera_info_msg = camera_info_msg
        self.send_service_request()


    def send_service_request(self):
    # If all messages are there, send the service request
        if self._rgb_image_msg and self._depth_image_msg and self._camera_info_msg:
            rospy.loginfo("All data available. Calling service calls.")
            # Send each service request
            self._send_crack_service_request()
            self._send_pothole_service_request()
            self._send_lane_edge_service_request()

    def _send_crack_service_request(self):
        rospy.loginfo("Calling crack service.")

        service_request = FindCrackRequest()
        service_request.image_rgb = self._rgb_image_msg
        service_request.image_depth = self._depth_image_msg
        service_request.camera_info = self._camera_info_msg

        service_response = self._crack_service_client(service_request)

        # Check
        rospy.loginfo(f"Received Crack Service Response: Success={service_response.success}")

        # And exit
        rospy.sleep(2)  # Give ROS some time before shutdown
        #rospy.signal_shutdown("we're finished")


    def _send_pothole_service_request(self):
        rospy.loginfo("Calling pothole service.")

        service_request = FindPotholeRequest()
        service_request.image_rgb = self._rgb_image_msg
        service_request.image_depth = self._depth_image_msg
        service_request.camera_info = self._camera_info_msg

        service_response = self._pothole_service_client(service_request)

        # Check
        rospy.loginfo(f"Received Pothole Service Response: Success={service_response.success}")

        # And exit
        #rospy.signal_shutdown("we're finished")


    def _send_lane_edge_service_request(self):
        rospy.loginfo("Calling lane edge service.")

        service_request = FindLaneEdgeRequest()
        service_request.image_rgb = self._rgb_image_msg
        service_request.image_depth = self._depth_image_msg
        service_request.camera_info = self._camera_info_msg
        service_request.is_right_side = self._is_right_side

        service_response = self._lane_edge_service_client(service_request)

        # Then print the results
        rospy.loginfo(f"Received Lane Edge Service Response: Success={service_response.success}")
        rospy.loginfo(service_response)

        # And exit
        #rospy.signal_shutdown("we're finished")




    def start_spin(self):
        rospy.loginfo("Idling until exit")
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
