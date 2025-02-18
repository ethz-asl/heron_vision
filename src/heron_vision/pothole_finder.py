import rospy
import cv2
from cv_bridge import CvBridge
from heron_msgs.srv import *
from robotnik_msgs.srv import *
import numpy as np
from scipy.ndimage import center_of_mass
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from .inference import segment_defect


class PotholeFinder:
    def __init__(self, name):
        # Register a node with the ROS Master server and register a shutdown hook
        #    - only one node allowed per process
        rospy.init_node(name)

        self._cv_bridge = CvBridge()

        # Create a service server for the find_potholes service.
        self._pothole_service = rospy.Service(
            "find_potholes", FindPothole, self.find_pothole_callback)
        self._get_bool_service = rospy.Service(
            "get_bool", GetBool, self.get_bool_callback)

        # Also publish the segmentation on a topic.
        self._segmentation_pub = rospy.Publisher(
            "pothole_segmentation", Image, queue_size=1, latch=True)

    # Define a callback for the potholes server.
    def find_pothole_callback(self, req: FindPotholeRequest):
        print("Finding some potholes")

        # Save RGB image
        cv_image = self._cv_bridge.imgmsg_to_cv2(
            req.image_rgb, desired_encoding='passthrough')
        print(cv_image.shape)

        # Save depth image
        depth_image = self._cv_bridge.imgmsg_to_cv2(
            req.image_depth, desired_encoding='passthrough')

        # Save camera info
        camera_info = req.camera_info

        # uint8 (0 backgoround, 255 mask)
        seg_pothole_mask = segment_defect(cv_image, "pothole")

        # Publish the pothole mask
        segmentation_image = self._cv_bridge.cv2_to_imgmsg(seg_pothole_mask, encoding="passthrough")
        segmentation_image.header = req.image_rgb.header
        self._segmentation_pub.publish(segmentation_image)

        # Find connected pothole regions using connected components
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
            seg_pothole_mask, connectivity=8)

        if num_labels <= 1:  # No potholes detected
            rospy.logwarn("No potholes found.")
            response = FindPotholeResponse()
            response.success = False
            return response

        # Identify the largest pothole (ignoring label 0, which is background)
        largest_label = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
        largest_pothole_mask = (labels == largest_label)

        # Compute depth values for the largest pothole
        pothole_depths = depth_image[largest_pothole_mask]
        avg_depth = np.mean(pothole_depths) if pothole_depths.size > 0 else 0

        # Get real-world coordinates of the largest pothole's center of mass
        center_x, center_y = centroids[largest_label]
        real_x = (center_x - camera_info.K[2]) * avg_depth / camera_info.K[0]
        real_y = (center_y - camera_info.K[5]) * avg_depth / camera_info.K[4]
        real_z = avg_depth

        # Compute surface area
        surface_area = self.find_surface_area(
            largest_pothole_mask, depth_image, camera_info)
        rospy.loginfo(f"Pothole Surface Area: {surface_area:.4f} m^2")

        # Construct PoseStamped message for center of mass
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "front_rgbd_camera_rgb_camera_optical_frame"
        pose_msg.pose.position.x = real_x
        pose_msg.pose.position.y = real_y
        pose_msg.pose.position.z = real_z

        # Prepare response
        response = FindPotholeResponse()
        response.center_of_mass = pose_msg
        response.surface_area_m = surface_area
        response.success = True

        return response

    def find_surface_area(self, mask, depth_image, camera_info):
        """
        Compute the real-world surface area of a pothole using depth information.

        Args:
            mask (np.ndarray): Binary mask (1 = pothole, 0 = background).
            depth_image (np.ndarray): Corresponding depth image (in meters).
            camera_info (sensor_msgs.msg.CameraInfo): Camera intrinsics.

        Returns:
            float: Surface area in square meters.
        """

        # Get focal length from camera intrinsics
        K = np.array(camera_info.K).reshape(3, 3)
        # Focal lengths (determines how pixels map to real-world distances)
        fx, fy = K[0, 0], K[1, 1]

        # Find indices of pothole pixels
        pothole_pixels = np.argwhere(mask)

        # If no pothole detected, return 0 area
        if pothole_pixels.size == 0:
            return 0.0

        # Extract depth values for pothole pixels
        depths = depth_image[pothole_pixels[:, 0], pothole_pixels[:, 1]]

        # Approximate mean depth (avoid zero-depth areas)
        valid_depths = depths[depths > 0]
        if valid_depths.size == 0:
            return 0.0  # No valid depth data

        avg_depth = np.mean(valid_depths)

        # Calculate real-world pixel area using depth and focal length
        pixel_size_x = avg_depth / fx
        pixel_size_y = avg_depth / fy

        # Total pothole area in square meters
        pothole_area = pixel_size_x * pixel_size_y * pothole_pixels.shape[0]

        return pothole_area

    def get_bool_callback(self, req: GetBoolRequest):
        print("Getting a bool")
        return GetBoolResponse()

    def start_spin(self):
        rospy.loginfo("Idling until exit")
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
