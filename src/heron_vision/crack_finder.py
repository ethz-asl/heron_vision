import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from heron_msgs.srv import *
from robotnik_msgs.srv import *
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image
from .inference import segment_defect


class CrackFinder:
    def __init__(self, name):
        # Register a node with the ROS Master server and register a shutdown hook
        #    - only one node allowed per process
        rospy.init_node(name)

        self._cv_bridge = CvBridge()

        # Create a service server for the find_cracks service.
        self._crack_service = rospy.Service(
            "find_cracks", FindCrack, self.find_crack_callback)
        self._get_bool_service = rospy.Service(
            "get_bool", GetBool, self.get_bool_callback)

        # Also publish the segmentation on a topic.
        self._segmentation_pub = rospy.Publisher(
            "~/crack_segmentation", Image, queue_size=1, latch=True)

    # Define a callback for the cracks server.

    def find_crack_callback(self, req: FindCrackRequest):
        print("Finding some cracks")

        # Save RGB image
        cv_image = self._cv_bridge.imgmsg_to_cv2(
            req.image_rgb, desired_encoding='passthrough')
        print(cv_image.shape)

        # Save depth image
        depth_image = self._cv_bridge.imgmsg_to_cv2(
            req.image_depth, desired_encoding="passthrough")

        # Save camera info
        camera_info = req.camera_info

        seg_crack_mask = segment_defect(cv_image, "crack") # uint8 (0 backgoround, 255 mask)

        # Set up everything for morphological operations
        kernel = np.ones((5,5),np.uint8)

        # Apply closing
        seg_crack_mask = cv2.morphologyEx(seg_crack_mask, cv.MORPH_CLOSE, kernel)

        # Find connected components (separate cracks)
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(seg_crack_mask, connectivity=8)

        if num_labels <= 1:  # No cracks found
            rospy.logwarn("No cracks detected.")
            response = FindCrackResponse()
            response.success = False
            return response

        # Find the largest crack (excluding background)
        largest_crack_idx = np.argmax(stats[1:, cv2.CC_STAT_AREA]) + 1
        largest_crack_mask = (labels == largest_crack_idx)

        # Find the 3D points of the crack
        crack_3d_points = self.extract_crack_3d_points(largest_crack_mask, depth_image, camera_info)

        if not crack_3d_points:
            rospy.logwarn("No valid depth data for crack.")
            response = FindCrackResponse()
            response.success = False
            return response

        # Get the closest and farthest 3D points
        start_3d, end_3d = self.find_farthest_crack_points(crack_3d_points)

        # Create PointStamped messages
        start_point_msg = PointStamped()
        start_point_msg.header.frame_id = req.camera_info.header.frame_id
        start_point_msg.point.x, start_point_msg.point.y, start_point_msg.point.z = start_3d

        end_point_msg = PointStamped()
        end_point_msg.header.frame_id = req.camera_info.header.frame_id
        end_point_msg.point.x, end_point_msg.point.y, end_point_msg.point.z = end_3d

        # Convert segmentation mask to ROS Image
        segmentation_mask_msg = self._cv_bridge.cv2_to_imgmsg(seg_crack_mask, encoding="mono8")
        segmentation_mask_msg.header = req.image_rgb.header
        self._segmentation_pub.publish(segmentation_mask_msg)

        # Prepare response
        response = FindCrackResponse()
        response.start_point = start_point_msg
        response.end_point = end_point_msg
        response.segmentation_mask = segmentation_mask_msg
        response.success = True

        return response

    def extract_crack_3d_points(self, mask, depth_image, camera_info):
        """Extracts 3D coordinates of the crack pixels using camera intrinsics."""
        K = np.array(camera_info.K).reshape(3, 3)
        fx, fy = K[0, 0], K[1, 1]    # Focal lengths (determines how pixels map to real-world distances)
        cx, cy = K[0, 2], K[1, 2]    # Principal point (optical center)

        crack_pixels = np.argwhere(mask > 0)  # Get (v, u) indices

        crack_3d_points = []
        for (v, u) in crack_pixels:
            z = depth_image[v, u]/1000.0  # Depth at pixel (u, v)
            if z > 0:  # Avoid invalid depth
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy
                crack_3d_points.append((x, y, z))

        return crack_3d_points

    def find_farthest_crack_points(self, crack_3d_points):
        """Finds the closest and farthest points along the Z-axis."""
        crack_3d_points.sort(key=lambda p: p[2])  # Sort by Z-depth (distance)
        return crack_3d_points[0], crack_3d_points[-1]

    def get_bool_callback(self, req: GetBoolRequest):
        print("Getting a bool")
        return GetBoolResponse()

    def start_spin(self):
        rospy.loginfo("Idling until exit")
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
