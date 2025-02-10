import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from heron_msgs.srv import *
from robotnik_msgs.srv import *
from .inference import segment_defect 


class LaneEdgeFinder:
    def __init__(self, name):
        rospy.init_node(name)

        self._cv_bridge = CvBridge()

        # Create a service server for the find_laneEdges service.
        self._lane_edge_service = rospy.Service(
            "find_lane_edges", FindLaneEdge, self.find_lane_edge_callback)

    def find_lane_edge_callback(self, req: FindLaneEdgeRequest):
        rospy.loginfo("Finding lane edges")

        # Convert RGB image
        cv_image = self._cv_bridge.imgmsg_to_cv2(
            req.image_rgb, desired_encoding='passthrough')

        # Convert depth image
        depth_image = self._cv_bridge.imgmsg_to_cv2(
            req.image_depth, desired_encoding="passthrough")

        # Extract camera info
        camera_info = req.camera_info

        # Segment the lane edge
        seg_lane_edge_mask = segment_defect(cv_image, "lane_edge")  # Binary mask (0 background, 255 mask)

        # Find connected components (separate lane edges)
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(seg_lane_edge_mask, connectivity=8)

        if num_labels <= 1:  # No lane edges detected
            rospy.logwarn("No lane edges detected.")
            response = FindLaneEdgeResponse()
            response.success = False
            return response

        # Find the leftmost or rightmost lane edge component
        if req.is_right_side:
            lane_edge_idx = np.argmax(centroids[1:, 0]) + 1  # Rightmost
        else:
            lane_edge_idx = np.argmin(centroids[1:, 0]) + 1  # Leftmost

        lane_edge_mask = (labels == lane_edge_idx)

        # Extract 3D points from the detected lane edge
        lane_3d_points = self.extract_3d_points(lane_edge_mask, depth_image, camera_info)

        if not lane_3d_points:
            rospy.logwarn("No valid depth data for lane edge.")
            response = FindLaneEdgeResponse()
            response.success = False
            return response

        # Get the closest and farthest 3D points
        start_3d, end_3d = self.find_farthest_points(lane_3d_points)

        # Create PointStamped messages
        start_point_msg = PointStamped()
        start_point_msg.header.frame_id = req.camera_info.header.frame_id
        start_point_msg.point.x, start_point_msg.point.y, start_point_msg.point.z = start_3d

        end_point_msg = PointStamped()
        end_point_msg.header.frame_id = req.camera_info.header.frame_id
        end_point_msg.point.x, end_point_msg.point.y, end_point_msg.point.z = end_3d

        # Prepare response
        response = FindLaneEdgeResponse()
        response.start_point = start_point_msg
        response.end_point = end_point_msg
        response.success = True

        return response

    def extract_3d_points(self, mask, depth_image, camera_info):
        """Extracts 3D coordinates of the detected lane edge using camera intrinsics."""
        K = np.array(camera_info.K).reshape(3, 3)
        fx, fy = K[0, 0], K[1, 1]    # Focal lengths (determines how pixels map to real-world distances)
        cx, cy = K[0, 2], K[1, 2]    # Principal point (optical center)

        lane_pixels = np.argwhere(mask > 0)  # Get (v, u) indices

        lane_3d_points = []
        for (v, u) in lane_pixels:
            z = depth_image[v, u]  # Depth at pixel (u, v)
            if z > 0:  # Avoid invalid depth
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy
                lane_3d_points.append((x, y, z))

        return lane_3d_points

    def find_farthest_points(self, lane_3d_points):
        """Finds the closest and farthest points along the Z-axis."""
        lane_3d_points.sort(key=lambda p: p[2])  # Sort by Z-depth (distance)
        return lane_3d_points[0], lane_3d_points[-1]

    def start_spin(self):
        rospy.loginfo("Idling until exit")
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
