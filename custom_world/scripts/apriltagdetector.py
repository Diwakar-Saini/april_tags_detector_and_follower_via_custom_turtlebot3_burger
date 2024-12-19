import rospy
import cv2
import tf2_ros
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf.transformations import quaternion_from_euler
import tf2_geometry_msgs
import tf.transformations

class AprilTagDetector:
    def __init__(self):
        rospy.init_node("april_tag_detector", anonymous=True)

        # Camera parameters
        self.camera_matrix = np.array([[1206.89, 0, 960], [0, 1206.89, 540], [0, 0, 1]])  # Example values
        self.dist_coeffs = np.zeros(5)  # Assuming no distortion
        self.tag_size = 0.2  # Size of the tag in meters

        # ROS Subscribers and Publishers
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/bot1/camera/rgb/image_raw", Image, self.image_callback)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.goal_pub = rospy.Publisher("/bot1/move_base_simple/goal", PoseStamped, queue_size=10)

        # TF2 setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Marker detection state
        self.marker_detected = False

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Detect AprilTags using OpenCV
            dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
            parameters = cv2.aruco.DetectorParameters()
            corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)

            if ids is not None:
                self.marker_detected = True
                for i, corner in enumerate(corners):
                    # Estimate pose of each marker
                    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corner, self.tag_size, self.camera_matrix, self.dist_coeffs)

                    # Convert rvec to a quaternion
                    # quaternion = quaternion_from_euler(*rvec[0][0])  # rvec is a 3x1 rotation vector
                    # Convert rvec to rotation matrix
                    rotation_matrix, _ = cv2.Rodrigues(rvec)
        
                     # Extract yaw (rotation around Z-axis) from the rotation matrix
                    yaw = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
        
                     # Construct a quaternion with roll = 0, pitch = 0, and the calculated yaw
                    quaternion = quaternion_from_euler(0, 0, yaw)


                    # Publish transforms in camera_link frame
                    self.publish_tf(ids[i][0], tvec[0][0], quaternion, "bot1/camera_link")

                    # Transform to base_footprint frame
                    tvec_base_footprint = self.transform_to_base_footprint(tvec[0][0])
                    self.publish_tf(ids[i][0], tvec_base_footprint, quaternion, "bot1/base_footprint")

                    # Transform to map frame and publish goal
                    self.publish_goal_in_map_frame(ids[i][0], tvec_base_footprint, quaternion)
                    rospy.sleep(.002)

                    # Draw detected marker and axes
                    cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                    cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec[0][0], tvec[0][0], 0.1)
            else:
                # No marker detected
                self.marker_detected = False

            # Show the image with detections
            resized_frame = cv2.resize(frame, (720, 640))
            cv2.imshow("AprilTag Detection", resized_frame)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(f"Error in image_callback: {e}")

    def publish_tf(self, marker_id, tvec, quaternion, parent_frame):
        try:
            transform = TransformStamped()
            transform.header.stamp = rospy.Time.now()
            transform.header.frame_id = parent_frame
            transform.child_frame_id = f"tag_{marker_id}"

            # Set translation
            transform.transform.translation.x = tvec[0]
            transform.transform.translation.y = tvec[1]
            transform.transform.translation.z = tvec[2]

            # Set rotation
            transform.transform.rotation.x = quaternion[0]
            transform.transform.rotation.y = quaternion[1]
            transform.transform.rotation.z = quaternion[2]
            transform.transform.rotation.w = quaternion[3]

            self.tf_broadcaster.sendTransform(transform)
        except Exception as e:
            rospy.logerr(f"Error publishing TF for tag {marker_id}: {e}")

    def transform_to_base_footprint(self, tvec):
    # Static transform (example values)
        tvec_corrected = np.array([
            tvec[2],  # Z (camera) → X (base_footprint)
            -tvec[0],  # X (camera) → -Y (base_footprint)
            -tvec[1]   # Y (camera) → -Z (base_footprint)
        ])
    
    # Translation vector from base_link to camera_link (if necessary)
        camera_to_base = np.array([0.036, -0.065, 0.094])  # Example values, adjust based on actual configuration
    
    # Combine both transformations if needed (e.g., corrected tvec + camera-to-base offset)
        return tvec_corrected + camera_to_base


    def publish_goal_in_map_frame(self, marker_id, tvec, quaternion):
     try:
        # Create a PoseStamped message in the base_footprint frame
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "bot1/base_footprint"  # This is where the goal is initially created

        goal.pose.position.x = tvec[0]
        goal.pose.position.y = tvec[1]
        goal.pose.position.z = tvec[2]
        goal.pose.orientation.x = quaternion[0]
        goal.pose.orientation.y = quaternion[1]
        goal.pose.orientation.z = quaternion[2]
        goal.pose.orientation.w = quaternion[3]

        # Transform the goal to the map frame
        transformed_goal = self.transform_to_map(goal)
        if transformed_goal:
            transformed_goal.header.frame_id = "map"  # Ensure the frame is set to map before publishing
            self.goal_pub.publish(transformed_goal)  # Publish the goal in map frame
            rospy.loginfo(f"Published goal for tag {marker_id} in map frame: {transformed_goal.pose}")
        else:
            rospy.logwarn(f"Failed to transform and publish goal for tag {marker_id}")
     except Exception as e:
        rospy.logerr(f"Error publishing goal in map frame for tag {marker_id}: {e}")


  
    def transform_to_map(self, pose):
     try:
        # Lookup the transform from base_footprint to map
        transform = self.tf_buffer.lookup_transform(
            "map",  # Target frame
            pose.header.frame_id,  # Source frame (base_footprint)
            rospy.Time(0),  # Use the latest transform
            rospy.Duration(1.0)  # Timeout duration
        )
        # Transform the pose
        transformed_pose = tf2_geometry_msgs.do_transform_pose(pose, transform)
        return transformed_pose
     except Exception as e:
        rospy.logerr(f"Error transforming pose to map frame: {e}")
        return None



if __name__ == "__main__":
    try:
        detector = AprilTagDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
