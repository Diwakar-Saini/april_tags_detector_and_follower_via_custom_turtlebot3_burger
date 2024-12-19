#!/usr/bin/env python3
import tf2_ros
import tf2_geometry_msgs
import rospy
from geometry_msgs.msg import PoseStamped
from apriltag_ros.msg import AprilTagDetectionArray
import math
import tf.transformations
from geometry_msgs.msg import Quaternion

# Global variable to store the current tag name to search for and its corresponding tag ID
target_tag = ""
target_tag_id = None

# Publisher for move_base simple goal
move_base_goal_pub = None

# Global variable to track if the goal is reached
goal_reached = False

# Function to get tag ID based on tag name
def get_tag_id_by_name(tag_name):
    if tag_name == "pallet":
        return 0
    elif tag_name == "charging_station":
        return 10
    elif tag_name == "cart":
        return 8
    else:
        rospy.logwarn(f"No tag ID found for {tag_name}")
        return None  # Return None if tag is not found

def calculate_distance(x1, y1, x2, y2):
    """
    Calculate the Euclidean distance between two points.
    """
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def callback(data):
    """
    Callback function for processing AprilTag detections.
    """
    global goal_reached  # Use the global flag

    try:
        # If the goal has already been reached, do nothing
        if goal_reached:
            rospy.loginfo("Goal has been reached. Waiting for new task.")
            return  # Exit the callback early without doing anything further

        # Ensure there are detections
        if len(data.detections) > 0:
            rospy.loginfo(f"Detected {len(data.detections)} tags")
            # Check if the detected tag matches the target tag ID
            for detection in data.detections:
                tag_id = detection.id[0]  # Extract the actual tag ID from the tuple
                rospy.loginfo(f"Detected tag ID: {tag_id}")
                
                if tag_id == target_tag_id:  # Match the tag ID to the target tag ID
                    tag_pose = detection.pose.pose.pose.position
                    tag_orientation = detection.pose.pose.pose.orientation
                    tag_z = tag_pose.z  # Distance to the tag (forward, becomes x in goal frame)
                    tag_x = tag_pose.x  # Horizontal offset to the tag (becomes y in goal frame)

                    # Check if the distance to the tag is less than the threshold
                    distance_to_tag = calculate_distance(0, 0, tag_z, tag_x)
                    rospy.loginfo(f"Distance to tag: {distance_to_tag:.2f} meters")
                    
                    if distance_to_tag < 0.6:  # Goal is considered reached
                        rospy.loginfo("Goal reached!")
                        goal_reached = True
                        return
                    
                    # Publish the goal if not yet reached
                    publish_goal(tag_pose,tag_orientation)
    except Exception as e:
        rospy.logerr(f"Error in callback: {e}")

def publish_goal(tag_pose, tag_orientation):
    """
    Transforms the detected tag pose to the map frame and publishes it as a goal for move_base.
    """
    try:
        # Create a TF buffer and listener
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        # Define the detected pose in the base_link frame
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "base_link"
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.pose.position.x = tag_pose.z  # Use tag_z as x in base_link
        goal_pose.pose.position.y = tag_pose.x  # Use tag_x as y in base_link
        goal_pose.pose.position.z = tag_pose.y      # Ignore z for navigation
        roll = 0.01
        pitch = -0.05
        yaw = 1.38  # You can modify this value based on how you want the robot to be oriented

        # Create the quaternion from roll, pitch, yaw
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        # Assign the quaternion to the goal pose
        goal_pose.pose.orientation = Quaternion(*quaternion)

        # Wait for the transform from base_link to map
        tf_buffer.can_transform("map", "base_link", rospy.Time(0), rospy.Duration(1.0))

        # Transform the pose to the map frame
        transformed_goal = tf2_geometry_msgs.do_transform_pose(
            goal_pose,
            tf_buffer.lookup_transform("map", "base_link", rospy.Time(0))
        )

        # Publish the transformed goal
        move_base_goal_pub.publish(transformed_goal)
        rospy.loginfo("Published goal to move_base: %s", transformed_goal)
    except Exception as e:
        rospy.logerr("Error transforming and publishing goal: %s", e)


if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('apriltag_follower')

    # Initialize the publisher
    move_base_goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

    # Prompt for user input to choose a tag to search
    target_tag = input("Enter the tag name to search (e.g., 'charging_station' or 'pallet' or 'cart'): ")

    # Get the corresponding tag ID based on user input
    target_tag_id = get_tag_id_by_name(target_tag)

    if target_tag_id is None:
        rospy.logwarn("No valid tag ID found for the entered tag name. Exiting...")
        exit(1)
    else:
        rospy.loginfo(f"Searching for tag ID: {target_tag_id} ({target_tag})")

    # Subscribe to AprilTag detections
    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, callback)

    # Keep the node running
    rospy.spin()
