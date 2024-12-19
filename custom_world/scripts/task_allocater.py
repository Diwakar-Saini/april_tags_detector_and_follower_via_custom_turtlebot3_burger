#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from simple_pid import PID
from apriltag_ros.msg import AprilTagDetectionArray

# Global variable to store the current tag name to search for and its corresponding tag ID
target_tag = ""
target_tag_id = None

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

def callback(data):
    """
    Callback function for processing AprilTag detections.
    """
    global goal_reached  # Use the global flag

    try:
        # If the goal has already been reached, stop the bot and do nothing
        if goal_reached:
            rospy.loginfo("Goal has been reached. Waiting for new task.")
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            velocity_publisher.publish(vel_msg)
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
                    tag_z = tag_pose.z  # Distance to the tag (forward)
                    tag_x = tag_pose.x  # Horizontal offset to the tag

                    # Calculate distance and angle
                    dis = (tag_z**2 + tag_x**2)**0.5  # Euclidean distance
                    ang = tag_x / tag_z if tag_z != 0 else 0  # Angle approximation

                    rospy.loginfo(f"Distance: {dis}, Angle: {ang}")

                    # PID outputs
                    control_lin = pid_linear(dis)
                    control_ang = pid_ang(ang)
                    control_ang2 = pid_ang2(ang)

                    # Clamp linear velocity
                    control_lin = max(min(control_lin, 0.5), -0.5)

                    # Clamp angular velocity
                    control_ang = max(min(control_ang, 0.5), -0.5)

                    # Movement logic
                    if dis > 0.5:  # Approach the tag
                        vel_msg.linear.x = -control_lin
                        vel_msg.angular.z = control_ang
                        rospy.loginfo("Moving towards the tag")
                    # Stop the bot if the goal is reached
                    elif dis < 0.6:
                            vel_msg.linear.x = 0
                            vel_msg.angular.z = 0
                            rospy.loginfo("Goal reached, stopping bot...")

                            # Optionally, you can publish a message to indicate the goal has been reached.
                            rospy.loginfo("Goal reached: Stopped. Ready for new task.")
                            goal_reached = True  # Set the flag to True when goal is reached
                            break  # Stop processing further detections if the target tag is reached    
                    else:  # Fine angular adjustment near the tag
                        vel_msg.linear.x = 0
                        vel_msg.angular.z = control_ang2
                        rospy.loginfo("Fine adjustment near the tag")

                    # Publish velocity command
                    velocity_publisher.publish(vel_msg)
                    break  # Stop processing further detections if the target tag is found

        else:
            # No detections: rotate to search (if goal hasn't been reached)
            if not goal_reached:
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0.5  # Rotate to search for the tag
                velocity_publisher.publish(vel_msg)
                rospy.loginfo("No tags detected, rotating to search")

    except Exception as e:
        rospy.logwarn(f"Callback error: {e}")
        # Default behavior on error
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0.5
        velocity_publisher.publish(vel_msg)
        rospy.loginfo("Fallback behavior: rotating")

if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('apriltag_follower')

    # Publisher for velocity commands
    velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    # PID controllers
    pid_linear = PID(1.0, 0.0, 0.0, setpoint=0.0)  # Linear velocity control
    pid_ang = PID(1.0, 0.0, 0.0, setpoint=0.0)    # Angular velocity control
    pid_ang2 = PID(1.0, 0.01, 0.0, setpoint=0.0)  # Fine angular adjustment

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
