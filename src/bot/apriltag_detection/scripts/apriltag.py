#!/usr/bin/env python3

import json
import rospy
import tf
from apriltag_ros.msg import AprilTagDetectionArray

# Dictionary to store AprilTag poses
apriltag_poses = {}

def apriltag_callback(detections):
    for detection in detections.detections:
        tag_id = detection.id[0]
        tag_key = f'tag_{tag_id}'
        print(tag_key)  # Output the tag name

        # Initially store the pose from the detection
        apriltag_poses[tag_id] = detection.pose.pose.pose

        try:
            # Attempt to transform the tag pose to the 'map' frame
            (translation, rotation) = transform_listener.lookupTransform(tag_key, 'map', rospy.Time(0))
            apriltag_poses[tag_id] = (translation, rotation)
            print("Reaching callback of AprilTag detection")
        except tf.Exception as e:
            rospy.loginfo(f"Transform not found: {e}")
            continue

if __name__ == '__main__':
    rospy.init_node('apriltag_detection_node', anonymous=True)
    transform_listener = tf.TransformListener()
    rospy.loginfo("AprilTag detection node initialized")
    
    # Subscriber to AprilTag detections
    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, apriltag_callback)
    
    # Keep the node running
    rospy.spin()
    
    # Save the detected AprilTag poses to a JSON file
    with open('apriltag_poses.json', 'w') as file:
        json.dump(apriltag_poses, file)
