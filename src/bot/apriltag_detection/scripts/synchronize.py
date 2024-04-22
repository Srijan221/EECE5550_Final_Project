#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo

# Publishers for synchronized image and camera information
image_publisher = rospy.Publisher('synced_image', Image, queue_size=10)
camera_info_publisher = rospy.Publisher('synced_camera_info', CameraInfo, queue_size=10)

# Global variable to store the latest camera information
stored_camera_info = None

def camera_info_handler(camera_info):
    global stored_camera_info
    stored_camera_info = camera_info

def image_handler(image):
    # Process image and camera info
    if stored_camera_info is None:
        return
    # Update the timestamp of the camera info to match the image's timestamp
    stored_camera_info.header.stamp = image.header.stamp
    # Publish the synchronized image and camera info
    image_publisher.publish(image)
    camera_info_publisher.publish(stored_camera_info)

if __name__ == '__main__':
    rospy.init_node('synchronizer')
    # Subscribers to raw image and camera info topics
    rospy.Subscriber('usb_cam/image_raw', Image, image_handler)
    rospy.Subscriber('usb_cam/camera_info', CameraInfo, camera_info_handler)

    # Keep the node running
    rospy.spin()
