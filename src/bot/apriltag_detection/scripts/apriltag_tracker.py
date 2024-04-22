#!/usr/bin/env python3

import rospy
from tf import TransformListener, TransformBroadcaster
from geometry_msgs.msg import Twist, PoseStamped
from apriltag_ros.msg import AprilTagDetectionArray

class ApriltagTracker:
    def __init__(self):
        rospy.init_node('apriltag_tracker', log_level=rospy.DEBUG)
        rospy.loginfo("Apriltag tracker node initialized.")

        # Configuration parameters
        self.detection_topic = rospy.get_param('~detection_topic', '/tag_detections')
        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel')
        self.angular_coefficient = rospy.get_param('~angular_coefficient', 2)
        self.map_frame = rospy.get_param('~map_frame', '/map')

        self.tf_listener = TransformListener()
        self.tag_poses = {}

        # Subscriptions
        rospy.Subscriber(self.detection_topic, AprilTagDetectionArray, self.tag_detection_callback)
        rospy.Subscriber(self.cmd_vel_topic, Twist, self.cmd_vel_callback)

        # Publisher for tag poses in map frame
        self.pose_publisher = rospy.Publisher('mapped_tags', PoseStamped, queue_size=10)

    def tag_detection_callback(self, msg):
        current_velocity_score = self.calculate_velocity_score()

        for detection in msg.detections:
            rospy.logdebug(f"Detected tag: {detection.id}")
            map_pose = self.transform_to_map(detection.pose.pose.pose)

            try:
                if self.tag_poses[detection.id][0] > current_velocity_score:
                    self.tag_poses[detection.id] = (current_velocity_score, map_pose)
            except KeyError:
                self.tag_poses[detection.id] = (current_velocity_score, map_pose)

    def cmd_vel_callback(self, msg):
        self.velocity_score = msg.linear.x**2 + msg.linear.y**2 + msg.linear.z**2 + \
                              self.angular_coefficient * (msg.angular.x**2 + msg.angular.y**2 + msg.angular.z**2)
        rospy.logdebug(f"Velocity Score: {self.velocity_score}")

    def transform_to_map(self, pose):
        pose_stamped = PoseStamped(header=pose.header, pose=pose)
        return self.tf_listener.transformPose(self.map_frame, pose_stamped)

    def publish_tag_transforms(self):
        broadcaster = TransformBroadcaster()
        for id, (_, pose) in self.tag_poses.items():
            broadcaster.sendTransform((pose.position.x, pose.position.y, pose.position.z),
                                      (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                                      rospy.Time.now(), f"map_tag_{id}", self.map_frame)

    def spin(self):
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            self.publish_tag_transforms()
            rate.sleep()

if __name__ == '__main__':
    try:
        tracker = ApriltagTracker()
        tracker.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Apriltag tracker node terminated.")
