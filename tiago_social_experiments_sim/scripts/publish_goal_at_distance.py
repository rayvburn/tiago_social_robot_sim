#!/usr/bin/env python

import rospy
import sys
from people_msgs.msg import People
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, PoseStamped
import math
import yaml

class HumanDistanceChecker:
    def __init__(self, goal_pose_file):
        # Initialize the ROS node
        rospy.init_node('human_distance_checker')

        # Parameters
        # Name of the target human
        self.target_human_name = rospy.get_param('~target_human_name', 'target_human')
        # Distance threshold (10 meters by default)
        self.distance_threshold = rospy.get_param('~distance_threshold', 10.0)
        # One-shot operation means that if the distance threshold has been violated and the goal has been sent,
        # class' the subscribers will be unregistered
        self.one_shot = rospy.get_param('~one_shot', True)

        # Subscribers
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.people_sub = rospy.Subscriber('/people', People, self.people_callback)

        # Publisher
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        # Initialize variables
        self.robot_position = Point()
        self.target_human_position = None

        rospy.loginfo(
            "Tracking distance between the robot and a human '" + str(self.target_human_name) + "'. "
            "The threshold is " + str(self.distance_threshold) + ", once violated, the node will publish the "
            "navigation goal from file at " + str(goal_pose_file) + "'"
        )

        # Load the goal pose from the YAML file
        self.goal_pose = self.load_goal_pose(goal_pose_file)

    def load_goal_pose(self, file_path):
        """Load the goal pose from a YAML file."""
        with open(file_path, 'r') as file:
            try:
                pose_data = yaml.safe_load(file)
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = pose_data['header']['frame_id']
                pose_stamped.header.stamp = rospy.Time.now()
                pose_stamped.pose.position.x = pose_data['pose']['position']['x']
                pose_stamped.pose.position.y = pose_data['pose']['position']['y']
                pose_stamped.pose.position.z = pose_data['pose']['position']['z']
                pose_stamped.pose.orientation.x = pose_data['pose']['orientation']['x']
                pose_stamped.pose.orientation.y = pose_data['pose']['orientation']['y']
                pose_stamped.pose.orientation.z = pose_data['pose']['orientation']['z']
                pose_stamped.pose.orientation.w = pose_data['pose']['orientation']['w']
                return pose_stamped
            except yaml.YAMLError as e:
                rospy.logerr("Error loading YAML file: " + e)
                return None

    def odom_callback(self, msg):
        """Callback for the odometry topic."""
        self.robot_position = msg.pose.pose.position

    def people_callback(self, msg):
        """Callback for the people topic."""
        for person in msg.people:
            if str(person.name) == str(self.target_human_name):
                self.target_human_position = person.position
                self.check_distance()
                break

    def check_distance(self):
        """Check the distance between the robot and the target human."""
        if self.target_human_position is not None:
            distance = self.calculate_distance(self.robot_position, self.target_human_position)
            rospy.loginfo_throttle(
                2.0,
                "Distance from the robot to '" + str(self.target_human_name) + "' is " + str(distance) + " m"
            )
            if distance <= self.distance_threshold and self.goal_pose is not None:
                rospy.loginfo(
                    "'" + str(self.target_human_name) + "' is within the range of "
                    + str(self.distance_threshold) + "m! Sending goal."
                )
                self.goal_pub.publish(self.goal_pose)
                if self.one_shot:
                    self.unregister_subscribers()

    def unregister_subscribers(self):
        self.odom_sub.unregister()
        self.people_sub.unregister()

    @staticmethod
    def calculate_distance(position1, position2):
        """Calculate Euclidean distance between two points."""
        return math.sqrt(
            (position1.x - position2.x) ** 2 +
            (position1.y - position2.y) ** 2 +
            (position1.z - position2.z) ** 2
        )

if __name__ == '__main__':
    if len(sys.argv) < 2:
        rospy.logerr("Usage: rosrun your_package_name human_distance_checker.py /path/to/goal_pose.yaml")
        sys.exit(1)

    goal_pose_file = sys.argv[1]

    try:
        distance_checker = HumanDistanceChecker(goal_pose_file)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
