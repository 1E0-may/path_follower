#!/usr/bin/env python
import numpy as np
import angles
import os
import pandas as pd
import rospy
from tf import transformations
from geometry_msgs.msg import Twist, Point32
from nav_msgs.msg import Odometry

__authors__ = ['Leo Herszenhaut']
__version__ = '0.0.1'

PATH_FILE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "path.csv")


class PathFollower:

    def __init__(self):

        rospy.init_node("path_follower")
        rospy.loginfo("Starting Path Follower")
        # internal variables
        self._robot_rpy = None
        self._path = pd.read_csv(PATH_FILE_PATH, header=None).values.reshape((-1, 2))
        self._idx = 2
        # publishers
        self._cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self._angle_debug = rospy.Publisher("/debug/angle", Point32, queue_size=1)
        # subscribers
        rospy.Subscriber("/odom", Odometry, self.imuDataCb, queue_size=1)
        # run
        rospy.spin()

    def imuDataCb(self, data):

        quaternion = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w)
        self._robot_rpy = transformations.euler_from_quaternion(quaternion)

        self._angle_debug.publish(Point32(x=self._robot_rpy[0],
                                          y=self._robot_rpy[1],
                                          z=self._robot_rpy[2]))

        heading = angles.normalize_angle_positive(
            np.arctan2(-data.pose.pose.position.y + self._path[self._idx][1],
                       -data.pose.pose.position.x + self._path[self._idx][0]))
        angle_diff = angles.shortest_angular_distance(heading, self._robot_rpy[2])

        distance = np.sqrt((data.pose.pose.position.x - self._path[self._idx][0]) ** 2 +
                           (data.pose.pose.position.y - self._path[self._idx][1]) ** 2)

        rospy.loginfo("({:.2f}, {:.2f}) Heading to {:.2f} bearing {:.2f} | ({:.2f}, {:.2f}) Next Target at {:.4f} [m]".format(
            data.pose.pose.position.x, data.pose.pose.position.y, heading, self._robot_rpy[2],
            self._path[self._idx][0], self._path[self._idx][1], distance))

        tmp = Twist()
        if np.rad2deg(np.abs(angle_diff)) >= 10:
            rospy.loginfo("Angular Distance {:.2f} [deg]".format(np.rad2deg(angle_diff)))
            tmp.angular.z = -0.5 * angle_diff
            self._cmd_vel.publish(tmp)
        elif distance <= 0.1:
            rospy.loginfo("Moving to next target")
            self._idx += 1
            if self._idx == len(self._path):
                rospy.loginfo("Target Reached")
                self._cmd_vel.publish(tmp)
                rospy.signal_shutdown("Target Reached")
        else:
            rospy.loginfo("Distance {:.2} [m]".format(distance))
            tmp.linear.x = 0.5
            self._cmd_vel.publish(tmp)


if __name__ == "__main__":

    path_follower = PathFollower()
