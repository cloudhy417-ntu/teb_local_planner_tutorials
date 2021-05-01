#!/usr/bin/env python

# Author: franz.albers@tu-dortmund.de
import sys
import rospy, math, tf
import numpy as np
import pandas as pd
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from std_msgs.msg import Header, ColorRGBA, Int64
from geometry_msgs.msg import PoseWithCovarianceStamped, PolygonStamped, Point32, QuaternionStamped, Quaternion, TwistWithCovariance, Pose, Point, Vector3
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty

class Robot():
    def __init__(self):
        self.sim_time = -1
        self.start_time = 780
        rospy.wait_for_service('/reset_positions')
        self.stage_reset_pos = rospy.ServiceProxy('/reset_positions', Empty)
        self.amcl_reset_pos = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.sim_time_sub = rospy.Subscriber('sim_time', Int64, self.sim_timeCB)
        self.client.wait_for_server()
    def sim_timeCB(self, sim_time):
        self.sim_time = sim_time.data
        if self.sim_time == self.start_time:
            self.start_time += 30
            self.send_goal(5,0,0)

    def reset_pos(self):
        self.stage_reset_pos.call()
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = '/map'
        initial_pose.pose.pose.position.x = -5
        initial_pose.pose.pose.position.y = -0
        q = quaternion_from_euler(0,0,0)
        initial_pose.pose.pose.orientation.z = q[2]
        initial_pose.pose.pose.orientation.w = q[3]
        self.amcl_reset_pos.publish(initial_pose)
        # rospy.sleep(1.)
    def send_goal(self, x, y, theta):
        t0 = rospy.Time.now()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        orientation = quaternion_from_euler(0,0,theta)
        goal.target_pose.pose.orientation.z = orientation[2]
        goal.target_pose.pose.orientation.w = orientation[3]
        self.client.send_goal(goal)
        finished = self.client.wait_for_result(timeout = rospy.Duration(20.0))
        t1 = rospy.Time.now()
        if finished:
            print('{} travelling time:{}.{}'.format(self.start_time-30, (t1-t0).secs, (t1-t0).nsecs))
        else:
            print('{} timed out'.format(self.start_time-30))
        self.reset_pos()
if __name__ == '__main__': 
    try:
        rospy.init_node("send_goal")
        robot = Robot()
        rospy.spin()
        # while not rospy.is_shutdown()
        #     robot.send_goal(0,3,math.pi/2)
    except rospy.ROSInterruptException:
        print("finished playback")
        pass
# demo scenerio 4236

