#!/usr/bin/env python
import sys, math
from os import path
from collections import OrderedDict
import rospy
import tf
import pickle
import message_filters
from std_msgs.msg import Int64
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Pose
from costmap_converter.msg import ObstacleArrayMsg
class Summarizer():
    def __init__(self):
        robot_pose_sub = message_filters.Subscriber('/base_pose_ground_truth', Odometry)
        obst_pose_sub = message_filters.Subscriber('/move_base/TebLocalPlannerROS/obstacles', ObstacleArrayMsg)
        ts = message_filters.ApproximateTimeSynchronizer([robot_pose_sub, obst_pose_sub], 10,0.05)
        ts.registerCallback(self.pos_CB)
        self.trial = -1
        trial_sub = rospy.Subscriber('trial', Int64, self.trial_CB)
    def trial_CB(self, trial):
        self.trial = trial.data
    def pos_CB(self, robot_pose, obstacles):
        robot_pose = robot_pose.pose.pose.position
        min_dist = 100
        for obstacle in obstacles.obstacles:
            dist = math.sqrt(math.pow(robot_pose.x-obstacle.polygon.points[0].x, 2) + math.pow(robot_pose.y-obstacle.polygon.points[0].y, 2))
            if dist<min_dist:
                min_dist = dist
        if min_dist < 0.35:
            print '\n{:5d}COLLISION: {:05f}'.format(self.trial,min_dist),
            sys.stdout.flush()
        else:
            print '\rmin_dist: {:05f}'.format(min_dist),
            sys.stdout.flush()
if __name__=='__main__':
    try:
        rospy.init_node("sim_summarize", anonymous=True)
        summarizer = Summarizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass