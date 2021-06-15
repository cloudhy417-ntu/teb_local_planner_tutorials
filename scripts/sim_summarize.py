#!/usr/bin/env python
import sys
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
        trial_sub = rospy.Subscriber('trial', Int64, self.trial_CB)
        move_base_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.move_base_CB)
        self.listener = tf.TransformListener()
        self.trial = 0
        self.move_base_status = None
        robot_pose_sub = message_filters.Subscriber('/base_pose_ground_truth', Odometry)
        obst_pose_sub = message_filters.Subscriber('/move_base/TebLocalPlannerROS/obstacles', ObstacleArrayMsg)
        ts = message_filters.ApproximateTimeSynchronizer([robot_pose_sub, obst_pose_sub], 10,0.05)
        ts.registerCallback(self.pos_CB)
        self.status_list = OrderedDict()
        self.trans = None
    def move_base_CB(self, status):
        self.move_base_status = status
    def trial_CB(self, trial):
        self.trial = trial.data
        print(trial)
    def pos_CB(self, robot_pose, obstacles):
        try:
            (self.trans,rot) = self.listener.lookupTransform('/base_link', '/map', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        robot_pose = robot_pose.pose.pose.position
        obstacles = [obstacle.polygon.points[0] for obstacle in obstacles.obstacles]
        move_base_status = [status.status for status in self.move_base_status.status_list]
        if self.trial == 0 or self.trans==None:
            return
        if self.trial in self.status_list.keys():
            self.status_list[self.trial] += [(move_base_status, robot_pose, obstacles, self.trans)]
        else:
            self.status_list[self.trial] = [(move_base_status, robot_pose, obstacles, self.trans)]
def summary():
    name = 'eth_right2left4830.pkl'
    while path.exists(name):
        name += '.1'
    with open(name, 'wb') as f:
        pickle.dump(summarizer.status_list, f)
    print("finished")
if __name__=='__main__':
    try:
        global summarizer
        rospy.init_node("sim_summarize", anonymous=True)
        rospy.on_shutdown(summary)
        summarizer = Summarizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass