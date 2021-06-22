#!/usr/bin/env python

# Author: franz.albers@tu-dortmund.de
import sys
import rospy, math, tf
import rospkg, yaml
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
from std_msgs.msg import Bool
class Robot():
    def __init__(self):
        rospack = rospkg.RosPack()
        config_path = rospack.get_path('teb_local_planner_tutorials')+'/cfg/trial_cfg.yaml'
        with open(config_path, 'r') as f:
            self.config = yaml.load(f, Loader=yaml.FullLoader)
        self.trial = -1
        self.start_time = self.config['start_time']
        rospy.wait_for_service('/reset_positions')
        self.stage_reset_pos = rospy.ServiceProxy('/reset_positions', Empty)
        self.amcl_reset_pos = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        self.finish_pub = rospy.Publisher('finished', Bool, queue_size=10)
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.sim_time_sub = rospy.Subscriber('trial', Int64, self.trialCB)
        self.client.wait_for_server()
        rospy.wait_for_service('/move_base/clear_costmaps')
        self.clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
    def trialCB(self, trial):
        self.trial = trial.data
        if self.trial == self.start_time:
            self.start_time += 30
            self.send_goal(self.config['goal']['x'],self.config['goal']['y'],self.config['goal']['theta'])

    def reset_pos(self):
        self.stage_reset_pos.call()
        self.clear_costmaps.call()
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = '/map'
        initial_pose.pose.pose.position.x = self.config['start']['x']
        initial_pose.pose.pose.position.y = self.config['start']['y']
        q = quaternion_from_euler(0,0,self.config['start']['theta'])
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
        self.client.wait_for_result(timeout = rospy.Duration(self.config['trial_time']))
        result = self.client.get_result()
        t1 = rospy.Time.now()
        if result:
            self.finish_pub.publish(Bool(True))
            print('{} travelling time:{}.{}'.format(self.start_time-30, (t1-t0).secs, (t1-t0).nsecs))
        else:
            print('{} timed out'.format(self.start_time-30))
        self.reset_pos()
if __name__ == '__main__': 
    try:
        rospy.init_node("send_goal", anonymous=True)
        robot = Robot()
        rospy.spin()
        # while not rospy.is_shutdown()
        #     robot.send_goal(0,3,math.pi/2)
    except rospy.ROSInterruptException:
        print("finished playback")
        pass
# demo scenerio 4236

