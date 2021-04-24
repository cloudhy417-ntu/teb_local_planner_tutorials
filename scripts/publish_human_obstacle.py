#!/usr/bin/env python

# Author: franz.albers@tu-dortmund.de
import sys
import rospy, math, tf
import numpy as np
import pandas as pd
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import PolygonStamped, Point32, QuaternionStamped, Quaternion, TwistWithCovariance, Pose, Point, Vector3
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty

def read_file(_path, delim='\t'):
    data = []
    if delim == 'tab':
        delim = '\t'
    elif delim == 'space':
        delim = ' '
    with open(_path, 'r') as f:
        for line in f:
            line = line.strip().split(delim)
            line = [float(i) for i in line]
            data.append(line)
    return np.asarray(data)
def get_ppl_loc(dataframe,time):
    interval = 10
    time_0 = time//10 * 10
    time_1 = (time//10 + 1) * 10
    if not(time_0 in  dataframe.index) or not(time_1 in dataframe.index):
      return None 
    ppl_set_0 = set(dataframe.loc[[time_0]].id.values)
    ppl_set_1 = set(dataframe.loc[[time_1]].id.values)
    ppl_set_intersection = list(ppl_set_0.intersection(ppl_set_1))
    return_dict = {}
    for id in ppl_set_intersection:
        x_0 = dataframe.loc[[time_0]].loc[dataframe.loc[[time_0]].id==id].x.values[0]
        x_1 = dataframe.loc[[time_1]].loc[dataframe.loc[[time_1]].id==id].x.values[0]
        x = x_1*(time-time_0)/10 + x_0*(10-(time-time_0))/10
        vx = x_1 - x_0
        y_0 = dataframe.loc[[time_0]].loc[dataframe.loc[[time_0]].id==id].y.values[0]
        y_1 = dataframe.loc[[time_1]].loc[dataframe.loc[[time_1]].id==id].y.values[0]
        y = y_1*(time-time_0)/10 + y_0*(10-(time-time_0))/10
        vy = y_1 - y_0
        return_dict[id] = (x,y, vx,vy)
    return return_dict
class HumanObstaclePublisher():
  def __init__(self, path):
    raw_data = read_file(path)
    dataframe = pd.DataFrame(raw_data, columns=['frame', 'id', 'x', 'y'])
    dataframe['frame'] = dataframe['frame'].astype(int)
    self.dataframe = dataframe.set_index('frame')
    self.dataframe.x -=dataframe.x.mean()
    self.dataframe.y -=dataframe.y.mean()
    self.obstacle_publisher = rospy.Publisher('/move_base/TebLocalPlannerROS/obstacles', ObstacleArrayMsg, queue_size=1)
    self.marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=100)
    self.time = 780
  def publish_obstacle_msg(self, t):
    obstacle_msg = ObstacleArrayMsg() 
    obstacle_msg.header.frame_id = '/map'
    ppl_dict = get_ppl_loc(self.dataframe, t)
    if ppl_dict is None:
      return
    for id in ppl_dict.keys():
      obst = ObstacleMsg()
      obst.header.frame_id='map'
      obst.id = id
      obst.radius = 0.6
      obst.polygon.points = [Point32(x=ppl_dict[id][0], y=ppl_dict[id][1])]
      
      obst.velocities.twist.linear.x = ppl_dict[id][2]*1.0
      obst.velocities.twist.linear.y = ppl_dict[id][3]*1.0
      
      obstacle_msg.obstacles.append(obst)
    for obstacle in obstacle_msg.obstacles:
      marker = Marker(
            type=Marker.SPHERE,
            id=obstacle.id,
            pose=Pose(Point(obstacle.polygon.points[0].x, obstacle.polygon.points[0].y, 0.0), Quaternion(0, 0, 0, 1)),
            scale=Vector3(0.6, 0.6, 0.6),
            header=Header(frame_id='map'),
            color=ColorRGBA(0.0, 1.0, 0.0, 0.8))
      marker.lifetime = rospy.Duration.from_sec(1.1)
      self.marker_publisher.publish(marker)
    self.obstacle_publisher.publish(obstacle_msg)

if __name__ == '__main__': 
  try:
    rospy.init_node("test_obstacle_msg")
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()
    path = '/home/cloudhy/programs/sgan/datasets/eth/test/biwi_eth.txt'
    human_obstacle_publisher = HumanObstaclePublisher(path)
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 0
    goal.target_pose.pose.position.y = 3
    orientation = quaternion_from_euler(0,0,1.57)
    goal.target_pose.pose.orientation.z = orientation[2]
    goal.target_pose.pose.orientation.w = orientation[3]
    # client.send_goal(goal)
    r = rospy.Rate(10)
    t = 4228
    # t = 780
    while not rospy.is_shutdown():
      human_obstacle_publisher.publish_obstacle_msg(t)
      print '\rTimeStep: {:05d}'.format(t),
      sys.stdout.flush()
      t += 1
      if t==4236:
        client.send_goal(goal)
      if t>4400:
        human_obstacle_publisher.publish_obstacle_msg(15000)
        goal.target_pose.pose.position.x = -2
        goal.target_pose.pose.position.y = -3
        t = 4236
        client.send_goal(goal)
        client.wait_for_result()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = 0
        goal.target_pose.pose.position.y = 3
        client.send_goal(goal)
      r.sleep()
  except rospy.ROSInterruptException:
    print("finished playback")
    pass

