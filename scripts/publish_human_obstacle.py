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
    rospy.wait_for_service('/reset_positions')
    self.stage_reset_pos = rospy.ServiceProxy('/reset_positions', Empty)
    self.amcl_reset_pos = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
  def reset_pos(self):
    self.stage_reset_pos.call()
    initial_pose = PoseWithCovarianceStamped()
    initial_pose.header.frame_id = '/map'
    initial_pose.pose.pose.position.x = -0
    initial_pose.pose.pose.position.y = 3
    q = quaternion_from_euler(0,0,-math.pi/2)
    initial_pose.pose.pose.orientation.z = q[2]
    initial_pose.pose.pose.orientation.w = q[3]
    self.amcl_reset_pos.publish(initial_pose)
    rospy.sleep(1.)
  def publish_obstacle_msg(self, t):
    radius = 0.4
    obstacle_msg = ObstacleArrayMsg() 
    obstacle_msg.header.frame_id = '/map'
    ppl_dict = get_ppl_loc(self.dataframe, t)
    if ppl_dict is None:
      self.obstacle_publisher.publish(obstacle_msg)
      return
    for id in ppl_dict.keys():
      obst = ObstacleMsg()
      obst.header.frame_id='map'
      obst.id = id
      obst.radius = radius
      obst.polygon.points = [Point32(x=ppl_dict[id][0], y=ppl_dict[id][1])]
      
      obst.velocities.twist.linear.x = ppl_dict[id][2]*2.0
      obst.velocities.twist.linear.y = ppl_dict[id][3]*2.0
      
      obstacle_msg.obstacles.append(obst)
    for obstacle in obstacle_msg.obstacles:
      marker = Marker(
            type=Marker.SPHERE,
            id=obstacle.id,
            pose=Pose(Point(obstacle.polygon.points[0].x, obstacle.polygon.points[0].y, 0.0), Quaternion(0, 0, 0, 1)),
            scale=Vector3(radius, radius, radius),
            header=Header(frame_id='map'),
            color=ColorRGBA(0.0, 1.0, 0.0, 1.0))
      marker.lifetime = rospy.Duration.from_sec(1.1)
      self.marker_publisher.publish(marker)
    self.obstacle_publisher.publish(obstacle_msg)

if __name__ == '__main__': 
  try:
    rospy.init_node("test_obstacle_msg", anonymous=True)
    path = '/home/cloudhy/programs/sgan/datasets/eth/test/biwi_eth.txt'
    human_obstacle_publisher = HumanObstaclePublisher(path)
    sim_time_publisher = rospy.Publisher('/sim_time', Int64, queue_size=10)
    trial_publisher = rospy.Publisher('/trial', Int64, queue_size=10)
    r = rospy.Rate(10)
    t0 = 750
    t = t0
    while not rospy.is_shutdown():
      sim_time_publisher.publish(t)  
      human_obstacle_publisher.publish_obstacle_msg(t)
      print '\rTimeStep: {:05d}'.format(t),
      sys.stdout.flush()
      t += 1
      if t > t0 + 150:
        human_obstacle_publisher.reset_pos()
        t0 += 30
        t = t0
        trial_publisher.publish(t0)
      r.sleep()
      if t>12580:
        rospy.signal_shutdown('finished playback')
  except rospy.ROSInterruptException:
    print("finished playback")
    pass
# demo scenerio 4236

