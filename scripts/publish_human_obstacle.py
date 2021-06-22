#!/usr/bin/env python

# Author: franz.albers@tu-dortmund.de
import sys
import time
import pathos.pools as pp
import rospy, math, tf, rospkg
import yaml
import numpy as np
import pandas as pd
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import Header, ColorRGBA, Int64
from geometry_msgs.msg import PoseWithCovarianceStamped, PolygonStamped, Point32, QuaternionStamped, Quaternion, TwistWithCovariance, Pose, Point, Vector3
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray
from jsk_recognition_msgs.msg import PolygonArray

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty
from std_msgs.msg import Bool
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
def get_state(args):
  dataframe, id, t, time_0, time_1 = args
  x_0 = dataframe.loc[[time_0]].loc[dataframe.loc[[time_0]].id==id].x.values[0]
  x_1 = dataframe.loc[[time_1]].loc[dataframe.loc[[time_1]].id==id].x.values[0]
  x = x_1*(t-time_0)/10 + x_0*(10-(t-time_0))/10
  vx = 0.0 #x_1 - x_0
  y_0 = dataframe.loc[[time_0]].loc[dataframe.loc[[time_0]].id==id].y.values[0]
  y_1 = dataframe.loc[[time_1]].loc[dataframe.loc[[time_1]].id==id].y.values[0]
  y = y_1*(t-time_0)/10 + y_0*(10-(t-time_0))/10
  vy = 0.0 #y_1 - y_0
  return (x,y, vx,vy)
class HumanObstaclePublisher():
  def __init__(self, config):
    self.config = config
    raw_data = read_file(config['path'])
    dataframe = pd.DataFrame(raw_data, columns=['frame', 'id', 'x', 'y'])
    dataframe['frame'] = dataframe['frame'].astype(int)
    self.dataframe = dataframe.set_index('frame')
    self.dataframe.x -=dataframe.x.mean()
    self.dataframe.y -=dataframe.y.mean()
    self.obstacle_publisher = rospy.Publisher('/move_base/TebLocalPlannerROS/obstacles', ObstacleArrayMsg, queue_size=1)
    self.marker_publisher = rospy.Publisher('visualization_marker', MarkerArray, queue_size=100)
    self.local_human_group_publisher = rospy.Publisher('/move_base/local_costmap/humangrouplayer/human_group', PolygonArray, queue_size=1)
    self.global_human_group_publisher = rospy.Publisher('/move_base/global_costmap/humangrouplayer/human_group', PolygonArray, queue_size=1)
    self.finish_sub = rospy.Subscriber('/finished', Bool, self.finish_CB)
    self.goal = False
    self.time = 0
    self.p = pp.ProcessPool(8)
    rospy.wait_for_service('/reset_positions')
    self.stage_reset_pos = rospy.ServiceProxy('/reset_positions', Empty)
    rospy.wait_for_service('/move_base/clear_costmaps')
    self.clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
    self.amcl_reset_pos = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
  def finish_CB(self, move_base_status):
    self.goal = move_base_status.data
  def reset_pos(self):
    self.goal = False
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
    rospy.sleep(1.)
  def publish_obstacle_msg(self, t):
    radius = 0.4
    obstacle_msg = ObstacleArrayMsg() 
    obstacle_msg.header.frame_id = '/map'
    obstacle_msg.header.stamp = rospy.Time.now()
    individual_polygons = PolygonArray()
    individual_polygons.header.frame_id = '/map'
    individual_polygons.header.stamp = rospy.Time.now()
    marker_msg = MarkerArray()
    ppl_tuple = self.get_ppl_loc(t)
    if ppl_tuple is None:
      self.obstacle_publisher.publish(obstacle_msg)
      self.local_human_group_publisher.publish(individual_polygons)
      self.global_human_group_publisher.publish(individual_polygons)
      return
    for id, state in ppl_tuple:
      obst = ObstacleMsg()
      individual_polygon = PolygonStamped()
      individual_polygon.polygon.points = [Point32(x=state[0]+0.2, y=state[1]), Point32(x=state[0], y=state[1]+0.2), 
                                           Point32(x=state[0]-0.2, y=state[1]), Point32(x=state[0], y=state[1]-0.2)]
      individual_polygons.polygons.append(individual_polygon)
      obst.header.frame_id='map'
      obst.id = id
      obst.radius = radius
      obst.polygon.points = [Point32(x=state[0], y=state[1])]
      
      obst.velocities.twist.linear.x = state[2]*2.0
      obst.velocities.twist.linear.y = state[3]*2.0
      
      obstacle_msg.obstacles.append(obst)
      
      marker = Marker(
            type=Marker.SPHERE,
            id=obst.id,
            pose=Pose(Point(obst.polygon.points[0].x, obst.polygon.points[0].y, 0.0), Quaternion(0, 0, 0, 1)),
            scale=Vector3(radius, radius, radius),
            header=Header(frame_id='map'),
            color=ColorRGBA(0.0, 1.0, 0.0, 1.0))
      marker.lifetime = rospy.Duration.from_sec(1.1)
      marker_msg.markers.append(marker)
    self.marker_publisher.publish(marker_msg)
    self.obstacle_publisher.publish(obstacle_msg)
    self.local_human_group_publisher.publish(individual_polygons)
    self.global_human_group_publisher.publish(individual_polygons)
        
  def get_ppl_loc(self,t):
    interval = 10
    time_0 = t//10 * 10
    time_1 = (t//10 + 1) * 10
    if not(time_0 in  self.dataframe.index) or not(time_1 in self.dataframe.index):
      return None 
    ppl_set_0 = set(self.dataframe.loc[[time_0]].id.values)
    ppl_set_1 = set(self.dataframe.loc[[time_1]].id.values)
    ppl_set_intersection = list(ppl_set_0.intersection(ppl_set_1))
    return_dict = {}
    return_list = self.p.map(get_state, [(self.dataframe, id, t, time_0, time_1) for id in ppl_set_intersection])
    return_tuple = zip(ppl_set_intersection, return_list)
    return return_tuple
if __name__ == '__main__': 
  try:
    rospy.init_node("test_obstacle_msg", anonymous=True)
    rospack = rospkg.RosPack()
    config_path = rospack.get_path('teb_local_planner_tutorials')+'/cfg/trial_cfg.yaml'
    with open(config_path, 'r') as f:
      config = yaml.load(f, Loader=yaml.FullLoader)
    human_obstacle_publisher = HumanObstaclePublisher(config)
    sim_time_publisher = rospy.Publisher('/sim_time', Int64, queue_size=10)
    trial_publisher = rospy.Publisher('/trial', Int64, queue_size=10)
    r = rospy.Rate(10)
    t0 = config['start_time']-30
    t = t0
    while not rospy.is_shutdown():
      sim_time_publisher.publish(t)  
      human_obstacle_publisher.publish_obstacle_msg(t)
      print '\rTimeStep: {:05d}'.format(t),
      sys.stdout.flush()
      t += 1
      if t > t0 + int(config['trial_time']*10) or human_obstacle_publisher.goal:
        human_obstacle_publisher.reset_pos()
        t0 += 30
        t = t0
        trial_publisher.publish(t0)
      r.sleep()
      if t>config['end_time']:
        rospy.signal_shutdown('finished playback')
  except rospy.ROSInterruptException:
    print("finished playback")
    pass

