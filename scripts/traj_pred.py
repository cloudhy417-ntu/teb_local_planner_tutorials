#!/usr/bin/env python3
import sys
sys.path.append('/home/cloudhy/programs/sgan')
import argparse
import os
import torch
from collections import deque
import numpy as np

from attrdict import AttrDict

from sgan.data.loader import data_loader
from sgan.models import TrajectoryGenerator
from sgan.losses import displacement_error, final_displacement_error
from sgan.utils import relative_to_abs, get_dset_path

print (sys.version)
sys.path.insert(0,'/home/cloudhy/python3_ws/devel/lib/python3/dist-packages')
import rospy, math, tf
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from std_msgs.msg import Header
from nav_msgs.msg import Path
from geometry_msgs.msg import PolygonStamped, Point32, QuaternionStamped, Quaternion, TwistWithCovariance, Point, Quaternion, Pose, PoseStamped
from tf.transformations import quaternion_from_euler, quaternion_matrix
import rospy
import rospkg
from std_msgs.msg import String
# from zed_interfaces.msg import ObjectsStamped, Object
FIX_FRAME = 'map'
def get_generator(checkpoint):
    args = AttrDict(checkpoint['args'])
    generator = TrajectoryGenerator(
        obs_len=args.obs_len,
        pred_len=10,#args.pred_len
        embedding_dim=args.embedding_dim,
        encoder_h_dim=args.encoder_h_dim_g,
        decoder_h_dim=args.decoder_h_dim_g,
        mlp_dim=args.mlp_dim,
        num_layers=args.num_layers,
        noise_dim=args.noise_dim,
        noise_type=args.noise_type,
        noise_mix_type=args.noise_mix_type,
        pooling_type=args.pooling_type,
        pool_every_timestep=args.pool_every_timestep,
        dropout=args.dropout,
        bottleneck_dim=args.bottleneck_dim,
        neighborhood_size=args.neighborhood_size,
        grid_size=args.grid_size,
        batch_norm=args.batch_norm)
    generator.load_state_dict(checkpoint['g_state'])
    generator.cuda()
    generator.eval()
    return generator

class Traj_pred:
    def __init__(self, model_path, max_obj = 1000, max_obs_len = 1000):
        self.prediction_pub = rospy.Publisher('/move_base/TebLocalPlannerROS/humanObstacles', ObstacleArrayMsg, queue_size=10)
        self.history_visualize_pub = rospy.Publisher('previous_path', Path, queue_size=100)
        self.prediction_visualize_pub = rospy.Publisher('predicted_path', Path, queue_size=100)
        self.tf_listener = tf.TransformListener()
        self.history = deque(maxlen=max_obs_len)
        print('load model from '+model_path)
        checkpoint = torch.load(model_path)
        self.generator = get_generator(checkpoint)
        self.trans_matrix = None
        self.lastUpdate = rospy.get_time()-1.0
        print('finished initialize')
    def obs_traj_callback(self, data):
        if len(data.obstacles)==0:
            return
        if (rospy.get_time()-self.lastUpdate < 0.4):
            return
        # store people's position by dict, using label_id as key
        objs = {obj.id:np.array([obj.polygon.points[0].x, obj.polygon.points[0].y])  for obj in data.obstacles}
        # store recorded time
        # objs['time'] = data.header.stamp
        # append the dict into history
        self.history.append(objs)
        # get previous observed position of the current people set
        obs_traj = []
        obs_time = []
        obj_set = list(objs.keys())
        for i in range(len(self.history)-8,len(self.history)):# 10: previous trajectory time steps
            try:
                obs = [self.history[i][key] for key in obj_set] #observed point for one time step
                obs_traj.append(obs)
                obs_time.append(obs['time'])
            except:
                pass
        obs_traj = np.array(obs_traj, dtype=float) # shape:(timestep, num_of_ppl, xy)
        # visualize previous trajectory
        human_previous_path_deque = deque(maxlen=1000)
        for i in range(0, obs_traj.shape[1]):# iterate through ppl
            human_previous_path = Path()
            human_previous_path.header.seq = i
            human_previous_path.header.stamp = rospy.Time.now()
            human_previous_path.header.frame_id = FIX_FRAME
            traj = [PoseStamped(Header(),Pose(Point(loc[0],loc[1],0), Quaternion())) for loc in obs_traj[:,i,:]]# single person trajectory
            human_previous_path.poses = traj
            human_previous_path_deque.append(human_previous_path)
        for path in human_previous_path_deque:
            self.history_visualize_pub.publish(path)
        if obs_traj.shape[0]!=8:
            return
        obs_traj *= 10
        # prepare data in social gan data format (for inference)
        obs_traj_rel = np.diff(obs_traj, axis=0)
        obs_traj_rel = np.insert(obs_traj_rel, 0, [0,0], axis=0)

        obs_traj = torch.from_numpy(obs_traj).float().cuda()
        obs_traj_rel = torch.from_numpy(obs_traj_rel).float().cuda()
        
        num_of_ppl = obs_traj.shape[1]
        start_end = torch.tensor([[0,num_of_ppl]]).cuda()
        
        # do the inference
        result = self.generator(obs_traj, obs_traj_rel, start_end)
        pred_traj_fake = relative_to_abs(result, obs_traj[-1])# shape:[10, num of ppl, 2(x,y)]
        pred_traj_fake/=10
        start = pred_traj_fake[:1,:,:].clone()
        pred_traj_fake -= start
        pred_traj_fake *= 2.5
        pred_traj_fake += start
        # make predicted data in ROS format to publish to local planner and visualize
        human_obstacle_msg = ObstacleArrayMsg() 
        human_obstacle_msg.header.stamp = rospy.Time.now()
        human_obstacle_msg.header.frame_id = FIX_FRAME # CHANGE HERE: odom/map
        human_predicted_path_deque = deque(maxlen=20)
        for i in range(num_of_ppl):
            # for visualize
            human_predicted_path = Path()
            human_previous_path.header.seq = i
            human_predicted_path.header.stamp = rospy.Time.now()
            human_predicted_path.header.frame_id = FIX_FRAME
            traj = [PoseStamped(Header(),Pose(Point(loc[0],loc[1],0), Quaternion())) for loc in pred_traj_fake[:,i,:]]
            human_predicted_path.poses = traj
            human_predicted_path_deque.append(human_predicted_path)
            # for local planner
            human_obstacle = ObstacleMsg()
            human_obstacle.header.stamp = rospy.Time.now()
            human_obstacle.header.frame_id = FIX_FRAME
            for j in range(pred_traj_fake.shape[0]):
                human_obstacle = ObstacleMsg()
                human_obstacle.id = obj_set[i] # To indicate the same person (id from zed SDK)
                human_obstacle.polygon.points = [Point32()]
                human_obstacle.polygon.points[0].x = pred_traj_fake[j,i,0]
                human_obstacle.polygon.points[0].y = pred_traj_fake[j,i,1]
                human_obstacle.polygon.points[0].z = 1/15 #delta T
                human_obstacle_msg.obstacles.append(human_obstacle)
        self.prediction_pub.publish(human_obstacle_msg)
        for path in human_predicted_path_deque:
            self.prediction_visualize_pub.publish(path)
    
def start_pred():
    rospy.init_node('zed_traj_pred', anonymous=True)
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('teb_local_planner_tutorials')
    model_path = os.path.join(package_path,'models/sgan-models/eth_8_model.pt')
    traj_pred = Traj_pred(model_path=model_path)
    rospy.Subscriber('/move_base/TebLocalPlannerROS/obstacles', ObstacleArrayMsg, traj_pred.obs_traj_callback)
    rospy.spin()

if __name__ == '__main__':
    start_pred()