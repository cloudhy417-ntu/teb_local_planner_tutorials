#!/usr/bin/env python3
import rospy
import math
import numpy as np
from sklearn.cluster import DBSCAN
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Path
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Vector3, Point32, PolygonStamped
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from jsk_recognition_msgs.msg import PolygonArray
import sys
print (sys.version)
sys.path.insert(0, '/home/cloudhy/python3_ws/devel/lib/python3/dist-packages')
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty

PI = math.pi
map_dict = {1:(0,-PI), 2:(PI/4, -3*PI/4), 3:(PI/2, -PI/2), 4:(3*PI/4, -PI/4)}
class MPDM:
    def __init__(self, flowmap, resolution, centerX, centerY):
        self.flowmap = flowmap
        self.resolution = resolution
        self.centerX = centerX
        self.centerY = centerY
        rospy.wait_for_service('/move_base/GlobalPlanner/make_plan')
        self.planner = rospy.ServiceProxy('/move_base/GlobalPlanner/make_plan', GetPlan)

        self.move_base_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.move_base_client.wait_for_server()
        self.goal_pub = rospy.Publisher('goal', PoseStamped, queue_size=100)
        
        self.human_sub = rospy.Subscriber('/move_base/TebLocalPlannerROS/obstacles', ObstacleArrayMsg, self.humanObstacleCallback)
        self.cluster_visualizer = rospy.Publisher('/group_cluster', MarkerArray, queue_size = 1)
        self.human_group_publisher = rospy.Publisher('/move_base/local_costmap/humangrouplayer/human_group', PolygonArray, queue_size=1)
        self.humans = ObstacleArrayMsg()
        self.robot_pose_sub = rospy.Subscriber('/robot_pose', Pose, self.update_pose)
        self.robot_pose = Pose()
        self.plan_visualizer = rospy.Publisher('/plan', Path, queue_size=1)
    def humanObstacleCallback(self, data):
        self.humans = data.obstacles
        markerArray = MarkerArray()
        humanIDs = []
        humanArr = []
        radius = 0.6
        for human in self.humans:
            x = human.polygon.points[0].x
            y = human.polygon.points[0].y
            velx = human.velocities.twist.linear.x
            vely = human.velocities.twist.linear.y
            vel = math.sqrt(math.pow(velx,2) + math.pow(vely,2))
            if vel != 0:
                humanArr.append([x,y,velx/vel, vely/vel])
            else:
                humanArr.append([x,y,0, 0])
            humanIDs.append(human.id)
        humanArr = np.array(humanArr)
        clustering=DBSCAN(eps=1.2,min_samples=2).fit(humanArr)
        group_id = 0
        group_polygons = PolygonArray()
        group_polygons.header.frame_id = '/map'
        while group_id in clustering.labels_:
            group_polygon = PolygonStamped()
            group_polygon.header.frame_id = '/map'
            indexes = np.where(clustering.labels_ == group_id)
            group_positions = humanArr[indexes]
            position = np.average(group_positions, axis=0)
            if(group_positions.shape[0]==2):
                group_positions = np.concatenate((group_positions, np.expand_dims(position, 0)), axis=0)
            Xrange = group_positions[:,0].max() - group_positions[:,0].min() + 1
            Yrange = group_positions[:,1].max() - group_positions[:,1].min() + 1
            marker = Marker(
                    type=Marker.SPHERE,
                    id=group_id,
                    pose=Pose(Point(position[0], position[1], 0.0), Quaternion(0, 0, 0, 1)),
                    scale=Vector3(Xrange, Yrange, 0.1),
                    header=Header(frame_id='map'),
                    color=ColorRGBA(1.0, 0.0, 0.0, 0.3))
            marker.lifetime = rospy.Duration.from_sec(0.5)
            group_id += 1
            markerArray.markers.append(marker)
            for position in group_positions:
                group_polygon.polygon.points.append(Point32(position[0], position[1], 0))
            group_polygons.polygons.append(group_polygon)    
        self.human_group_publisher.publish(group_polygons)
        self.cluster_visualizer.publish(markerArray)
    def update_pose(self, pose):
        self.robot_pose = pose
    def make_plan(self, goal):
        # make a virtual plan without consider dynamic obstacle
        # TODO: clear dynamic obstacle
        tolerance = 0.1
        start = PoseStamped()
        start.header.frame_id = 'map'
        start.pose = self.robot_pose
        # global plan scoring
        path = self.planner.call(start,goal,tolerance)
        path_score = self.evaluate_path(path.plan)
        # visualize for debugginh
        # print(path_score)
        self.plan_visualizer.publish(path.plan)
        # group clustering

    def score(self, map_dir, path_dir):
        map_dir_angle = map_dict[map_dir]
        return max(math.cos(path_dir-map_dir_angle[0]), math.cos(path_dir-map_dir_angle[1]))
    def evaluate_path(self, path):
        total_score = 0
        for i in range(len(path.poses)-1):
            indexX = int(path.poses[i+1].pose.position.x/self.resolution + self.centerX)
            indexY = int(path.poses[i+1].pose.position.y/self.resolution + self.centerY)
            map_dir = self.flowmap[indexX, indexY]
            dx = path.poses[i+1].pose.position.x - path.poses[i].pose.position.x
            dy = path.poses[i+1].pose.position.y - path.poses[i].pose.position.y
            path_dir = math.atan2(dy, dx)
            if map_dir != 0:
                total_score += self.score(map_dir, path_dir)
            else:
                total_score += 1
        return total_score/(len(path.poses)-1)
if __name__=='__main__':
    rospy.init_node('mpdm', anonymous=True)
    flowmap = np.load('/home/cloudhy/catkin_ws/src/teb_local_planner_tutorials/scripts/flowmap_direction_down_sampled.npy')
    resolution = 0.2
    centX = 32.5
    centY = 22.5
    mpdm = MPDM(flowmap,resolution,centX,centY)
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    quaternion = quaternion_from_euler(0, 0, 0.5)
    goal.pose = Pose(Point(1,2,0), Quaternion(0,0,quaternion[2],quaternion[3]))
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        mpdm.make_plan(goal)
        r.sleep()
    rospy.spin()
    a = PoseStamped()
    a.pose[0].position