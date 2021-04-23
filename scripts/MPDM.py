import rospy
import math
import numpy as np
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
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
        self.robot_pose_sub = rospy.Subscriber('/robot_pose', Pose, self.update_pose)
        self.robot_pose = Pose()
        self.plan_visualizer = rospy.Publisher('/plan', Path, queue_size=1)
    def update_pose(self, pose):
        self.robot_pose = pose
    def make_plan(self, goal):
        tolerance = 0.1
        start = PoseStamped()
        start.header.frame_id = 'map'
        start.pose = self.robot_pose
        path = self.planner.call(start,goal,tolerance)
        path_score = self.evaluate_path(path.plan)
        print(path_score)
        self.plan_visualizer.publish(path.plan)
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
    flowmap = np.load('flowmap_direction_down_sampled.npy')
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