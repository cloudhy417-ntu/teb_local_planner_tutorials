#!/usr/bin/env python3
import numpy as np
import rospy
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
class FlowMap():
    def __init__(self, width, height, resolution):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.map = np.zeros((int(width/resolution), int(height/resolution), 3)) # W*H*(x,y,num of sample)
        self.object_subscriber =  rospy.Subscriber('/move_base/TebLocalPlannerROS/obstacles', ObstacleArrayMsg, self.updateMap)
    def updateMap(self, data):
        for obstacle in data.obstacles:
            x = obstacle.polygon.points[0].x
            y = obstacle.polygon.points[0].y
            vx = obstacle.velocities.twist.linear.x 
            vy = obstacle.velocities.twist.linear.y
            indexX = int(x//self.resolution + (self.width//self.resolution)//2)
            indexY = int(y//self.resolution + (self.height//self.resolution)//2)
            if(indexX<0 or indexX >= int(self.width//self.resolution) or indexY<0 or indexY >= int(self.height//self.resolution)):
                continue
            self.map[indexX,indexY][:2] = (self.map[indexX,indexY][:2] * self.map[indexX,indexY][2] + np.array([vx,vy]))/(self.map[indexX,indexY][2]+1)
            self.map[indexX,indexY][2]+=1  
    def saveMap(self, path = './flowmap.npy'):
        np.save(path, self.map)
if __name__ == '__main__':
    rospy.init_node('flowmap', anonymous=True)
    flowmap = FlowMap(13,9,0.05)
    rospy.on_shutdown(flowmap.saveMap)
    rospy.spin()
        