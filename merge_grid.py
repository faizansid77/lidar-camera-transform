#!/usr/bin/env python

import rospy
from sensor_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge, CvBridgeError

resolution = 0.5
map_width = 200
map_height = 200
map_limit = map_width*map_height
map_origin_position ={-200.0, -200.0, 0.0}

pImg=0.5 #image prob
pLid=0.5 #lidar prob

class merge_maps:
    def __init__(self):
        self.bridge=CvBridge()

        self.img_sub=rospy.Subscriber("/image_map",OccupancyGrid,self.callbackImg)
        self.lidar_sub=rospy.Subscriber("/lidar_map",OccupancyGrid,self.callbackLidar)
        self.grid_pub=rospy.Publisher("/lidar_camera_grid",OccupancyGrid,queue_size=1000)
        self.merged_grid=OccupancyGrid()
        self.merged_grid.header.frame_id = "odom"
        self.merged_grid.info.width = map_width
        self.merged_grid.info.height = map_height
        self.merged_grid.info.resolution = resolution
        self.merged_grid.info.origin.position.x = map_origin_position[0]
        self.merged_grid.info.origin.position.y = map_origin_position[1]
        self.merged_grid.data.resize(map_limit)
        self.imageSubDone=False
        self.lidarSubDone=False
        for i in range(map_limit):
            self.merged_grid.data[i] = -1


    def callbackImg(self,data):
        if(not self.lidarSubDone):
            for i in range(map_limit):
                self.merged_grid.data[i]=data[i]
                self.imageSubDone=True
        else:
            for i in range(map_limit):
                self.merged_grid.data[i]=pImg*data[i]+pLid*self.merged_grid.data[i]
            self.grid_pub.publish(self.merged_grid)
            self.imageSubDone=False
            self.lidarSubDone=False
    def callbackLidar(self,data):
        if(not self.imageSubDone):
            for i in range(map_limit):
                self.merged_grid.data[i]=data[i]
                self.lidarSubDone=True
        else:
            for i in range(map_limit):
                self.merged_grid.data[i]=pLid*data[i]+pImg*self.merged_grid.data[i]
            self.grid_pub.publish(self.merged_grid)
            self.imageSubDone=False
            self.lidarSubDone=False



def main():
    merge=merge_maps()
    rospy.init_node('merge_grid',anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")
    