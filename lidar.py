#!/usr/bin/env python
import rospy
import numpy
import math
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from sensor_msgs.msg import LaserScan

def dist((x1,y1),(x2,y2)):  
    distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)  
    return distance 

def main():
    rospy.init_node('scan_node', anonymous =True)
    scan_pub = rospy.Publisher('/my_scan', LaserScan, queue_size=100)
    rospy.Subscriber("/map", OccupancyGrid , callback) 
    scan_data = LaserScan()
    scan_msg = scan_data
    now = rospy.get_rostime()
    scan_data.header.stamp.secs = now.secs
    scan_data.header.stamp.nsecs = now.nsecs
    scan_data.header.frame_id = 'base_scan'
    scan_data.angle_min = 0
    scan_data.angle_max = 6.26573181152
    scan_data.angle_increment = 0.0174532923847        #(scan_data.angle_max-scan_data.angle_min)/360
    scan_data.time_increment = 2.9888                 
    scan_data.range_min= 0.11999
    scan_data.range_max = 3.5       
    base_scan_pose = [0, 0, 0.0]                     # base scan pose in terms of map-cell 
    base_scan_position = (base_scan_pose[0],base_scan_pose[1]) 
    scan_ranges = []            
    scan_data.ranges = scan_ranges   
    scan_angle = []                
    obs_dist =[]   
    occ_cell = [(20, 20),(20, 21),(20, 22),(20, 23),(20, 24),(21, 20),(21, 21),(21, 22),(21, 23),(21, 24),(22, 20),(22, 21),(22, 22),(22, 23),(22, 24),(23, 20),(23, 21),(23, 22),(23, 23),(23, 24),(24, 20),(24, 21),(24, 22),(24, 23),(24, 24),(25, 20),(25, 21),(25, 22),(25, 23),(25, 24),(26, 20),(26, 21),(26, 22),(26, 23),(26, 24),(27, 20),(27, 21),(27, 22),(27, 23),(27, 24),(28, 20),(28, 21),(28, 22),(28, 23),(28, 24),(29, 20),(29, 21),(29, 22),(29, 23),(29, 24),(30, 20),(30, 21),(30, 22),(30, 23),(30, 24),(31, 20),(31, 21),(31, 22),(31, 23),(31, 24),(32, 20),(32, 21),(32, 22),(32, 23),(32, 24),(33, 20),(33, 21),(33, 22),(33, 23),(33, 24),(34, 20),(34, 21),(34, 22),(34, 23),(34, 24),(35, 20),(35, 21),(35, 22),(35, 23),(35, 24),(36, 20),(36, 21),(36, 22),(36, 23),(36, 24),(37, 20),(37, 21),(37, 22),(37, 23),(37, 24),(38, 20),(38, 21),(38, 22),(38, 23),(38, 24),(39, 20),(39, 21),(39, 22),(39, 23),(39, 24)]
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        for occ in occ_cell:           
            obs_angle = math.atan2(occ[1]-base_scan_position[1], occ[0]-base_scan_position[0])  ##calculate angle between obstacle cell and laser pose
            scan_angle.append(obs_angle)
            my_dist = (dist(occ,base_scan_position)*0.05)                                      ## calculate distance between obst. cell and laser scan in meters
            obs_dist.append(my_dist)
            for i in scan_angle : 
                theta_degree = i*180/math.pi
                #print theta_degree
        max_angle = max(scan_angle)
        min_angle = min(scan_angle)
        for k in numpy.arange(scan_data.angle_min, scan_data.angle_max, scan_data.angle_increment):  
            k_index = int(round((k-scan_data.angle_min)/scan_data.angle_increment))
            if (min_angle <= k <= max_angle):   
                my_range = obs_dist[k_index]
                scan_ranges.append(my_range)
            else :
                my_range = scan_data.range_max
                scan_ranges.append(my_range)
            scan_pub.publish(scan_msg)
        obs_dist =[]
        scan_ranges = [] 
        scan_angle = []
        rate.sleep() 
if __name__=="__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
