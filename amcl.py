#!/usr/bin/env python
import rospy
import numpy
import math
import time
from nav_msgs.msg import OccupancyGrid ,MapMetaData
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped 

def dist_sq(x,y):                                   ## square of distance between two co-odrinates
    dist = (x**2 +y**2)  
    return dist

def MAP_GTW(hit,map_x):                              ## world_coordinates to map_grid co-ordinates 
    global map_resolution
    mx = (hit-map_x)/map_resolution          
    return mx

def MAP_INDEX(mi,mj):                               ## map_index from map_grid co-ordinates
    global map_width
    my_index = mi+mj*map_width           
    return my_index        

def INDEX_CELL(my_index):                            ## map_grid co-ordinates from map_index
    global map_width
    my = my_index/map_width            
    mx = my_index -(my*map_width)
    return (mx,my)

def scan_callback(data):                                             
    global scan_ranges, min_angle, max_angle, increment, max_range
    scan_ranges = data.ranges
    min_angle = data.angle_min
    max_angle = data.angle_max 
    increment = data.angle_increment  
    max_range = data.range_max

def map_callback(data):
    global map_occupancy,map_width, map_height, map_resolution,map_origin_x, map_origin_y
    map_occupancy = data.data
    map_width = data.info.width
    map_height = data.info.height
    map_resolution = data.info.resolution
    map_origin_x = data.info.origin.position.x 
    map_origin_y = data.info.origin.position.y 

def pose_callback(data):
    global base_scan_position, base_scan_orientation
    base_scan_position = [data.pose.pose.position.x + math.cos(2*math.asin(data.pose.pose.orientation.z))*(-0.128) ,data.pose.pose.position.y+ math.sin(2*math.asin(data.pose.pose.orientation.z))*(-0.128)]
    base_scan_orientation = math.atan2(base_scan_position[1],base_scan_position[0])
    
def main():
    global  max_range, scan_ranges, min_angle, max_angle,increment, map_origin_x, map_origin_y, map_width, map_height, map_resolution, map_occupancy,base_scan_position, base_scan_orientation
    rospy.init_node('amcl_node', anonymous =True)
    rospy.Subscriber("/my_scan", LaserScan, scan_callback) 
    rospy.Subscriber('/map', OccupancyGrid, map_callback)
    rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, pose_callback)
    scan_ranges = list()
    min_angle = float()
    max_angle = float() 
    increment = float() 
    max_range = float()
    map_origin_x = float()
    map_origin_y = float()
    map_resolution = float()
    map_width = int()
    map_height = int()
    map_occupancy = list()
    base_scan_position = float()
    base_scan_orientation = float()
    rate = rospy.Rate(1)
    time.sleep(1)
    max_occ_dist = max_range
    s = int(max_occ_dist/map_resolution)
    occ_dist_lookup = [] 
    ## set all cell to max_occ_dist
    occ_dist_lookup = [max_occ_dist for Y in xrange(map_height) for X in xrange(map_width)]    

    ## set all occ_dist zero to occupied cells       
    for Y in xrange(map_height): 
        for X in xrange(map_width): 
            Index = X+Y*map_width                                #MAP_INDEX(X,Y)
            if map_occupancy[Index] == 100:
                occ_dist_lookup[Index] = 0
                
    ## calculate occ_dist in Neighbourhood
                for NY in xrange(-s,+s,1): 
                    for NX in xrange(-s,+s,1):
                        if (0<= (X+NX) < map_width) and (0<= (Y+NY) < map_height):
                            N_Index = (X+NX) + (Y+NY)*map_width                         #MAP_INDEX(X+NX,Y+NY)
                            #D = dist_sq(NX,NY)
                            D = NX**2+NY**2   
                            if (D < (occ_dist_lookup[N_Index]/map_resolution)**2): 
                                occ_dist_lookup[N_Index] = math.sqrt(D)*map_resolution    

    rospy.loginfo('amcl_node started')                       
    while not rospy.is_shutdown():                    
        z_sum = 0  
        for theta in (range(len(scan_ranges))):
            theta_k = math.radians(theta)
            z_k = scan_ranges[theta]
            if z_k > 0 :
                hit_x = base_scan_position[0] + z_k*math.cos(base_scan_orientation+theta_k)             ## coordinates of hit into map
                hit_y = base_scan_position[1] + z_k*math.sin(base_scan_orientation+theta_k)
                m_x = MAP_GTW(hit_x, map_origin_x)                                                      ## convert into map_grid_coordinates
                m_y = MAP_GTW(hit_y,map_origin_y)
                I = int(MAP_INDEX(m_x,m_y))                                      ## map_index as per formula            
                occ_dist = occ_dist_lookup[I]                             ## occupied_dist of hit_cell
                z_sum = z_sum + occ_dist                                  ## sum of all occupied_dist
                g.write('score :{}'.format(z_sum))    
                g.write('\n')         
                #print z_sum , occ_dist, I, theta   
        rate.sleep()           

if __name__=="__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
