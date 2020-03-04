#!/usr/bin/env python
import rospy
import math
import numpy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped

def dist((x1,y1),(x2,y2)):                                    #calculate distance between two co-ordinates
    distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)  
    return distance
 
def first(n): 
    return n[1]

def sort(tuples): 
    return sorted(tuples, key = first) 

def main():
    rospy.init_node('map_scan_node', anonymous =True)
    map_pub = rospy.Publisher('/my_map', OccupancyGrid, queue_size=100)
    meta_pub = rospy.Publisher('/map_metadata', MapMetaData, queue_size=100)
    scan_pub = rospy.Publisher('/my_scan', LaserScan, queue_size=100)
    pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=100)
    map_data = OccupancyGrid()
    msg = map_data
    scan_data = LaserScan()
    scan_msg = scan_data
    pose_data = PoseWithCovarianceStamped()
    meta_data = MapMetaData()
## header_msg
    header_data = map_data.header
    header_data.frame_id = 'map'
## map_metadata parameters
    info_data = map_data.info
    info_data.resolution= 0.01
    info_data.width = 250
    info_data.height = 500
    info_data.origin.position.x = 0                     ##map_origin
    info_data.origin.position.y = 0
    info_data.origin.position.z = 0
    info_data.origin.orientation.x = 0 
    info_data.origin.orientation.y = 0 
    info_data.origin.orientation.z = 0
    info_data.origin.orientation.w = 1 
    meta_data = info_data
    map_size= [info_data.height, info_data.width]
    resolution = info_data.resolution
## scan parameters
    scan_data.header.frame_id = 'base_scan'
    scan_data.angle_min = 0.000
    scan_data.angle_max = 6.26573181152
    scan_data.angle_increment = 0.0174532923847      
    scan_data.time_increment = 0.00000298899994959
    scan_data.range_min= 0.11999
    scan_data.range_max = 3.5       

## initial_pose
    theta = 0.0
    actual_yaw = math.radians(theta)
    pose_data.header.frame_id = 'map'
    pose_data.pose.pose.position.x = 1.5   ## add initial pose 
    pose_data.pose.pose.position.y = 1.5
    pose_data.pose.pose.position.z = 0.0
    pose_data.pose.pose.orientation.x = 0.0 
    pose_data.pose.pose.orientation.y = 0.0 
    pose_data.pose.pose.orientation.z = math.sin(actual_yaw/2)
    pose_data.pose.pose.orientation.w = math.cos(actual_yaw/2)                            
    pose_data.pose.covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0., 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    error = [-0.2,-0.2,0.0]            ## add random error
    yaw = actual_yaw + error[2]
    t_W_R = [pose_data.pose.pose.position.x +error[0], pose_data.pose.pose.position.y+ error[1]]       ## robot_pose according to intial pose
    T_WR_S = [math.cos(yaw)*(-0.128) , math.sin(yaw)*(-0.128)]                       ## robot_frame rotation w.r.t world_frame 
    occ_data = []
    map_data.data = occ_data
    scan_ranges = []            
    scan_data.ranges = scan_ranges   
    obs_dist = []
    occ_cell =[]
    new_list= []
    phi_list = []       
    new_phi_list=[]
    new_d_list =[]
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        for i in range(map_size[0]):       
            for j in range(map_size[1]):              ## add occupancy value as required
                if j in range(60,85):                 ## assign occupancy value to map                         
                    if i in range(80,120) :              
                        occupancy = 100  
                        cell = (j,i)
                        occ_cell.append(cell)
                        occ_data.append(occupancy) 
                    else :
                        occupancy = 0
                        occ_data.append(occupancy)      
                else :
                    occupancy = 0
                    occ_data.append(occupancy)
        now = rospy.get_rostime()
        info_data.map_load_time = now
        header_data.stamp.secs = now.secs
        header_data.stamp.nsecs = now.nsecs
        meta_pub.publish(meta_data)
        map_pub.publish(msg)
## calculating obstacle co-ord w.r.t base_scan_pose  
        for occ in occ_cell:
            t_W_P = [occ[0]*resolution , occ[1]*resolution]                                            ## obstacle co-ord in  world_frame   
            t_WS_P = [t_W_P[0]- t_W_R[0]-T_WR_S[0] , t_W_P[1]- t_W_R[1]-T_WR_S[1] ]                    ## obstacle co-ord in scan_frame 
            t_S_P = [math.cos(yaw)*t_WS_P[0]+ math.sin(yaw)*t_WS_P[1] , -(math.sin(yaw))*t_WS_P[0]+ math.cos(yaw)*t_WS_P[1]]      
            phi = int(round(math.degrees(math.atan2(t_S_P[1],t_S_P[0]))))                         ## angle between base_scan and obstacle
            hit_dist = dist((t_S_P[0],t_S_P[1]),(0.0,0.0))  
            if phi < 0 :
                phi = phi + 360                                       ## distance between base_scan and obstacle  
            hit_tuple = (phi, hit_dist)                               ## list distance to obs_dist
            obs_dist.append(hit_tuple)
            sort_obs_dist = sort(obs_dist)
            
        for k in sort_obs_dist:
            i,j = k
            new_tuple = () 
            if i not in phi_list:
                phi_list.append(i)
                new_tuple = (i,j)
                new_list.append(new_tuple)
                new_tuple = ()          
        for k in new_list :
            new_phi,new_d = k
            #print k
            new_phi_list.append(new_phi)
            new_d_list.append(new_d)   
        for k in  range(0,359):                                                                
            if k in new_phi_list:
                my_index = new_phi_list.index(k)
                #print len(new_d_list)
                my_range = new_d_list[my_index]                                 ## assign to scan_range
                scan_ranges.append(my_range)
            else :
                my_range = 0.0
                scan_ranges.append(my_range)      
        now = rospy.get_rostime()
        scan_data.header.stamp.secs = now.secs
        scan_data.header.stamp.nsecs = now.nsecs
        pose_data.header.stamp.secs = now.secs
        pose_data.header.stamp.nsecs = now.nsecs
        pose_pub.publish(pose_data)
        scan_pub.publish(scan_msg)
        phi_list= [] 
        occ_cell =[] 
        occ_data = [] 
        obs_dist = []
        new_list= []
        new_phi_list= []
        new_d_list = []
        scan_ranges = []
        rate.sleep()

if __name__=="__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
