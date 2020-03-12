#!/usr/bin/env python
import rospy
import math
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import LaserScan , PointCloud2
from laser_geometry import LaserProjection
from shapely.geometry import Point, Polygon
from std_msgs.msg import Int16, Bool

" To warn turtlebot in within certain zone if moving obstacles come in that region and stop the movement if limit exceeds"


#--------------------------------------------------------------
# Parameters
#--------------------------------------------------------------

# Turtlebot_dimensions, 0 = laser_position
[o,a,b,c,d] = [(0,0),(-215,-165),(-215,165),(215,165),(215,-165)] 

# warn_zone_limit, w
w = 250   
 # stop_zone_limit, s                                                         
s = 50                                                              

warn_zone = Polygon([(a[0]-w,a[1]-w),(b[0]-w,b[1]+w),(c[0]+w,c[1]+w),(d[0]+w,d[1]-w)])
stop_zone = Polygon([(a[0]-s,a[1]-s),(b[0]-s,b[1]+s),(c[0]+s,c[1]+s),(d[0]+s,d[1]-s)])

#--------------------------------------------------------------
# Callback function
#--------------------------------------------------------------

def callback(data):
    global msg
    global trigger
    msg = data
    trigger = 'TRUE'

#--------------------------------------------------------------
# Main function
#--------------------------------------------------------------
    
def main():   
        global msg
        global trigger
        # Init node --------------------------------------
        rospy.init_node("collision_node", anonymous=True)
        # set frequency 50 Hz 
        rate = rospy.Rate(50)         
        # set_up motor_power publisher                                       
        motor_power_pub = rospy.Publisher('/motor_power',Bool,queue_size=10)  
        # set_up publisher to  coll_warn topic
        coll_warn_pub = rospy.Publisher('coll_warn',Int16,queue_size=10)     
        # set_up subscriber to laserscan
        laser_sub= rospy.Subscriber("/scan", LaserScan, callback)           
        motor_power_cmd = Bool()
        # set coll_warn message 
        coll_warn_msg = [0,1,2]                                              
        msg = LaserScan()
        laser_projection = LaserProjection()
        # set_up initial conditions 
        trigger = 'FALSE'
        warninig_now = 'FALSE'
        warning_updated = 'FALSE'
        stop_now = 'FALSE'
        stop_updated = 'FALSE'
        # set_motor_power True as initial condition
        motor_power_cmd.data = 1

        # Control loop --------------------------------------------
        while not rospy.is_shutdown():
            warning_now = 'FALSE'
            stop_now = 'FALSE'
            if trigger == 'TRUE':                  
                # convert laser_scan data to Point_Cloud2                        
                point_cloud2_msg = laser_projection.projectLaser(msg)     
                # read pc2 as points      
                point_generator = pc2.read_points(point_cloud2_msg)        
                for point in point_generator:
                    x = point[0]
                    y = point[1]
                    # access as co-ordinates
                    p = Point(x*1000,y*1000)                            
                    if p.within(warn_zone):                                    
                        if p.within(stop_zone):
                            stop_now = 'TRUE'
                        else:
                            warning_now = 'TRUE'

                # check for stop_zone condition --------------------
                if stop_now == 'TRUE' or stop_updated =='TRUE': 
                    motor_power_cmd.data = 0
                    motor_power_pub.publish(motor_power_cmd)                    
                    coll_warn_pub.publish(coll_warn_msg[2])
                    print ('stop')

                # check for warn_zone condition --------------------
                elif warning_now == 'TRUE':
                    motor_power_cmd.data = 1
                    coll_warn_pub.publish(coll_warn_msg[1])  
                    print ('warning') 
                else:     
                    if warning_updated == 'TRUE' or stop_updated == 'TRUE':
                        motor_power_cmd.data = 1
                        motor_power_pub.publish(motor_power_cmd)
                        print ('clear')    
                        coll_warn_pub.publish(coll_warn_msg[0])  
                # update warning and stop      
                warning_updated = warning_now                                                  
                stop_updated = stop_now 
                # set trigger as False
                trigger = 'FALSE'
            rate.sleep()             
                       
if __name__ == '__main__':
    try:
        main()      
    except rospy.ROSInterruptException:
        pass        

