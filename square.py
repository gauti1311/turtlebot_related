#!/usr/bin/env python 
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pi

angular_speed = pi/30
linear_speed = 0.02 
distance = 0.3
angle = pi/2

def callback(data):
        global V
        global w
        V = data.twist.twist.linear.x
        w = data.twist.twist.angular.z 
def main():
        rospy.init_node('tim_path', anonymous=True)
        rate = rospy.Rate(20)
        pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
        rospy.Subscriber("/odom", Odometry, callback)    # subscribe for feedback from actual odometry measruement to calculate error
        msg = Twist()
        distance_current =0
        angle_current= 0
        distance_old =0
        angle_old =0
        actual_linear_speed = 0
        actual_angular_speed = 0
        count = 0 
        state = 0
        global V
        global w
        V= float()
        w= float()
        while not rospy.is_shutdown():
            t1 = rospy.get_time()
            distance_current = actual_linear_speed*(t1-t0)+ distance_old   # update distance and angle
            angle_current = actual_angular_speed*(t1-t0)+ angle_old 
        ## state machine
            # start cycle, state=0
            if state ==0:    
                state = 1                                                         # move forward, state=1
            elif state == 1:   
               if distance_current > distance:
                   state=2
            elif state == 2:
               if angle_current > angle:                      # turn 90, state=2
                   state =3 
                   count = count+1 
               print angle_current
            elif state ==3:
                 if count < 4 :                             # repeat state =1                                            
                    state =1  
                    print count  
                 else:                        
                    state = 4 
                    print ('stop')                    #stop, state =4
                               
                                                     
          # velocity signal for all states
            if state == 0:                                                      
                msg.linear.x= 0
                msg.angular.z= 0
            elif state == 1:
                msg.linear.x= abs(linear_speed)
                msg.angular.z=0
            elif state == 2:
                msg.linear.x= 0 
                msg.angular.z= abs(angular_speed)
            elif state == 3:
                distance_current = 0 
                angle_current =0 
                msg.angular.z=0
            elif state == 4:
                msg.linear.x= 0
                msg.angular.z=0   
            pub.publish(msg)                           # publish msg
            t0 = rospy.get_time() 
            distance_old = distance_current            # update distance and angle
            angle_old = angle_current
            actual_linear_speed = V                           # update speed from odometry signals
            actual_angular_speed= w
            rate.sleep()                                                     
              
if __name__ == '__main__':
     try:
        main()
       
     except rospy.ROSInterruptException: 
         pass
