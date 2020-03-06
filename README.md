# turtlebot_related Nodes in Python  (kindly refer the turtlebot documentation)  

'AMCL 'is to calculate localization score of robot pose in given map which requrired dependancies related to python and ROS msgs as subscription.

'Map & Scan' is to create a manual map and laser scan in given initial pose transformation is done only for Waffle pi model. and it will publish /my_map and /my_scan topics which cane be called using map_server service to save the map. 

'Lidar' is to publish a /scan topic given map occupancy data. or subscribing from /map

'stop' is to control the motor movement by subscribing laser_scan data and control the robot in moving obstacle envrionment
