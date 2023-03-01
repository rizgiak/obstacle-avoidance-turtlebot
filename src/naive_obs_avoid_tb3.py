#!/usr/bin/env python3
import rospy # Python library for ROS
from sensor_msgs.msg import LaserScan # LaserScan type message is defined in sensor_msgs
from geometry_msgs.msg import Twist


#          0
#       15    345 
#    30          330
#  60               300

def steering(dist30, dist60, dist300, dist330):
    z_val = 0.5

    return z_val

def obstacle_avoidance(dt):
    x_val = 0
    z_val = 0

    min_dist0 = 0.8
    min_dist15 = 0.8
    min_dist30 = 0.8
    min_dist60 = 0.8
    min_dist300 = 0.8
    min_dist330 = 0.8
    min_dist345 = 0.8

    # acc
    if dt.ranges[0]>min_dist0 and dt.ranges[15]>min_dist15 and dt.ranges[345]>min_dist345:
        x_val = 0.5 # go forward (linear velocity)
    else:
        x_val = 0.0 # stop
    
    # steering
    if dt.ranges[0]>min_dist0 and dt.ranges[15]>min_dist15 and dt.ranges[345]>min_dist345:
        z_val = 0.0 # do not rotate (angular velocity)
    else: # edit here---------
        z_val = steering(min_dist30, min_dist60, min_dist300, min_dist330) # rotate counter-clockwise
    return x_val, z_val

def callback(dt):
    print ('-------------------------------------------')
    print ('Range data at 0 deg:   {}'.format(dt.ranges[0]))
    print ('Range data at 15 deg:  {}'.format(dt.ranges[15]))
    print ('Range data at 345 deg: {}'.format(dt.ranges[345]))
    print ('-------------------------------------------')
    x_val, z_val = obstacle_avoidance(dt)
    move.linear.x = x_val
    move.angular.z = z_val
    pub.publish(move) # publish the move object


move = Twist() # Creates a Twist message type object
rospy.init_node('obstacle_avoidance_node') # Initializes a node
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)  # Publisher object which will publish "Twist" type messages
                            				 # on the "/cmd_vel" Topic, "queue_size" is the size of the
                                                         # outgoing message queue used for asynchronous publishing

sub = rospy.Subscriber("/scan", LaserScan, callback)  # Subscriber object which will listen "LaserScan" type messages
                                                      # from the "/scan" Topic and call the "callback" function
						      # each time it reads something from the Topic

rospy.spin() # Loops infinitely until someone stops the program execution

