#!/usr/bin/env python3
import random
import rospy
from move_base_msgs.msg import MoveBaseActionGoal
from move_base_msgs.msg import MoveBaseActionResult
from nav_msgs.msg import OccupancyGrid

# limitation to generate random target based on map, adjusted value regarding to limit validation time.
x_max = 2.1
x_min = -2.1
y_max = 2.1
y_min = -2.1
w_max = 0.99
w_min = 0.4

def validate_random_target(x,y):
    x_offset = occupancy_grid.info.origin.position.x
    y_offset = occupancy_grid.info.origin.position.y
    width = occupancy_grid.info.width

    resolution = occupancy_grid.info.resolution
    xint = (x - x_offset) / resolution
    yint = (y - y_offset) / resolution
    pos = int(yint * width + xint) #position of x,y in array of data
    return occupancy_grid.data[pos] == 0 #0 equal to achievable position, return True resulting target has been validated.

def create_random_target():
    x = random.uniform(x_min, x_max)
    y = random.uniform(y_min, y_max)
    w = random.uniform(w_min, w_max)
    return x, y, w

def generate_random_target():
    x = 0
    y = 0
    w = 0
    flag_validate = False
    while(not flag_validate): #loop if position not satisfied
        (x,y,w) = create_random_target()
        flag_validate = validate_random_target(x,y)
    return x,y,w

def callback(dt): #get the data of map
    global occupy_map_flag, occupancy_grid
    if occupy_map_flag:
        occupy_map_flag = False
        occupancy_grid = dt
        
def result_callback(dt): #get the result of trajectory execution
    global reset_target_flag, reset_counter
    if dt.status.status == 3 or reset_counter > 25: #if the goal achieved or timeout, reset and generate new random target
        reset_target_flag = True
        reset_counter = 0
    elif dt.status.status == 2:
        reset_counter+=1

rospy.init_node ('object_avoidance')
pub = rospy.Publisher ('/move_base/goal', MoveBaseActionGoal, queue_size = 5)
sub = rospy.Subscriber("/map", OccupancyGrid, callback)
result = rospy.Subscriber("/move_base/result", MoveBaseActionResult, result_callback)
rate = rospy.Rate (1)

target_pos = MoveBaseActionGoal ()
target_pos.goal.target_pose.header.frame_id = "map"

global occupy_map_flag, occupancy_grid, reset_target_flag, reset_counter
occupancy_grid = OccupancyGrid()
occupy_map_flag = True
reset_target_flag = False
reset_counter = 0
rospy.sleep(0.5)

(x_pos, y_pos, w_pos) = generate_random_target()
print("x:",x_pos, " y:",y_pos, " w:",w_pos)
target_pos.goal.target_pose.pose.position.x = x_pos
target_pos.goal.target_pose.pose.position.y = y_pos
target_pos.goal.target_pose.pose.orientation.w = w_pos

while not rospy.is_shutdown ():
    if reset_target_flag:
        (x_pos, y_pos, w_pos) = generate_random_target()
        print("x:",x_pos, " y:",y_pos, " w:",w_pos)
        target_pos.goal.target_pose.pose.position.x = x_pos
        target_pos.goal.target_pose.pose.position.y = y_pos
        target_pos.goal.target_pose.pose.orientation.w = w_pos
        reset_target_flag = False

    target_pos.header.stamp = rospy.Time.now ()
    target_pos.goal.target_pose.header.stamp = rospy.Time.now ()
    pub.publish (target_pos)
    rate.sleep ()