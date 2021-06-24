#!/usr/bin/env python
import rospy
# from geometry_msgs.msg import PoseStamped, Point, Quaternion
from turtlesim.msg import Pose

rospy.init_node("turtlesim_goal")
while True:
    print("Enter Goal: ")
    x,y,theta = input()
    x = float(x)
    y = float(y)
    theta = float(theta)
    # pub = rospy.Publisher('/move_base_simple/goal',PoseStamped,queue_size=10)
    pub = rospy.Publisher('/target_pose',Pose,queue_size=10)
    rate = rospy.Rate(10)
    goal = Pose()
    goal.x = x
    goal.y = y
    goal.theta = theta
    goal.linear_velocity = 0
    goal.angular_velocity = 0
    pub.publish(goal)
    # rate.sleep()
