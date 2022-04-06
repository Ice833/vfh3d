#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

def init():
    rospy.init_node("goal_pub", anonymous=True)
    pub = rospy.Publisher("/goal", PoseStamped, queue_size=10)
    rate = rospy.Rate(2)
    goal = PoseStamped()
    goal.pose.position.x = 6
    goal.pose.position.y = -7.5
    goal.pose.position.z = 10
    goal.pose.orientation.w = 0.3827
    goal.pose.orientation.x = 0
    goal.pose.orientation.y = 0
    goal.pose.orientation.z = -0.3827
    goal.header.frame_id = "map"
    rospy.loginfo(goal)
    while not rospy.is_shutdown():
        pub.publish(goal)
        rate.sleep()

if __name__ == "__main__":
    init()
