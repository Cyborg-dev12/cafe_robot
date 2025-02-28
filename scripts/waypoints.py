#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

waypoints = {
    'home': PoseStamped(),
    'kitchen': PoseStamped(),
    'table1': PoseStamped(),
    'table2': PoseStamped(),
    'table3': PoseStamped()
}

def init_waypoints():
    waypoints['home'].header.frame_id = "map"
    waypoints['home'].pose.position.x = 0.0
    waypoints['home'].pose.position.y = 0.0
    waypoints['home'].pose.orientation.w = 1.0

    waypoints['kitchen'].header.frame_id = "map"
    waypoints['kitchen'].pose.position.x = 2.0
    waypoints['kitchen'].pose.position.y = 0.0
    waypoints['kitchen'].pose.orientation.w = 1.0

    waypoints['table1'].header.frame_id = "map"
    waypoints['table1'].pose.position.x = 1.0
    waypoints['table1'].pose.position.y = 1.0
    waypoints['table1'].pose.orientation.w = 1.0

    waypoints['table2'].header.frame_id = "map"
    waypoints['table2'].pose.position.x = 2.0
    waypoints['table2'].pose.position.y = 1.0
    waypoints['table2'].pose.orientation.w = 1.0

    waypoints['table3'].header.frame_id = "map"
    waypoints['table3'].pose.position.x = 3.0
    waypoints['table3'].pose.position.y = 1.0
    waypoints['table3'].pose.orientation.w = 1.0

if __name__ == '__main__':
    rospy.init_node('waypoints')
    init_waypoints()
    rospy.loginfo("Waypoints initialized")
