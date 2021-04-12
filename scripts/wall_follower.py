#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class Follower(object):
    """WallFollower object that subscribes to /scan and publishes to /cmd_vel to follow the wall
    Note that this works best when inside covnex shapes """

    # initialize values for object
    def __init__(self):
        rospy.init_node('follower')
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.turn_speed = 0

    # recieve Lidar Scan data and process it
    def process_scan(self, data):
        if data.ranges[0] < 1.5: #wall up ahead, begin turn early
            self.turn_speed = -0.5
        elif data.ranges[45] > data.ranges[135]: #front angle further, turn CCW to be parallel to wall
            self.turn_speed = ((data.ranges[45] - data.ranges[135])/2) ** 0.5
        else: #front angle closer, turn CW to be parallel to wall
            self.turn_speed = -(((data.ranges[135] - data.ranges[45])/2) ** 0.5)

        if 0.2 < data.ranges[90] and data.ranges[90] < 0.7: # far from wall, turn CCW into it
            self.turn_speed += -(((data.ranges[90] - 0.2))*2 ** 0.5)
        elif data.ranges[90] < 0.2: # too close to wall, turn CW away from it it
            self.turn_speed += ((0.2 - data.ranges[90])*2) ** 0.5

        #publish velocity message
        velocity_msg = Twist(Vector3(0.3, 0, 0), Vector3(0, 0, self.turn_speed))
        self.publisher.publish(velocity_msg)

    # subscribes to scan, which causes the object to begin taking in data
    # and begin updating its velocities
    def run(self):
        rospy.Subscriber("/scan", LaserScan, self.process_scan)

if __name__ == '__main__':
    follower_node = Follower()
    follower_node.run()
    rospy.spin()