#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class Follower(object):
    """Follower object that subscribes to /scan, gets nearest object, and publishes to /cmd_vel to follow it"""

    # initialize values for object
    def __init__(self, speed_update_size:int = 0.1):
        rospy.init_node('follower')
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.current_speed = 0
        self.speed_update_size = speed_update_size
        self.target_speed = 0
        self.turn_speed = 0

    # gradually adjust speed every scan call to avoid drifting
    def update_target_speed(self):
        if self.current_speed < self.target_speed: #current speed is less than target, increase speed
            self.current_speed = min(self.current_speed + self.speed_update_size, self.target_speed)
        elif self.current_speed > self.target_speed: #current speed is more than target, decrease speed
            self.current_speed = max(self.current_speed - self.speed_update_size, self.target_speed)

    # recieve Lidar Scan data and process it
    def process_scan(self, data):
        min_distance = data.ranges[0]
        min_angle = 0
        #loop over all angles and find closest object
        for index in range(360):
            if data.ranges[index] < min_distance:
                min_angle = index
                min_distance = data.ranges[index]
        #make angles > 180 negative angles, makes setting angular velocity easy.
        if min_angle > 180:
            min_angle += -360

        if min_distance > 0.5: #Further than 0.5m, increase speed up to 1m/sec
            self.target_speed = min(abs(min_distance - 0.5), 1)
        else: #closer than 0.5m,
            self.target_speed = (min_distance - 1)
        #Increase speed when angle is aligned with object
        self.target_speed = 5*self.target_speed/max(abs(min_angle), 5)
        self.update_target_speed()

        #pi/180 ~= 0.0175, divide by speed to turn slower at high speeds
        self.turn_speed = (min_angle*0.0175)/(10*max(self.current_speed, 0.1))

        #publish velocity message
        velocity_msg = Twist(Vector3(self.current_speed, 0, 0), Vector3(0, 0, self.turn_speed))
        self.publisher.publish(velocity_msg)

    # subscribes to scan, which causes the object to begin taking in data
    # and begin updating its velocities
    def run(self):
        rospy.Subscriber("/scan", LaserScan, self.process_scan)

if __name__ == '__main__':
    follower_node = Follower()
    follower_node.run()
    rospy.spin()