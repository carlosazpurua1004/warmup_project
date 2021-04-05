#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist


class SquareDriver(object):
    """This object publishes to /cmd_vel"""

    # note default value of turn speed is an approximation of pi/10,
    # so that when 5 seconds are spent turning, a 90 degree turn is made
    def __init__(self, init_state:str='foward', max_speed:float=0.5, turn_speed:float = 0.32):
        rospy.init_node('square_driver')
        self.state = init_state
        self.max_speed = max_speed
        self.foward_duration = 25
        self.turn_speed = turn_speed
        self.turn_duration = 25
        self.current_speed = 0
        self.current_turn_speed = 0

    # computes speed as function of time, so that speed can gradually
    # be increased to avoid the effects of friction
    def compute_speed(self):
        self.current_speed = self.max_speed * (-1 * abs(13 - self.foward_duration) + 13 )/13

    # runs the publishing while loop of the driver 
    def run(self):
        publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        r = rospy.Rate(5)
        # publishes 5 messages per second
        while not rospy.is_shutdown():
            if self.state == 'foward': # robot in 'foward' state
                self.compute_speed()
                self.foward_duration -= 1
                if self.foward_duration < 0: #25 cycles passed, switch states
                    self.foward_duration = 25
                    self.current_speed = 0
                    self.state = 'turn'
            elif self.state == 'turn': # robot in 'turn' state
                self.current_turn_speed = self.turn_speed
                self.turn_duration -= 1
                if self.turn_duration < 0: #25 cycles passed, switch states
                    self.turn_duration = 25
                    self.current_turn_speed = 0
                    self.state = 'foward'
            velocity_msg = Twist(Vector3(self.current_speed, 0, 0), Vector3(0, 0, self.current_turn_speed))
            publisher.publish(velocity_msg)
            r.sleep()

if __name__ == '__main__':
    sd_node = SquareDriver()
    sd_node.run()