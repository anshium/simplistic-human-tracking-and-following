#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Int16
from geometry_msgs.msg import Twist

class PersonFollowingRobot:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('person_following_robot', anonymous=True)
        
        # Initialize subscribers
        self.delta_sub = rospy.Subscriber('/delta', Float32, self.delta_callback)
        self.distance_sub = rospy.Subscriber('/distance', Float32, self.distance_callback)
        self.alpha_sub = rospy.Subscriber('/alpha', Float32, self.alpha_callback)
        self.is_person_sub = rospy.Subscriber('/is_person', Int16, self.is_person_callback)
        
        # Initialize publisher
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Initialize variables
        self.delta = 0.0
        self.distance = 0.0
        self.alpha = 0.0
        self.is_person = 0
        
        # Initialize rate
        self.rate = rospy.Rate(10)  # 10 Hz
        
        # Run the main loop
        self.run()

    def delta_callback(self, data):
        self.delta = data.data

    def distance_callback(self, data):
        self.distance = data.data

    def alpha_callback(self, data):
        self.alpha = data.data

    def is_person_callback(self, data):
        self.is_person = data.data

    def run(self):
        while not rospy.is_shutdown():
            twist_msg = Twist()
            
            if self.is_person:
                # Calculate linear and angular velocities based on delta, distance, and alpha
                twist_msg.linear.x = self.calculate_linear_velocity(self.distance)
                twist_msg.angular.z = self.calculate_angular_velocity(self.delta, self.alpha)
            else:
                # If no person is detected, stop the robot
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0

            # Publish the twist message
            self.cmd_vel_pub.publish(twist_msg)

            # Sleep to maintain the loop rate
            self.rate.sleep()

    def calculate_linear_velocity(self, distance):
        # Implement a function to calculate linear velocity based on distance
        max_speed = 0.5  # Maximum speed of the robot
        min_distance = 0.5  # Minimum distance to the person
        
        if distance > min_distance:
            return min(max_speed, distance * 0.2)  # Simple proportional control
        else:
            return 0.0

    def calculate_angular_velocity(self, delta, alpha):
        # Implement a function to calculate angular velocity based on delta and alpha
        max_turn_speed = 1.0  # Maximum turn speed of the robot
        
        return max_turn_speed * (delta * 0.005 + alpha * 0.01)  # Simple proportional control

if __name__ == '__main__':
    try:
        PersonFollowingRobot()
    except rospy.ROSInterruptException:
        pass
