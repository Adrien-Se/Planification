#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class EmergencyStop:
    def __init__(self):
        """ Initialize the node and the publishers/subscribers """
        
        # Subscriber to the /scan topic
        self.subScan = rospy.Subscriber("/scan", LaserScan, self.callbackScan)
        # Publisher to the /cmd_vel topic
        self.pubCMD_VEL = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        # Initialize the node
        rospy.init_node("E_Stop", anonymous=True)
        # Make the node wait for incoming messages
        rospy.spin()
        
    def stop(self):
        """ Stop the robot """
        print("\r Stop ! Je me suis arrete, juste avant la collision \r")
        
        # Create a message of type Twist
        msg = Twist()
        # Set the velocity to 0, both linear and angular
        msg.linear.x = 0.
        msg.angular.z = 0.
        # Publish the message on the /cmd_vel topic
        self.pubCMD_VEL.publish(msg)
        
    def turn(self):
        """ Turn the robot """
        print("\r Tourne ! Je tourne pour eviter la collision \r")
        
        # Create a message of type Twist
        msg = Twist()
        # Set the linear velocity to 0 and the angular velocity to 0.4 to turn left
        msg.linear.x = 0.
        msg.angular.z = 0.4
        # Publish the message on the /cmd_vel topic
        self.pubCMD_VEL.publish(msg)

    def forward(self):
        """ Move the robot forward """
        print("\r Avance ! J'avance car il n'y a pas d'obstacle \r")
        
        # Create a message of type Twist
        msg = Twist()
        # Set the linear velocity to 1 and the angular velocity to 0 to move forward
        msg.linear.x = 1.
        msg.angular.z = 0.
        # Publish the message on the /cmd_vel topic
        self.pubCMD_VEL.publish(msg)


    def callbackScan(self, scan):
        """ Callback function for the /scan topic """
        
        # Get the ranges from the scan message
        ranges = scan.ranges
        
        # Check if there is an obstacle closer than 0.2m
        for val in ranges:
            if val < 0.2:
                # If there is, stop and exit to scan again
                self.stop()
                return 0
            
        # Check if there is an obstacle closer than 0.3m
        for val in ranges:
                if val < 0.30:
                    # If there is, turn and exit to scan again
                    self.turn()
                    return 0
                
        # If there is no obstacle closer than 0.3m, move forward
        self.forward()

# Initialize the node and start the main loop
EmergencyStop()