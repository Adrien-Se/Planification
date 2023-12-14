#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class EmergencyStop:
    def __init__(self):
        self.subScan = rospy.Subscriber("/scan", LaserScan, self.callbackScan)
        self.pubCMD_VEL = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.init_node("E_Stop", anonymous=True)
        rospy.spin()
        
    def stop(self):
        print("\rJe me suis arreté, macogne\r")
        msg = Twist()
        msg.linear.x = 0.
        msg.angular.z = 0.
        self.pubCMD_VEL.publish(msg)
        
    def turn(self):
        print("\rJe tourne\r")
        msg = Twist()
        msg.linear.x = 0.
        msg.angular.z = 0.4
        self.pubCMD_VEL.publish(msg)

    def forward(self):
        print("\rJ'avance\r")
        msg = Twist()
        msg.linear.x = 1.
        msg.angular.z = 0.
        self.pubCMD_VEL.publish(msg)


    def callbackScan(self, scan):
        ranges = scan.ranges
        for val in ranges:
            if val < 0.2:
                self.stop()
                return 0
        for val in ranges:
            if val < 0.30:
                self.turn()
                return 0
        self.forward()

rospy.init_node("E_Stop", anonymous=True)
print("Node startup done")
EmergencyStop()


