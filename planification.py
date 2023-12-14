#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from nav_msgs.srv import GetMap
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

class AutonomousNavigationRobot:
    def __init__(self):
        rospy.init_node("autonomous_navigation_robot", anonymous=True)

        # Abonnements aux topics nécessaires
        self.sub_scan = rospy.Subscriber("/scan", LaserScan, self.callback_scan)
        self.sub_odom = rospy.Subscriber("/odom", Odometry, self.callback_odom)

        # Éditeur de commandes de vitesse
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Initialisation des variables pour le SLAM
        self.mapping_complete = False
        self.robot_pose = None

        # Service pour demander la carte générée par gmapping
        self.get_map_service = rospy.ServiceProxy('/dynamic_map', GetMap)

        # Attendre que le service de la carte soit disponible
        rospy.wait_for_service('/dynamic_map')

        # Appeler le service pour générer la carte
        self.call_get_map_service()

        # Initialiser l'action client pour move_base
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()

        rospy.spin()

    def call_get_map_service(self):
        try:
            # Appeler le service pour obtenir la carte
            response = self.get_map_service()

            # Stocker la carte dans une variable (par exemple, response.map)
            # Implémentez ici la logique pour utiliser la carte générée par gmapping
            pass

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def callback_scan(self, scan):
        # Implémentez ici la logique de gestion des scans pour gmapping
        pass

    def callback_odom(self, odom):
        # Obtenez la pose du robot depuis le message d'odométrie
        self.robot_pose = odom.pose.pose

        # Si la carte est terminée et que la position du robot est disponible, déplacez le robot
        if self.mapping_complete and self.robot_pose is not None:
            self.navigate()

    def navigate(self):
        # Implémentez ici la logique de navigation en utilisant move_base

        # Exemple: Définir une position de destination (à ajuster selon votre carte)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = 1.0
        goal.target_pose.pose.position.y = 1.0
        goal.target_pose.pose.orientation.w = 1.0

        # Envoyer la position de destination à move_base
        self.move_base_client.send_goal(goal)

if __name__ == "__main__":
    autonomous_navigation_robot = AutonomousNavigationRobot()
