import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class EmergencyStop:
    
    def __init__(self):
        rospy.init_node("E_Stop", anonymous=True)
        self.subScan = rospy.Subscriber("/scan", LaserScan, self.callbackScan)
        self.pubCMD_VEL = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.spin()
        
    def stop(self):
        print("\rJe me suis arreté, macogne\r")
        msg = Twist()
        msg.linear.x = 0.
        msg.angular.z = 0.
        self.pubCMD_VEL.publish(msg)
        
    def turn_left(self):
        print("\rJe tourne a droite\r")
        msg = Twist()
        msg.linear.x = 0.
        msg.angular.z = 0.4
        self.pubCMD_VEL.publish(msg)
        
    def turn_right(self):
        print("\rJe tourne a gauche\r")
        msg = Twist()
        msg.linear.x = 0.
        msg.angular.z = -0.4
        self.pubCMD_VEL.publish(msg)

    def forward(self):
        print("\rJ'avance tout droit\r")
        msg = Twist()
        msg.linear.x = 1.
        msg.angular.z = 0.
        self.pubCMD_VEL.publish(msg)


    def callbackScan(self, scan):
        ranges = scan.ranges
        
        # Séparer les valeurs du scan en deux moitiés, gauche et droite
        left_ranges = ranges[:len(ranges)//2]
        right_ranges = ranges[len(ranges)//2:]
        
        # Vérifier s'il y a un obstacle à gauche
        if min(left_ranges) < 0.3:
            self.turn_left()
        # Vérifier s'il y a un obstacle à droite
        elif min(right_ranges) < 0.3:
            self.turn_right()
        # Aucun obstacle détecté, avancer
        else:
            self.forward()
            
        for val in ranges:
            if val < 0.2:
                self.stop()
                return 0


if __name__ == "__main__":
    # Instancier la classe EmergencyStop
    e_stop = EmergencyStop()

    # Attente de l'arrêt du programme
    rospy.spin()
