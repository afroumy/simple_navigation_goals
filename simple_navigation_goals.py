#! /usr/bin/env python
import rospy
from math import pow, atan2, sqrt
from geometry_msgs.msg import Twist, Pose2D


class TurtleBot:
    def __init__(self):
        # Creating a node
        rospy.init_node("turtlebot3_ctl", anonymous=True)

        # Using publisher to edit linear and angular velocity
        self.velocity_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        # Uing publisher to edit pose2D
        self.pose2D_subscriver = rospy.Subscriber(
            "/turtlebot3/pose2D", Pose2D, self.update_pose2D
        )

        self.pose2D = Pose2D()
        self.rate = rospy.Rate(10)

    def update_pose2D(self, data):
        self.pose2D = data
        self.pose2D.x = round(self.pose2D.x, 4)
        self.pose2D.y = round(self.pose2D.y, 4)

    def distance(self, goal_pose2D):
        # distance entre la position actuelle et le goal
        return sqrt(
            pow((goal_pose2D.x - self.pose2D.x), 2)
            + pow((goal_pose2D.y - self.pose2D.y), 2)
        )

    def line_speed(self, goal_pose2D, constant=0.1):
        # Definition de la vitesse lineaire
        return constant * self.distance(goal_pose2D)

    def steering_angle(self, goal_pose2D):
        # Definition angle rotation
        return atan2(goal_pose2D.y - self.pose2D.y, goal_pose2D.x - self.pose2D.x)

    def angular_speed(self, goal_pose2D, constant=0.2):
        # Definie le calcul de la vitesse angulaire
        return constant * (self.steering_angle(goal_pose2D) - self.pose2D.theta)

    def goto(self):
        # Deplace le robot jusqu'au point voulu
        goal_pose2D = Pose2D()

        # recuperation des informations de l'utilisateur
        goal_pose2D.x = input("Rentrez la position en x :")
        goal_pose2D.y = input("Rentrez la position en y:")
        goal_pose2D.theta = input("Rentrez l'angle")

        # Definition tolerance --> gestion de l'espace proche
        distance_tolerance = input("Definition de la tolerance :")

        speed_msg = Twist()

        while self.distance(goal_pose2D) >= distance_tolerance:

            # Proportionnel controle

            # Vitesse linaire en x
            speed_msg.linear.x = self.line_speed(goal_pose2D)
            speed_msg.linear.y = 0
            speed_msg.linear.z = 0

            # Vitesse angulaire en z
            speed_msg.angular.x = 0
            speed_msg.angular.y = 0
            speed_msg.angular.z = self.angular_speed(goal_pose2D)

            # Edition notre vitesse
            self.velocity_publisher.publish(speed_msg)

            # Edition attente
            self.rate.sleep()

        # Arret du robot une fois que le point est atteint
        speed_msg.linear.x = 0
        speed_msg.angular.z = 0
        self.velocity_publisher.publish(speed_msg)

        # Gestion de l'annulation du programme (crl+c)
        rospy.spin()


if __name__ == "__main__":
    try:
        x = TurtleBot()
        x.goto()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
