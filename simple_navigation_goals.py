#! /usr/bin/env python
import rospy
from math import pow, atan2, sqrt
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, Quaternion


class TurtleBot:
    def __init__(self):
        # Creating a node
        rospy.init_node("turtlebot3_ctl", anonymous=True)

        # Using publisher to edit linear and angular velocity
        self.velocity_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        # Uing publisher to edit pose2D
        self.pose_subscriver = rospy.Subscriber(
            "/turtlebot3/pose", PoseWithCovarianceStamped, self.update_pose,
        )

        self.pose = PoseWithCovarianceStamped()
        self.rate = rospy.Rate(10)

    def update_pose(self, data):
        self.pose = data
        self.pose.pose.point.x = round(self.pose.pose.point.x, 4)
        self.pose.pose.point.y = round(self.pose.pose.point.y, 4)
        self.pose.pose.quaternion.x = round(self.pose.pose.quaternion.x, 4)
        self.pose.pose.quaternion.y = round(self.pose.pose.quaternion.y, 4)
        self.pose.pose.quaternion.z = round(self.pose.pose.quaternion.z, 4)
        self.pose.pose.quaternion.w = round(self.pose.pose.quaternion.w, 4)

    def distance(self, goal_pose):
        # distance entre la position actuelle et le goal
        return sqrt(
            pow((goal_pose.pose.pose.point.x - self.pose.pose.point.x), 2)
            + pow((goal_pose.pose.pose.point.y - self.pose.pose.point.y), 2)
        )

    def line_speed(self, goal_pose, constant=0.1):
        # Definition de la vitesse lineaire
        return constant * self.distance(goal_pose)

    def steering_angle(self, goal_pose):
        # Definition angle rotation
        return atan2(
            goal_pose.pose.pose.point.y - self.pose.pose.point.y,
            goal_pose.pose.pose.point.x - self.pose.pose.point.x,
        )

    # def quaternion_to_euler(self):
    #     x = self.pose.pose.pose.quaternion.x
    #     y = self.pose.pose.pose.quaternion.y
    #     z = self.pose.pose.pose.quaternion.z
    #     w = self.pose.pose.pose.quaternion.w
    #     q = Quaternion(x, y, z, w)
    #     print("quaternion = {}\n".format(q))
    #     return q

    def angular_speed(self, goal_pose, constant=0.2):
        # Definie le calcul de la vitesse angulaire
        # theta = quaternion_to_euler()
        theta = 1
        return constant * (self.steering_angle(goal_pose) - theta)

    def goto(self):

        # theta = self.quaternion_to_euler()
        theta = 1
        # Deplace le robot jusqu'au point voulu
        goal_pose = PoseWithCovarianceStamped()

        # recuperation des informations de l'utilisateur
        goal_pose.pose.point.x = input("Rentrez la position en x :")
        goal_pose.y = input("Rentrez la position en y :")
        goal_pose.theta = input("Rentrez l'angle :")

        # Definition tolerance --> gestion de l'espace proche
        distance_tolerance = 0.1

        speed_msg = Twist()

        print(
            "Position du robot: x {}, y {}, theta {}".format(
                self.pose.pose.pose.point.x, self.pose.pose.pose.point.y, theta,
            )
        )

        print("Lancement...")

        while self.distance(goal_pose) >= distance_tolerance:

            print("En cours")

            # Affiche les positions :
            #
            print(
                "Position du robot en x : {}, y : {} theta: {}\n".format(
                    self.pose.pose.pose.point.x, self.pose.pose.pose.point.y, theta,
                )
            )
            print(
                "Position souhaitee : x {}, y {}, theta {}\n".format(
                    goal_pose.x, goal_pose.y, goal_pose.theta,
                )
            )

            # Proportionnel controle

            # Vitesse linaire en x
            speed_msg.linear.x = self.line_speed(goal_pose)
            speed_msg.linear.y = 0
            speed_msg.linear.z = 0

            # Vitesse angulaire en z
            speed_msg.angular.x = 0
            speed_msg.angular.y = 0
            speed_msg.angular.z = self.angular_speed(goal_pose)

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
        while not rospy.is_shutdown():
            x = TurtleBot()
            x.goto()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
