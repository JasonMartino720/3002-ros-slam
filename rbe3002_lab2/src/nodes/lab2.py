#!/usr/bin/env python2

import math
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from tf.transformations import euler_from_quaternion

class Lab2:

    def __init__(self):
        """
        Class constructor
        """
        ### REQUIRED CREDIT
        ### Initialize node, name it 'lab2'
        rospy.init_node('lab2', anonymous=True)
        ### Tell ROS that this node publishes Twist messages on the '/cmd_vel' topic
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        ### Tell ROS that this node subscribes to Odometry messages on the '/odom' topic
        ### When a message is received, call self.update_odometry
        rospy.Subscriber("/odom", Odometry, self.update_odometry())

        ### Tell ROS that this node subscribes to PoseStamped messages on the '/move_base_simple/goal' topic
        ### When a message is received, call self.go_to
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.go_to())


    def send_speed(self, linear_speed, angular_speed):
        """
        Sends the speeds to the motors.
        :param linear_speed  [float] [m/s]   The forward linear speed.
        :param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
        """
        ### REQUIRED CREDIT
        ### Make a new Twist message
        msg_cmd_vel = Twist()
        linear = Vector3()
        linear.x = linear_speed
        linear.y = 0
        linear.z = 0
        angular = Vector3()
        angular.x = 0
        angular.y = 0
        angular.z = angular_speed
        msg_cmd_vel.linear = linear
        msg_cmd_vel.angular = angular
        ### Publish the message
        self.pub.publish(msg_cmd_vel)

    def drive(self, distance, linear_speed):
        """
        Drives the robot in a straight line.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The forward linear speed.
        """
        tolerance = 0.005 #meters

        self.ix = self.px
        self.iy = self.py
        self.send_speed(linear_speed,0)

        while(self.dist_between(self.ix,self.iy,self.px,self.py) < distance - tolerance):
            rospy.sleep(0.005)

        self.send_speed(0,0)

    def rotate(self, angle, aspeed):
        """
        Rotates the robot around the body center by the given angle.
        :param angle         [float] [rad]   The distance to cover.
        :param angular_speed [float] [rad/s] The angular speed.
        """
        tolerance = 0.005 #meters

        self.ith = self.pth
        self.send_speed(0, aspeed)

        while(self.pth < self.ith + angle - tolerance):
            rospy.sleep(0.005)

        self.send_speed(0, 0)

    def dist_between(self,ix,iy,x,y):
        return math.sqrt((ix-x * ix-x) + (iy-y * iy-y))

    def go_to(self, msg):
        """
        Calls rotate(), drive(), and rotate() to attain a given pose.
        This method is a callback bound to a Subscriber.
        :param msg [PoseStamped] The target pose.
        """

        ix = msg.point.x
        iy = msg.point.y
        self.rotate(math.atan((py-iy)/(px-ix)))

        self.drive(dist_between(msg.point.x,msg.point.y),0.1)

        quat_orig = msg.quaternion
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        (roll , pitch , yaw) = euler_from_quaternion(quat_list)
        self.rotate(yaw, 0.1)

    def update_odometry(self, msg):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        quat_orig = msg.pose.pose.orientation
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        (roll , pitch , yaw) = euler_from_quaternion(quat_list)
        self.pth = yaw



    def arc_to(self, position):
        """
        Drives to a given position in an arc.
        :param msg [PoseStamped] The target pose.
        """
        ### EXTRA CREDIT
        # TODO
        pass # delete this when you implement your code



    def smooth_drive(self, distance, linear_speed):
        """
        Drives the robot in a straight line by changing the actual speed smoothly.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The maximum forward linear speed.
        """
        ### EXTRA CREDIT
        # TODO
        pass # delete this when you implement your code



    def run(self):
        rospy.spin()

if __name__ == '__main__':
    Lab2().run()
