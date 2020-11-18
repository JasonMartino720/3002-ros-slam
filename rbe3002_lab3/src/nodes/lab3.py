#!/usr/bin/env python2

import math
import rospy
import sys
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetMap, GetPlan
from geometry_msgs.msg import PoseStamped, Twist, Vector3, Point, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class Lab3:

    def __init__(self):
        """
        Class constructor
        """
        self.px = 0
        self.py = 0
        self.pth = 0

        ### REQUIRED CREDIT
        ### Initialize node, name it 'lab3'
        rospy.init_node('lab3', anonymous=True)
        ### Tell ROS that this node publishes Twist messages on the '/cmd_vel' topic
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        ### Tell ROS that this node subscribes to Odometry messages on the '/odom' topic
        ### When a message is received, call self.update_odometry
        rospy.Subscriber("/odom", Odometry, self.update_odometry)

        ### Tell ROS that this node subscribes to PoseStamped messages on the '/move_base_simple/goal' topic
        ### When a message is received, call self.go_to
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.test)

    def test(self, msg):
        TOLERANCE = 0.1
        rospy.loginfo("Requesting the path")

        msg_to_send = GetPlan()
        curr_pos = PoseStamped()

        #Creating A PoseStamped msg of the current robot position for GetPlan.start
        curr_pos.pose.position = Point(self.px, self.py, 0)
        quat = quaternion_from_euler(0, 0, self.pth)
        curr_pos.pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
        # rospy.loginfo("Creating Msg w/ CurPos: " + str(curr_pos))
        # rospy.loginfo("Creating Msg w/ Goal: " + str(msg))

        #Request Plan
        path_planner = rospy.ServiceProxy('plan_path', GetPlan)
        get_plan_obj = path_planner(curr_pos, msg, TOLERANCE)
        waypoints = get_plan_obj.plan.poses

        rospy.loginfo(waypoints)
        for pose in waypoints:
            self.go_to(pose)

        rospy.loginfo("Path Completed!")

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
        TOLERANCE = 0.00008 #meters
        Kp = 0.4
        Ki = 0.01
        Kd = 0.04

        self.ix = self.px
        self.iy = self.py

        # for x in range(1000):
        #     rospy.sleep(0.005)
        #     self.send_speed(linear_speed*(x/1000.0),0)
        # self.send_speed(linear_speed,0)

        # while(dist_between(self.ix,self.iy,self.px,self.py) < distance - TOLERANCE):
        #     rospy.sleep(0.005)
        #     self.send_speed(linear_speed, 0)
        error = dist_between(self.goal.x,self.goal.y,self.ix,self.iy) - dist_between(self.ix, self.iy, self.px, self.py)
        lastError = 0
        intergral = 0
        while(error > TOLERANCE):

            error = dist_between(self.goal.x,self.goal.y,self.ix,self.iy) - dist_between(self.ix, self.iy, self.px, self.py)

            intergral += error
            derivative = error - lastError
            lastError = error

            pidOutput = (Kp*error) + (Ki*intergral) + (Kd*derivative)

            clamped = max(-linear_speed, min(pidOutput, linear_speed))
            self.send_speed(clamped, -self.angular_z)
            #rospy.loginfo('The target pos is %f, %f we are currently at %f, %f piderror %f clamped is %f' % (
            #self.goal.x, self.goal.y, self.px, self.py, piderror, clamped))


        # for x in range(1000):
        #     rospy.sleep(0.005)
        #     self.send_speed(linear_speed*((1000-x)/1000.0),0)

        # self.send_speed(0,0)


        rospy.loginfo("Move Done!")

        self.send_speed(0,0)

    def rotate(self, angle, aspeed):
        """
        Rotates the robot around the body center by the given angle.
        :param angle         [float] [rad]   The distance to cover.
        :param angular_speed [float] [rad/s] The angular speed.
        """
        TOLERANCE = 0.0001 #rad

        goalAngle = normalize_angle(angle+math.pi)
        # goalAngle = angle+math.pi

        # self.send_speed(0, aspeed)
        Kp = -24.9
        # Ki = -0.00002
        Ki = 0
        Kd = 0


        integral = 0
        lastError = 0

        error = self.pth - goalAngle
        # while abs(error) > TOLERANCE:
        while True:
            if(self.newOdomReady):
                self.newOdomReady = False
                error = self.pth - goalAngle

                integral += error

                derivative = error - lastError
                lastError = error

                pidOutput = (Kp*error) + (Ki*integral) + (Kd*derivative)

                clamped = max(-aspeed, min(pidOutput, aspeed))
                self.send_speed(0, clamped)
                rospy.loginfo('The target orientation is %f we are currently at %f the error is %f, the intergeral is %f, deriviate is %f -> clamped is %f' %(
                goalAngle, self.pth, error, integral, derivative,clamped))

        rospy.loginfo("Rotate Done!")
        self.send_speed(0, 0)

    def go_to(self, msg):
        """
        Calls rotate(), drive(), and rotate() to attain a given pose.
        This method is a callback bound to a Subscriber.
        :param msg [PoseStamped] The target pose.
        """

        #From: https://emanual.robotis.com/docs/en/platform/turtlebot3/specifications/
        ROTATION_SPEED = 0.84 #Rad/sec
        DRIVE_SPEED = 0.22 #Meters/sec

        self.goal = msg.pose.position
        # rospy.loginfo("Curr Pos: " + str(self.px) + " AND " + str(self.py))
        # rospy.loginfo("Goal Pos: " + str(goal.x) + " AND " + str(goal.y))

        rospy.loginfo("Going to intital angle: "+ str(angle_to_goal(self.px,self.py,self.goal.x,self.goal.y)))
        self.rotate(angle_to_goal(self.px,self.py,self.goal.x,self.goal.y), ROTATION_SPEED)
        rospy.loginfo("initial ended at this angle: " + str(self.pth))
        rospy.loginfo("Going distance of: " + str(dist_between(self.px,self.py,self.goal.x,self.goal.y)))
        self.drive(dist_between(self.px,self.py,self.goal.x,self.goal.y), DRIVE_SPEED)
        # rospy.loginfo("Going to final angle: " + str(orientation_to_yaw(msg.pose.orientation)))
        # self.rotate(orientation_to_yaw(msg.pose.orientation), ROTATION_SPEED)
        # rospy.loginfo("final ended at this angle: " + str(self.pth))
        rospy.loginfo("distance between robot and goal: " + str(dist_between(self.goal.x,self.goal.y,self.px,self.py)))

    def update_odometry(self, msg):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """
        self.angular_z = msg.twist.twist.angular.z
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        self.pth = orientation_to_yaw(msg.pose.pose.orientation)
        self.newOdomReady = True

    def arc_to(self, msg):
        """
        Drives to a given position in an arc.
        :param msg [PoseStamped] The target pose.
        """

        TOLERANCE = 0.1 #meters

        goal = msg.pose.position

        self.send_speed(0.22, solve_arc_omega(self.px,self.py,self.pth,goal.x,goal.y))
        while(dist_between(self.px,self.py,goal.x,goal.y) >  TOLERANCE):
            rospy.sleep(0.005)
        print("Arc Done!")

    def smooth_drive(self, distance, linear_speed):
        """
        Drives the robot in a straight line by changing the actual speed smoothly.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The maximum forward linear speed.
        """

        RAMP_DOWN = 0.5 #meters

        self.ix = self.px
        self.iy = self.py

        for x in range(1000):
            rospy.sleep(0.005)
            self.send_speed(linear_speed*(x/1000.0),0)
        self.send_speed(linear_speed,0)

        while(dist_between(self.ix,self.iy,self.px,self.py) < distance - RAMP_DOWN):
            rospy.sleep(0.005)

        for x in range(1000):
            rospy.sleep(0.005)
            self.send_speed(linear_speed*((1000-x)/1000.0),0)

        self.send_speed(0,0)


    def run(self):
        rospy.spin()


def dist_between(ix,iy,x,y):
    """
    Get the magnitude of the displacement between two points
    """
    return math.sqrt(((ix-x) * (ix-x)) + ((iy-y) * (iy-y)))

def normalize_angle(angle):
    """
    Converts any angle into the range of -pi -> pi
    Make sure the input is in radian
    :param angle the input angle
    """
    finalAngle = angle
    while(finalAngle > math.pi):
        finalAngle -= math.pi*2
    while(finalAngle < -math.pi):
        finalAngle += math.pi*2
    print('Input: %f | Output: %f',(angle, finalAngle))
    return finalAngle

def orientation_to_yaw(orientation):
    """
    Takes a pose.orientation object and returns the yaw (z rotation)
    """
    quat_orig = orientation
    quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
    (roll , pitch , yaw) = euler_from_quaternion(quat_list)
    return yaw

def solve_turn_dir(current_angle,goal_angle):
    """
    Takes angles between -pi -> pi and tells you which way to turn
    -1 means CCW and 1 mean CW
    """
    diff = goal_angle - current_angle
    if(diff < 0):
        diff += math.pi
    if(diff > math.pi/2):
        return 1 # left turn
    else:
        return -1 # right turn

def angle_to_goal(curr_x,curr_y,goal_x,goal_y):
    """
    Find the angle of the goal w.r.t to current position
    """
    return normalize_angle(math.atan2((curr_y - goal_y), (curr_x - goal_x)))



def solve_arc_omega(curr_x,curr_y,curr_theta,goal_x,goal_y):
    """
    Solve for the omega required for the arc
    """
    arc_triangle_theta = (math.pi / 2) - (angle_to_goal(curr_x,curr_y,goal_x,goal_y)-curr_theta)
    #Solve for the angle of the right trangle between arc and normal or robot
    a = dist_between(curr_x,curr_y,goal_x,goal_y)/2
    h = a / math.cos(arc_triangle_theta)
    #use trig to solve for hypotenous which is radius of the arc
    r = h
    v = 0.22 #m/sec
    omega = v/r
    return omega

if __name__ == '__main__':
    Lab3().run()
