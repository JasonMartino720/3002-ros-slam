#!/usr/bin/env python2

import math
import rospy
from nav_msgs.msg import Odometry, Path
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped, Twist, Vector3, Point, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class Lab4:

    def __init__(self):
        """
        Class constructor
        """
        self.px = 0
        self.py = 0
        self.pth = 0

        ### REQUIRED CREDIT
        ### Initialize node, name it 'lab3'
        rospy.init_node('lab4', anonymous=True)
        ### Tell ROS that this node publishes Twist messages on the '/cmd_vel' topic
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.pubGoal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)

        ### Tell ROS that this node subscribes to Odometry messages on the '/odom' topic
        ### When a message is received, call self.update_odometry
        rospy.Subscriber("/odom", Odometry, self.update_odometry)

        rospy.Subscriber("/robot_path", Path, self.execute_path)


    def execute_path(self, msg):
        """
        Expecing a world point Path
        """
        TOLERANCE = 0.1

        waypoints = msg.poses

        for pose in waypoints:
            # self.go_to(pose)
            self.pubGoal(pose)
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


    def rotate(self, angle, aspeed):
        """
        Rotates the robot around the body center by the given angle.
        :param angle         [float] [rad]   The distance to cover.
        :param angular_speed [float] [rad/s] The angular speed.
        """
        TOLERANCE = 0.0001  # rad

        goalAngle = normalize_angle(angle+math.pi)

        Kp = -12.5
        Ki = -1
        Kd = -6

        integral = 0
        lastError = 0

        error = self.pth - goalAngle
        while abs(error) > TOLERANCE:
            # while True:
            if (self.newOdomReady):
                self.newOdomReady = False
                error = self.pth - goalAngle

                integral += error
                integral = max(-5, min(integral, 5))

                derivative = error - lastError
                lastError = error

                pidOutput = (Kp * error) + (Ki * integral) + (Kd * derivative)

                clamped = max(-aspeed, min(pidOutput, aspeed))
                self.send_speed(0, clamped)
                rospy.loginfo(
                    'The target orientation is %f we are currently at %f the error is %f, the intergeral is %f, deriviate is %f -> clamped is %f' % (
                        goalAngle, self.pth, error, integral, derivative, clamped))

        rospy.loginfo("Rotate Done!")
        self.send_speed(0, 0)

    def drive_and_turn(self, angular_speed_lim, linear_speed_lim, goal):
        drive_Kp = 4.5
        # drive_Kp = 0.440
        # drive_Ki = 0.05
        drive_Ki = 0
        drive_Kd = 4
        turn_Kp = -1.0
        turn_Ki = 0
        turn_Kd = 0 #-12

        TOLERANCE = 0.5 #in meters from goal
        TURN_HEADSTART = 1.0 #secconds

        # curr_abs_angle = self.absolute_angle(bool(goalAngle > 0))

        turn_integral = 0
        turn_lastError = 0

        drive_lastError = 0
        drive_integral = 0

        while not dist_between(goal.x, goal.y, self.px, self.py) < TOLERANCE:

            if (self.newOdomReady):
                self.newOdomReady = False
                turn_target_angle = angle_to_goal(goal.x, goal.y, self.px, self.py)
                turn_error = self.pth - turn_target_angle

                turn_integral += turn_error
                turn_integral = max(-5, min(turn_integral, 5))

                turn_derivative = turn_error - turn_lastError
                turn_lastError = turn_error

                turn_pidOutput = (turn_Kp * turn_error) + (turn_Ki * turn_integral) + (turn_Kd * turn_derivative)

                turn_clamped = max(-angular_speed_lim, min(turn_pidOutput, angular_speed_lim))
                # rospy.loginfo(
                #     'The target orientation is %f we are currently at %f the error is %f, the intergeral is %f, deriviate is %f -> clamped is %f' % (
                #         turn_target_angle, self.pth, turn_error, turn_integral, turn_derivative, turn_clamped))

                #Drive Section Below

                drive_error = dist_between(goal.x, goal.y, self.px, self.py)

                drive_integral += drive_error
                drive_integral = max(-1, min(drive_integral, 1))
                drive_derivative = drive_error - drive_lastError
                drive_lastError = drive_error

                drive_pidOutput = (drive_Kp * drive_error) + (drive_Ki * drive_integral) + (drive_Kd * drive_derivative)

                drive_clamped = max(-linear_speed_lim, min(drive_pidOutput, linear_speed_lim))

                if(rospy.get_time() > self.startPIDTime + TURN_HEADSTART):
                    # rospy.loginfo(
                    #     'The target pos is %f, %f we are currently at %f, %f error %f int %f derivative %f clamped is %f' % (
                    #         self.goal.x, self.goal.y, self.px, self.py, drive_error, drive_integral, drive_derivative, drive_clamped))

                    self.send_speed(drive_clamped, turn_clamped)
                else:
                    rospy.loginfo("Turning Only ")
                    self.send_speed(0.0, turn_clamped)

        self.send_speed(0, 0)



    def absolute_angle(self, positive):
        if positive:
            if self.pth < 0:
                return self.pth + math.radians(360)
            else:
                return self.pth
        else:
            if self.pth > 0:
                return self.pth - math.radians(360)
            else:
                return self.pth


    def go_to(self, msg):
        """
        Calls rotate(), drive(), and rotate() to attain a given pose.
        This method is a callback bound to a Subscriber.
        :param msg [PoseStamped] The target pose.
        """

        # From: https://emanual.robotis.com/docs/en/platform/turtlebot3/specifications/
        MAX_ROTATION_SPEED = 2.84  # Rad/sec
        MAX_DRIVE_SPEED = 0.22  # Meters/sec

        self.goal = msg.pose.position

        rospy.loginfo("Going to goal using drive and turn")
        self.startPIDTime = rospy.get_time()
        self.drive_and_turn(MAX_ROTATION_SPEED, MAX_DRIVE_SPEED, msg.pose.position)
        rospy.sleep(0.5)

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
        self.newOdomReady2 = True

    def run(self):
        rospy.spin()


def orientation_to_yaw(orientation):
    """
    Takes a pose.orientation object and returns the yaw (z rotation)
    """
    quat_orig = orientation
    quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
    (roll, pitch, yaw) = euler_from_quaternion(quat_list)
    return yaw


def dist_between(ix, iy, x, y):
    """
    Get the magnitude of the displacement between two points
    """
    return math.sqrt(((ix - x) * (ix - x)) + ((iy - y) * (iy - y)))


def normalize_angle(angle):
    """
    Converts any angle into the range of -pi -> pi
    Make sure the input is in radian
    :param angle the input angle
    """
    finalAngle = angle
    while (finalAngle > math.pi):
        finalAngle -= math.pi * 2
    while (finalAngle < -math.pi):
        finalAngle += math.pi * 2
    print('Input: %f | Output: %f', (angle, finalAngle))
    return finalAngle




def angle_to_goal(curr_x, curr_y, goal_x, goal_y):
    """
    Find the angle of the goal w.r.t to current position
    """
    return normalize_angle(math.atan2((curr_y - goal_y), (curr_x - goal_x)))

def grid_to_world(self, x, y):
    """
    Transforms a cell coordinate in the occupancy grid into a world coordinate.
    :param mapdata [OccupancyGrid] The map information.
    :param x       [int]           The cell X coordinate.
    :param y       [int]           The cell Y coordinate.
    :return        [Point]         The position in the world.
    """
    world_point = Point()

    world_point.x = (x + 0.5) * self.map.info.resolution + self.map.info.origin.position.x
    world_point.y = (y + 0.5) * self.map.info.resolution + self.map.info.origin.position.y
    # rospy.loginfo("mapdata.info: " + str(mapdata.info))
    # rospy.loginfo("input for grid_to_world: " + str(x) + ", " + str(y))
    # rospy.loginfo("grid_to_world x, y: " + str(world_point.x) + ", " + str(world_point.y))
    return world_point

def world_to_grid(self, wp):
    """
    Transforms a world coordinate into a cell coordinate in the occupancy grid.
    :param mapdata [OccupancyGrid] The map information.
    :param wp      [Point]         The world coordinate.
    :return        [(int,int)]     The cell position as a tuple.
    """

    x = int((wp.x - self.map.info.origin.position.x) / self.map.info.resolution)
    y = int((wp.y - self.map.info.origin.position.y) / self.map.info.resolution)

    grid_coord = (x, y)

    return grid_coord

if __name__ == '__main__':
    Lab4().run()
