#!/usr/bin/env python

import math

import rospy
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from nav_msgs.msg import GridCells, Path, OccupancyGrid, Odometry
from nav_msgs.srv import GetPlan, GetMap
from tf_conversions.posemath import transformations
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from rbe3002_lab4.srv._frontiers import frontiers, frontiersResponse, frontiersRequest
from rbe3002_lab4.msg._tupleMSG import tupleMSG
from rbe3002_lab4.msg._clusterMSG import clusterMSG

from priority_queue import PriorityQueue


class PathPlanner:
    def __init__(self):
        """
        Class constructor
        """
        # Initialize the node and call it "path_planner"
        rospy.loginfo("Started path planner")
        rospy.init_node("path_planner")
        # Create a new service called "plan_path" that accepts messages of
        # type GetPlan and calls self.plan_path() when a message is received
        ## Create publishers for A* (expanded cells, frontier, ...)
        ## Choose a the topic names, the message type is GridCells

        self.pubPath = rospy.Publisher("/robot_path", Path, queue_size=10)

        rospy.Subscriber("/odom", Odometry, self.update_odometry)

        ### Tell ROS that this node subscribes to PoseStamped messages on the '/move_base_simple/goal' topic
        ### When a message is received, call self.go_to
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.phase_two_loop())

        rospy.sleep(1.0)
        rospy.loginfo("Path planner node ready")

        rospy.wait_for_service('cspace')
        self.set_info()
        self.phase_one_loop()


    def update_odometry(self, msg):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """
        self.angular_z = msg.twist.twist.angular.z
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        self.pth = self.orientation_to_yaw(msg.pose.pose.orientation)
        self.newOdomReady = True
        self.newOdomReady2 = True

    def set_info(self):
        cspace_srv = rospy.ServiceProxy('cspace', GetMap)
        try:
            self.info = cspace_srv().map.info
            self.header = cspace_srv().map.header
        except rospy.ServiceException, e:
            rospy.logerr(e)
            rospy.logerr("Info was not set!")
            self.info = None

    def phase_one_loop(self):
        while True:
            #Update C-space
            #Detect frontier cells with edge dectection
            cspace_srv = rospy.ServiceProxy('cspace', GetMap)
            self.map = cspace_srv().map
            #Cluster frontier cells

            rospy.loginfo("Calling frontier service")

            #Attribute error
            frontier_srv = rospy.ServiceProxy('frontierTopic', frontiers)
            frontier_list = frontier_srv().list_of_cells

            rospy.loginfo(frontier_list)
            rospy.loginfo("Return from frontier service")

            # frontier_list = frontier_srv().list_of_cells
            rospy.loginfo(frontier_list)
            if len(frontier_list) > 0:
                    #Are we expecting a list of lists of tuples
                    #(x,y),(x,y)...
                    # OR
                    #(x, y, size),(x, y, size)...
                #Path Plan to each frontier
                    #Using the conversion layer, we will convert curr pos to grid cells


                centroid_list = list()
                distance_list = list()

                centroidX = 0
                centroidY = 0

                rospy.loginfo("inside phase_one_loop")

                for cluster_MSG in frontier_list:
                    cluster = cluster_MSG.clusterDATA
                    for point_MSG in cluster:
                        point = point_MSG.tupleDATA
                        centroidX += point[0]
                        centroidY += point[1]
                    centroidX /= len(cluster)
                    centroidY /= len(cluster)
                    centroid_list.append((centroidX, centroidY))

                rospy.loginfo("centroid_list: " + str(centroid_list))

                for centroid in centroid_list:
                    pt = Point(self.px, self.py, 0)
                    pointlist = self.a_star(self.world_to_grid(pt), centroid)
                    distance_to_frontier = len(pointlist)
                    distance_list.append(distance_to_frontier)
                rospy.loginfo("distance_list: " + str(distance_list))

                #Return sorted list with gird plan length and size of frontier
                size_list = list()
                for cluster_MSG in frontier_list:
                    cluster = cluster_MSG.clusterDATA
                    size_list.append(len(cluster))


                metric_list = list()
                for size, distance in zip(size_list, distance_list):
                    metric = float(size) / float(distance)
                    metric_list.append(metric)

                rospy.loginfo("Lists")
                rospy.loginfo(metric_list)
                rospy.loginfo(centroid_list)
                rospy.loginfo(size_list)
                rospy.loginfo(distance_list)

                combined = list(reversed(sorted(zip(metric_list, centroid_list, size_list, distance_list))))
                rospy.loginfo(combined)
                sorted_centeroids = [x for _, x, _, _ in combined]
                rospy.loginfo(sorted_centeroids)
                #Has goal changed?
                #TEMP Removed
                # attribute error in line 123
                # curr_nav_goal = rospy.ServiceProxy('what_is_current_goal', "void->tuple")
                # curr_goal = curr_nav_goal()
                # temporary curr_goal
                curr_goal = (0,0)

                # next line is index out of range error, at this point, I think the sorted_centeroids list is empty
                # because all of the lists are empty, including distance_list and centroid_list from above.
                new_goal = sorted_centeroids[0]
                if not self.is_within_threshold(curr_goal, new_goal):
                # <If yes> convert grid plan to world plan ()

                    # Creating A PoseStamped msg of the current robot position for GetPlan.start
                    curr_pos = PoseStamped()
                    curr_pos.pose.position = Point(self.px, self.py, 0)
                    quat = quaternion_from_euler(0, 0, self.pth)
                    curr_pos.pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])

                    # Convert grid plan to world plan
                    world_point = self.grid_to_world(new_goal[0], new_goal[1])

                    goal_pos = PoseStamped()
                    goal_pos.pose.position = world_point
                    quat = quaternion_from_euler(0, 0, 0)
                    goal_pos.pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])

                    # Request Plan
                    plan_to_send_robot = self.get_path_to_point(curr_pos, goal_pos)

                    #Jason please send this plan to the Lab 4 robot and we're done
                    self.pubPath.publish(plan_to_send_robot)


                    # set_nav_path(get_plan_obj)



        #Then send this position to the new navigation node

        # <If no> Restart this loop

    def phase_two_loop(self):
        return None

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


    @staticmethod
    def euclidean_distance(x1, y1, x2, y2):
        """
        Calculates the Euclidean distance between two points.
        :param x1 [int or float] X coordinate of first point.
        :param y1 [int or float] Y coordinate of first point.
        :param x2 [int or float] X coordinate of second point.
        :param y2 [int or float] Y coordinate of second point.
        :return   [float]        The distance.
        """
        return math.sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1))

    @staticmethod
    def orientation_to_yaw(orientation):
        """
        Takes a pose.orientation object and returns the yaw (z rotation)
        """
        quat_orig = orientation
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)
        return yaw

    def grid_to_world(self, x, y):
        """
        Transforms a cell coordinate in the occupancy grid into a world coordinate.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The cell X coordinate.
        :param y       [int]           The cell Y coordinate.
        :return        [Point]         The position in the world.
        """
        world_point = Point()

        world_point.x = (x + 0.5) * self.info.resolution + self.info.origin.position.x
        world_point.y = (y + 0.5) * self.info.resolution + self.info.origin.position.y
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

        x = int((wp.x - self.info.origin.position.x) / self.info.resolution)
        y = int((wp.y - self.info.origin.position.y) / self.info.resolution)

        grid_coord = (x, y)

        return grid_coord

    def path_to_poses(self, path):
        """
        Converts the given path into a list of PoseStamped.
        :param mapdata [OccupancyGrid] The map information.
        :param  path   [[(int,int)]]   The path as a list of tuples (cell coordinates).
        :return        [[PoseStamped]] The path as a list of PoseStamped (world coordinates).
        """
        # rospy.loginfo("converting path into a list of PoseStamped")
        posestamp_list = []
        for i in range(len(path)):
            yaw = 0
            if i < len(path) - 1:
                yaw = PathPlanner.round_to_45(
                    math.degrees(math.atan2((path[i + 1][1] - path[i][1]), (path[i + 1][0] - path[i][0]))))
            single_pose = PoseStamped()
            pos = self.grid_to_world(path[i][0], path[i][1])
            q = transformations.quaternion_from_euler(0, 0, yaw)
            # going back to ros quaternion
            orient = Quaternion(q[0], q[1], q[2], q[3])
            single_pose.pose.position = pos
            single_pose.pose.orientation = orient
            single_pose.header = self.header
            posestamp_list.append(single_pose)
        return posestamp_list

    def a_star(self, start, goal):
        ### REQUIRED CREDIT
        # rospy.loginfo("Executing A* from (%d,%d) to (%d,%d)" % (start[0], start[1], goal[0], goal[1]))
        cspace_srv = rospy.ServiceProxy('cspace', GetMap)
        try:
            mapdata = cspace_srv()
        except rospy.ServiceException, e:
            rospy.logerr(e)
            mapdata = None

        frontier = PriorityQueue()
        frontier.put(start, 0)

        came_from = {start: None}
        cost_so_far = {start: 0}
        # Path Visualization
        visualize_path = []

        while not frontier.empty():
            current = frontier.get()
            # Path Visualization
            visualize_path.append(current)
            # rospy.loginfo("Adding %f %f to visited list" % (current[0], current[1]))

            if current == goal:
                break

            for neighbour in PathPlanner.neighbors_of_8(self, current[0], current[1]):
                # rospy.loginfo(str(type(cost_so_far[current])))
                # rospy.loginfo(str(cost_so_far[current]))
                # rospy.loginfo(str(current))
                a = cost_so_far[current]
                b = PathPlanner.euclidean_distance(current[0], current[1], neighbour[0], neighbour[1])
                # rospy.loginfo(a)
                # rospy.loginfo(b)
                new_cost = cost_so_far[current] + PathPlanner.euclidean_distance(current[0], current[1], neighbour[0],
                                                                                 neighbour[1])
                new_cost = a + b
                if neighbour not in cost_so_far or new_cost < cost_so_far[neighbour]:
                    cost_so_far[neighbour] = new_cost
                    priority = new_cost + PathPlanner.euclidean_distance(neighbour[0], neighbour[1], goal[0], goal[1])
                    frontier.put(neighbour, priority)
                    came_from[neighbour] = current
                    # rospy.loginfo("Adding a point %d %d to A*'s came from" %(neighbour[0], neighbour[1]))

                # Path Visualization
                visitedCells = []
                for cell in visualize_path:
                    world_point = PathPlanner.grid_to_world(self, cell[0], cell[1])
                    # rospy.loginfo("Converting %f %f to visited list" % (current[0], current[1]))
                    visitedCells.append(world_point)

                # Path Visualization
                ## Create a GridCells message and publish it
                # message for visualizing cells in the visualize_path list
                pvis = GridCells()
                pvis.cell_width = self.info.resolution
                pvis.cell_height = self.info.resolution
                pvis.cells = visitedCells
                pvis.header = self.header
                # self.pubVisited.publish(pvis)
                # rospy.loginfo(pvis)
                # rospy.loginfo("Published this to /visited")

        visualize_path.append(goal)
        # Path Visualization
        visitedCells = []
        for cell in visualize_path:
            world_point = PathPlanner.grid_to_world(self, cell[0], cell[1])
            # rospy.loginfo("Converting %f %f to visited list" % (current[0], current[1]))
            visitedCells.append(world_point)

        # Path Visualization
        ## Create a GridCells message and publish it
        # message for visualizing cells in the visualize_path list
        pvis = GridCells()
        pvis.cell_width = self.info.resolution
        pvis.cell_height = self.info.resolution
        pvis.cells = visitedCells
        pvis.header = self.header
        # self.pubVisited.publish(pvis)
        # rospy.loginfo(pvis)
        # rospy.loginfo("Published this to /visited")

        currPos = goal
        finalPath = []
        finalPath.append(goal)
        while currPos != start:
            currPos = came_from[currPos]
            finalPath.append(currPos)

        finalPath.reverse()
        rospy.loginfo(finalPath)

        # rospy.loginfo("A* FINAL PATH:")
        return finalPath

    @staticmethod
    def optimize_path(path):
        """
        Optimizes the path, removing unnecessary intermediate nodes.
        :param path [[(x,y)]] The path as a list of tuples (grid coordinates)
        :return     [[(x,y)]] The optimized path as a list of tuples (grid coordinates)
        """
        ### EXTRA CREDIT
        # rospy.loginfo("Optimizing path")
        curr_heading = 0
        last_heading = 0
        rmvIndexList = []

        rospy.loginfo("Original Path Length: " + str(len(path)))
        for i in range(1, len(path) - 1):
            rospy.loginfo("Current Point: " + str(i))
            curr_heading = PathPlanner.round_to_45(
                math.degrees(math.atan2((path[i + 1][1] - path[i][1]), (path[i + 1][0] - path[i][0]))))
            rospy.loginfo("Current Heading: " + str(curr_heading))
            rospy.loginfo("Last Heading: " + str(last_heading))

            if curr_heading == last_heading:
                rmvIndexList.append(i)
                last_heading = curr_heading
            else:
                last_heading = curr_heading

        for index in reversed(rmvIndexList):
            path.pop(index)
            rospy.loginfo("Popped: " + str(index))

        return path

    @staticmethod
    def round_to_45(value):
        """
        Round to the nearest 45 degree increment
        param value [double?] The value to be rounded in degrees
        """
        return round(value / 45.0) * 45.0

    def path_to_message(self, path):
        """
        Takes a path on the grid and returns a Path message.
        :param path [[(int,int)]] The path on the grid (a list of tuples)
        :return     [Path]        A Path message (the coordinates are expressed in the world)
        """
        ### REQUIRED CREDIT
        rospy.loginfo("Returning a Path message")
        path_message = Path()
        rospy.loginfo("The path is: " + str(path))
        path_message.poses = PathPlanner.path_to_poses(self, path)
        path_message.header = self.header
        rospy.loginfo("path_message: " + str(path_message))
        return path_message

    def get_path_to_point(self, start, goal):
        """
        Plans a path between the start and goal locations in the requested.
        Internally uses A* to plan the optimal path.
        :param req
        """

        # This pulls the newest map and solves C space

        # ## Execute A*
        start = self.world_to_grid(start.pose.position)
        goal = self.world_to_grid(goal.pose.position)
        path = self.a_star(start, goal)
        rospy.loginfo("a_star output: " + str(path))

        # ## Optimize waypoints
        waypoints = PathPlanner.optimize_path(path)
        rospy.loginfo("Optimized Waypoints: " + str(waypoints))
        # ## Return a Path message, this line can be erased and returned directly after debug
        return_obj = PathPlanner.path_to_message(self, waypoints)
        # rospy.loginfo("path_to_message output: " + str(return_obj))
        return return_obj

    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        # mapdata = PathPlanner.request_map()
        # self.calc_cspace(mapdata,1)
        rospy.loginfo("Path Planner is running")
        rospy.spin()

    def neighbors_of_4(self, x, y):
        """
        Returns the walkable 4-neighbors cells of (x,y) in the occupancy grid.
        :param self [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 4-neighbors.
        """
        return_list = []

        if x != 0 and self.is_cell_walkable(x - 1, y):
            return_list.append((x - 1, y))
        if x != self.info.width - 1 and self.is_cell_walkable(x + 1, y):
            return_list.append((x + 1, y))
        if y != 0 and self.is_cell_walkable(x, y - 1):
            return_list.append((x, y - 1))
        if y != self.info.height - 1 and self.is_cell_walkable(x, y + 1):
            return_list.append((x, y + 1))

        return return_list

    def neighbors_of_8(self, x, y):
        """
        Returns the walkable 8-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 8-neighbors.
        """
        # This already checks for in-boundness
        returnList = self.neighbors_of_4(x, y)

        if x != 0 and y != 0 and self.is_cell_walkable(x - 1, y - 1):
            returnList.append((x - 1, y - 1))
        if x != self.info.width - 1 and y != 0 and self.is_cell_walkable(x + 1, y - 1):
            returnList.append((x + 1, y - 1))
        if y != self.info.height - 1 and x != 0 and self.is_cell_walkable(x - 1, y + 1):
            returnList.append((x - 1, y + 1))
        if x != self.info.width - 1 and y != self.info.height - 1 and self.is_cell_walkable(x + 1, y + 1):
            returnList.append((x + 1, y + 1))

        return returnList

    def is_cell_walkable(self, x, y):
        """
        A cell is walkable if all of these conditions are true:
        1. It is within the boundaries of the grid;
        2. It is free (not unknown, not occupied by an obstacle)
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [boolean]       True if the cell is walkable, False otherwise
        """

        ### REQUIRED CREDIT
        "if the x and y coordinates are out of bounds"
        return self.is_cell_in_bounds(x, y) and self.get_cell_value(x, y) < 0.196

    def is_within_threshold(self, pos1, pos2):
        # Are the two thresholds close enough to be considered the same centeroids?
        return self.euclidean_distance(pos1[0], pos1[1], pos2[0], pos2[1]) < 2

    def is_cell_in_bounds(self, x, y):
        return 0 <= x < (self.info.width - 1) and (self.info.height - 1) > y >= 0

    def get_cell_value(self, x, y):
        if not self.is_cell_in_bounds(x, y):
            raise IndexError("The cell index (%d, %d) is outside of this map (size %dx%d)" % (
                x, y, self.info.width, self.info.height))
        # rospy.loginfo(self.map.data[self.grid_to_index(x, y)])
        return self.map.data[self.grid_to_index(x, y)]

    def grid_to_index(self, x, y):
        """
        Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
        :param x [int] The cell X coordinate.
        :param y [int] The cell Y coordinate.
        :return  [int] The index.
        """
        return y * self.map.info.width + x

if __name__ == '__main__':
    PathPlanner().run()
