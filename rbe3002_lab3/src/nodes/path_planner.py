#!/usr/bin/env python

import math
import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from tf_conversions.posemath import transformations
from priority_queue import PriorityQueue

class PathPlanner:
    def __init__(self):
        """
        Class constructor
        """
        # REQUIRED CREDIT
        # Initialize the node and call it "path_planner"
        rospy.init_node("path_planner")
        # Create a new service called "plan_path" that accepts messages of
        # type GetPlan and calls self.plan_path() when a message is received
        self.pathService = rospy.Service('plan_path', GetPlan, self.plan_path)
        # Create a publisher for the C-space (the enlarged occupancy grid)
        # The topic is "/path_planner/cspace", the message type is GridCells
        self.pubCspace = rospy.Publisher("/path_planner/cspace", GridCells, queue_size=10)
        ## Create publishers for A* (expanded cells, frontier, ...)
        ## Choose a the topic names, the message type is GridCells
        # TODO
        ## Initialize the request counter
        # TODO
        ## Sleep to allow roscore to do some housekeeping
        rospy.sleep(1.0)
        rospy.loginfo("Path planner node ready")

    @staticmethod
    def grid_to_index(mapdata, x, y):
        """
        Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
        :param x [int] The cell X coordinate.
        :param y [int] The cell Y coordinate.
        :return  [int] The index.
        """
        return y * mapdata.info.width + x

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
    def grid_to_world(mapdata, x, y):
        """
        Transforms a cell coordinate in the occupancy grid into a world coordinate.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The cell X coordinate.
        :param y       [int]           The cell Y coordinate.
        :return        [Point]         The position in the world.
        """
        world_point = Point()

        world_point.x = (x + 0.5) * mapdata.info.resolution + mapdata.info.origin.position.x
        world_point.y = (y + 0.5) * mapdata.info.resolution + mapdata.info.origin.position.y

        return world_point

    @staticmethod
    def world_to_grid(mapdata, wp):
        """
        Transforms a world coordinate into a cell coordinate in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param wp      [Point]         The world coordinate.
        :return        [(int,int)]     The cell position as a tuple.
        """

        x = int((wp.x - mapdata.info.origin.position.x) / mapdata.info.resolution)
        y = int((wp.y - mapdata.info.origin.position.y) / mapdata.info.resolution)

        grid_coord = (x, y)

        return grid_coord

    @staticmethod
    def path_to_poses(mapdata, path):
        """
        Converts the given path into a list of PoseStamped.
        :param mapdata [OccupancyGrid] The map information.
        :param  path   [[(int,int)]]   The path as a list of tuples (cell coordinates).
        :return        [[PoseStamped]] The path as a list of PoseStamped (world coordinates).
        """
        ### REQUIRED CREDIT
        rospy.loginfo("converting path into a list of PoseStamped")
        posestamp_list = []
        for i in range(len(path)-1):
            yaw = PathPlanner.round_to_45(
                math.degrees(math.atan2((path[i + 1][1] - path[i][1]), (path[i + 1][0] - path[i][0]))))
            single_pose = PoseStamped()
            pos = PathPlanner.grid_to_world(mapdata, path[i][0], path[i][1])
            q = transformations.quaternion_from_euler(0, 0, yaw)
            # going back to ros quaternion
            orient = Quaternion(q[0], q[1], q[2], q[3])
            single_pose.pose.position = pos
            single_pose.pose.orientation = orient
            posestamp_list.append(single_pose)
        return posestamp_list

    @staticmethod
    def is_cell_walkable(mapdata, x, y):
        """
        A cell is walkable if all of these conditions are true:
        1. It is within the boundaries of the grid;
        2. It is free (not unknown, not occupied by an obstacle)
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [boolean]       True if the cell is walkable, False otherwise
        """
        walkable = False

        ### REQUIRED CREDIT
        "if the x and y coordinates are out of bounds"
        if 0 <= x < mapdata.info.width - 1 and mapdata.info.height - 1 > y >= 0:
            "if the data in the cell is less than 0.196(threshold of the free cell)"
            if mapdata.data[PathPlanner.grid_to_index(mapdata, x, y)] < 0.196:
                return True
        return False

    @staticmethod
    def neighbors_of_4(mapdata, x, y):
        """
        Returns the walkable 4-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 4-neighbors.
        """

        if x < 0 | x > mapdata.info.width - 1 | y < 0 | y > mapdata.info.height - 1:
            raise ValueError("input cell is not within the bounds of the map")

        returnList = []

        if x != 0:
            if PathPlanner.is_cell_walkable(mapdata, x - 1, y):
                returnList.append((x - 1, y))
        if x != mapdata.info.width - 1:
            if PathPlanner.is_cell_walkable(mapdata, x + 1, y):
                returnList.append((x + 1, y))
        if y != 0:
            if PathPlanner.is_cell_walkable(mapdata, x, y - 1):
                returnList.append((x, y - 1))
        if y != mapdata.info.height - 1:
            if PathPlanner.is_cell_walkable(mapdata, x, y + 1):
                returnList.append((x, y + 1))

        return returnList

    @staticmethod
    def neighbors_of_8(mapdata, x, y):
        """
        Returns the walkable 8-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 8-neighbors.
        """
        # This already checks for in-boundness
        returnList = PathPlanner.neighbors_of_4(mapdata, x, y)

        if x != 0 and y != 0:
            if PathPlanner.is_cell_walkable(mapdata, x - 1, y - 1):
                returnList.append((x - 1, y - 1))
        if x != mapdata.info.width - 1 and y != 0:
            if PathPlanner.is_cell_walkable(mapdata, x + 1, y - 1):
                returnList.append((x + 1, y - 1))
        if y != mapdata.info.height - 1 and x != 0:
            if PathPlanner.is_cell_walkable(mapdata, x - 1, y + 1):
                returnList.append((x - 1, y - 1))
        if x != mapdata.info.width - 1 and y != mapdata.info.height - 1:
            if PathPlanner.is_cell_walkable(mapdata, x + 1, y + 1):
                returnList.append((x + 1, y + 1))

        return returnList

    @staticmethod
    def request_map():
        """
        Requests the map from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
                                None in case of error.
        """
        ### REQUIRED CREDIT
        rospy.loginfo("Requesting the map")
        try:
            map_server = rospy.ServiceProxy('static_map', GetMap)
            mapObj = map_server()
            x = mapObj.map.info.height
            y = mapObj.map.info.width
            mapObj.map.info.height = y
            mapObj.map.info.width = x
            return mapObj.map
        except rospy.ServiceException, e:
            return None

    @staticmethod
    def force_inbound(mapdata, curr_x, curr_y):
        maxY = mapdata.info.height - 1
        maxX = mapdata.info.width - 1
        minX = 0
        minY = 0

        newX = max(minX, min(curr_x, maxX))
        newY = max(minY, min(curr_y, maxY))

        return newX, newY

    def calc_cspace(self, mapdata, padding):
        """
        Calculates the C-Space, i.e., makes the obstacles in the map thicker.
        Publishes the list of cells that were added to the original map.
        :param mapdata [OccupancyGrid] The map data.
        :param padding [int]           The number of cells around the obstacles.
        :return        [OccupancyGrid] The C-Space.
        """
        OBSTACLE_THRESH = 90
        rospy.loginfo("Calculating C-Space")

        paddedArray = list(mapdata.data)

        ## Go through each cell in the occupancy grid
        for y in range(mapdata.info.height):
            for x in range(mapdata.info.width):
                ## Inflate the obstacles where necessary
                if mapdata.data[PathPlanner.grid_to_index(mapdata, x, y)] > OBSTACLE_THRESH:

                    for x2 in range(mapdata.info.height - padding, mapdata.info.height + padding):
                        for y2 in range(mapdata.info.width - padding, mapdata.info.width + padding):
                            x3, y3 = PathPlanner.force_inbound(mapdata, x2, y2)
                            paddedArray[self.grid_to_index(mapdata, x3, y3)] = 100
        paddedArray = tuple(paddedArray)
        gridCellsList = []

        for x in range(mapdata.info.height):
            for y in range(mapdata.info.width):
                ## Inflate the obstacles where necessary
                if paddedArray[PathPlanner.grid_to_index(mapdata, y, x)] > OBSTACLE_THRESH:
                    world_point = PathPlanner.grid_to_world(mapdata, x, y)
                    gridCellsList.append(world_point)

        ## Create a GridCells message and publish it
        msg = GridCells()
        msg.cell_width = mapdata.info.resolution
        msg.cell_height = mapdata.info.resolution
        msg.cells = gridCellsList
        msg.header = mapdata.header
        self.pubCspace.publish(msg)
        rospy.loginfo("GridCells: " + str(msg))
        mapdata.data = paddedArray

        return mapdata

    def a_star(self, mapdata, start, goal):
        ### REQUIRED CREDIT
        rospy.loginfo("Executing A* from (%d,%d) to (%d,%d)" % (start[0], start[1], goal[0], goal[1]))
        frontier = PriorityQueue()
        frontier.put(start, 0)

        came_from = {}
        came_from[start] = None
        cost_so_far = {}
        cost_so_far[start] = 0

        while not frontier.empty():
            current = frontier.get()

            if current == goal:
                break

            for neighbour in PathPlanner.neighbors_of_4(mapdata, current[0], current[1]):
                rospy.loginfo(str(type(cost_so_far[current])))
                rospy.loginfo(str(cost_so_far[current]))
                rospy.loginfo(str(current))
                a = cost_so_far[current]
                b = PathPlanner.euclidean_distance(current[0],current[1],neighbour[0],neighbour[1])
                rospy.loginfo(a)
                rospy.loginfo(b)
                #new_cost = cost_so_far[current] + PathPlanner.euclidean_distance(current[0],current[1],next[0],next[1])
                new_cost = a + b
                if neighbour not in cost_so_far or new_cost < cost_so_far[neighbour]:
                    cost_so_far[neighbour] = new_cost
                    priority = new_cost + PathPlanner.euclidean_distance(neighbour[0],neighbour[1],goal[0],goal[1])
                    frontier.put(neighbour,priority)
                    came_from[neighbour] = current

        currPos = goal
        finalPath = []
        finalPath.append(goal)
        while currPos != start:
            currPos = came_from[currPos]
            finalPath.append(currPos)

        finalPath.reverse()

        return finalPath

        frontier = priority_queue
        frontier.put(start, 0)
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0

        while not frontier.empty():
            current = frontier.get()
            if current == goal:
                break

            for next_n in graph.neighbours(current):
                new_cost = cost_so_far[current] + graph.cost(current, next_n)
                if next_n not in cost_so_far or new_cost < cost_so_far[next_n]:
                    cost_so_far[next_n] = new_cost
                    priority = new_cost + heuristic(goal, next_n)
                    frontier.put(next_n, priority)
                    came_from[next_n] = current





    @staticmethod
    def optimize_path(path):
        """
        Optimizes the path, removing unnecessary intermediate nodes.
        :param path [[(x,y)]] The path as a list of tuples (grid coordinates)
        :return     [[(x,y)]] The optimized path as a list of tuples (grid coordinates)
        """
        ### EXTRA CREDIT
        rospy.loginfo("Optimizing path")
        curr_heading = 0
        last_heading = 0

        pathCopy = path

        rospy.loginfo("Original Path Length: " + str(len(path)))

        #Divide lenght by two since leng returns, counting both X and Y
        for i in range(1, len(path)/2 - 1):
            rospy.loginfo(i)
            curr_heading = PathPlanner.round_to_45(
                math.degrees(math.atan2((path[i + 1][1] - path[i][1]), (path[i + 1][0] - path[i][0]))))
            last_heading = PathPlanner.round_to_45(
                math.degrees(math.atan2((path[i][1] - path[i - 1][1]), (path[i][0] - path[i - 1][0]))))

            if curr_heading == last_heading:
                pathCopy.pop(i)

        return pathCopy

    @staticmethod
    def round_to_45(value):
        """
        Round to the nearest 45 degree increment
        param value [double?] The value to be rounded in degrees
        """
        return round(value / 45.0) * 45.0

    @staticmethod
    def path_to_message(mapdata, path):
        """
        Takes a path on the grid and returns a Path message.
        :param path [[(int,int)]] The path on the grid (a list of tuples)
        :return     [Path]        A Path message (the coordinates are expressed in the world)
        """
        ### REQUIRED CREDIT
        rospy.loginfo("Returning a Path message")
        path_message = Path()
        path_message.poses = PathPlanner.path_to_poses(mapdata, path)
        path_message.poses.pop(0)
        return path_message

    def plan_path(self, msg):
        """
        Plans a path between the start and goal locations in the requested.
        Internally uses A* to plan the optimal path.
        :param req
        """
        ## Request the map
        ## In case of error, return an empty path
        mapdata = PathPlanner.request_map()
        # if mapdata is None:
        #     return Path()
        # ## Calculate the C-space and publish it
        rospy.wait_for_service('static_map', timeout=None)
        cspacedata = self.calc_cspace(mapdata, 1)
        rospy.sleep(0.10)
        # ## Execute A*
        start = PathPlanner.world_to_grid(mapdata, msg.start.pose.position)
        goal  = PathPlanner.world_to_grid(mapdata, msg.goal.pose.position)
        path  = self.a_star(cspacedata, start, goal)
        rospy.loginfo("a_star output: " + str(path))

        # ## Optimize waypoints
        waypoints = PathPlanner.optimize_path(path)
        rospy.loginfo("Optimized Waypoints: " + str(waypoints))
        # ## Return a Path message, this line can be erased and returned directly after debug
        return_obj = PathPlanner.path_to_message(mapdata, waypoints)
        rospy.loginfo("path_to_message output: " + str(return_obj))
        return return_obj

    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        # mapdata = PathPlanner.request_map()
        # self.calc_cspace(mapdata,1)
        rospy.spin()



if __name__ == '__main__':
    PathPlanner().run()
