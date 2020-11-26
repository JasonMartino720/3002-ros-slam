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
        self.pubVisited = rospy.Publisher("/path_planner/visited", GridCells, queue_size=10)
        ## Choose a the topic names, the message type is GridCells

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
        # rospy.loginfo("mapdata.info: " + str(mapdata.info))
        # rospy.loginfo("input for grid_to_world: " + str(x) + ", " + str(y))
        # rospy.loginfo("grid_to_world x, y: " + str(world_point.x) + ", " + str(world_point.y))
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
        # rospy.loginfo("converting path into a list of PoseStamped")
        posestamp_list = []
        for i in range(len(path)):
            yaw = 0
            if i < len(path)-1:
                yaw = PathPlanner.round_to_45(
                    math.degrees(math.atan2((path[i + 1][1] - path[i][1]), (path[i + 1][0] - path[i][0]))))
            single_pose = PoseStamped()
            pos = PathPlanner.grid_to_world(mapdata, path[i][0], path[i][1])
            q = transformations.quaternion_from_euler(0, 0, yaw)
            # going back to ros quaternion
            orient = Quaternion(q[0], q[1], q[2], q[3])
            single_pose.pose.position = pos
            single_pose.pose.orientation = orient
            single_pose.header = mapdata.header
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
                returnList.append((x - 1, y + 1))
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
            return map_server().map
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
        :return        [OccupancyGrid] The C-Space.self.pubCspace.publish(msg)
        """
        OBSTACLE_THRESH = 90
        rospy.loginfo("Calculating C-Space")

        paddedArray = list(mapdata.data)

        ## Go through each cell in the occupancy grid
        for x in range(mapdata.info.height):
            for y in range(mapdata.info.width):
                ## Inflate the obstacles where necessary
                if mapdata.data[PathPlanner.grid_to_index(mapdata, x, y)] > OBSTACLE_THRESH:
                    paddedArray[self.grid_to_index(mapdata, x, y)] = 100
                    for neighbor in PathPlanner.neighbors_of_8(mapdata, x, y):
                        x3, y3 = PathPlanner.force_inbound(mapdata, neighbor[0], neighbor[1])
                        paddedArray[self.grid_to_index(mapdata, x3, y3)] = 100
        paddedArray = tuple(paddedArray)
        gridCellsList = []

        for x in range(mapdata.info.height):
            for y in range(mapdata.info.width):
                ## Inflate the obstacles where necessary
                if paddedArray[PathPlanner.grid_to_index(mapdata, x, y)] > OBSTACLE_THRESH:
                    world_point = PathPlanner.grid_to_world(mapdata, x, y)
                    gridCellsList.append(world_point)

        ## Create a GridCells message and publish it
        msg = GridCells()
        msg.cell_width = mapdata.info.resolution
        msg.cell_height = mapdata.info.resolution
        msg.cells = gridCellsList
        msg.header = mapdata.header
        self.pubCspace.publish(msg)
        mapdata.data = paddedArray

        return mapdata

    def a_star(self, mapdata, start, goal):
        ### REQUIRED CREDIT
        # rospy.loginfo("Executing A* from (%d,%d) to (%d,%d)" % (start[0], start[1], goal[0], goal[1]))
        frontier = PriorityQueue()
        frontier.put(start, 0)

        came_from = {}
        came_from[start] = None
        cost_so_far = {}
        cost_so_far[start] = 0
        #Path Visualization
        visualize_path = []

        while not frontier.empty():
            current = frontier.get()
            #Path Visualization
            visualize_path.append(current)
            rospy.loginfo("Adding %f %f to visited list" %(current[0], current[1]))

            if current == goal:
                break

            for neighbour in PathPlanner.neighbors_of_8(mapdata, current[0], current[1]):
                # rospy.loginfo(str(type(cost_so_far[current])))
                # rospy.loginfo(str(cost_so_far[current]))
                # rospy.loginfo(str(current))
                a = cost_so_far[current]
                b = PathPlanner.euclidean_distance(current[0],current[1],neighbour[0],neighbour[1])
                # rospy.loginfo(a)
                # rospy.loginfo(b)
                new_cost = cost_so_far[current] + PathPlanner.euclidean_distance(current[0],current[1],neighbour[0],neighbour[1])
                new_cost = a + b
                if neighbour not in cost_so_far or new_cost < cost_so_far[neighbour]:
                    cost_so_far[neighbour] = new_cost
                    priority = new_cost + PathPlanner.euclidean_distance(neighbour[0],neighbour[1],goal[0],goal[1])
                    frontier.put(neighbour,priority)
                    came_from[neighbour] = current

                # Path Visualization
                visitedCells = []
                for cell in visualize_path:
                    world_point = PathPlanner.grid_to_world(mapdata, cell[0], cell[1])
                    # rospy.loginfo("Converting %f %f to visited list" % (current[0], current[1]))
                    visitedCells.append(world_point)

                # Path Visualization
                ## Create a GridCells message and publish it
                # message for visualizing cells in the visualize_path list
                pvis = GridCells()
                pvis.cell_width = mapdata.info.resolution
                pvis.cell_height = mapdata.info.resolution
                pvis.cells = visitedCells
                pvis.header = mapdata.header
                self.pubVisited.publish(pvis)
                rospy.loginfo(pvis)
                rospy.loginfo("Published this to /visited")

        visualize_path.append(goal)
        # Path Visualization
        visitedCells = []
        for cell in visualize_path:
            world_point = PathPlanner.grid_to_world(mapdata, cell[0], cell[1])
            # rospy.loginfo("Converting %f %f to visited list" % (current[0], current[1]))
            visitedCells.append(world_point)

        # Path Visualization
        ## Create a GridCells message and publish it
        # message for visualizing cells in the visualize_path list
        pvis = GridCells()
        pvis.cell_width = mapdata.info.resolution
        pvis.cell_height = mapdata.info.resolution
        pvis.cells = visitedCells
        pvis.header = mapdata.header
        self.pubVisited.publish(pvis)
        rospy.loginfo(pvis)
        rospy.loginfo("Published this to /visited")

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

        pathCopy = path

        rospy.loginfo("Original Path Length: " + str(len(path)))
        for i in range(1, len(path)-1):
            rospy.loginfo("Current Point: " + str(i))
            curr_heading = PathPlanner.round_to_45(math.degrees(math.atan2((path[i+1][1] - path[i][1]), (path[i+1][0] - path[i][0]))))
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
        rospy.loginfo("The path is: " + str(path))
        path_message.poses = PathPlanner.path_to_poses(mapdata, path)
        path_message.header = mapdata.header
        rospy.loginfo("path_message: " + str(path_message))
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
        if mapdata is None:
            return Path()

        # ## Calculate the C-space and publish it
        rospy.wait_for_service('static_map', timeout=None)
        cspacedata = self.calc_cspace(mapdata, 1)
        rospy.sleep(0.10)
        # ## Execute A*
        start = PathPlanner.world_to_grid(mapdata, msg.start.pose.position)
        goal  = PathPlanner.world_to_grid(mapdata, msg.goal.pose.position)
        path  = self.a_star(cspacedata, start, goal)
        #rospy.loginfo("a_star output: " + str(path))

        # ## Optimize waypoints
        waypoints = PathPlanner.optimize_path(path)
        #rospy.loginfo("Optimized Waypoints: " + str(waypoints))
        # ## Return a Path message, this line can be erased and returned directly after debug
        return_obj = PathPlanner.path_to_message(mapdata, waypoints)
        #rospy.loginfo("path_to_message output: " + str(return_obj))
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
