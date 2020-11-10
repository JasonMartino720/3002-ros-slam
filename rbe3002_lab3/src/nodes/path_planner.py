#!/usr/bin/env python

import math
import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped
from tf_conversions.transformations import quaternion_from_euler



class PathPlanner:



    def __init__(self):
        """
        Class constructor
        """
        ### REQUIRED CREDIT
        ## Initialize the node and call it "path_planner"
        rospy.init_node("path_planner")
        ## Create a new service called "plan_path" that accepts messages of
        ## type GetPlan and calls self.plan_path() when a message is received
        self.pathService = rospy.Service('plan_path',GetPlan,self.plan_path)
        ## Create a publisher for the C-space (the enlarged occupancy grid)
        ## The topic is "/path_planner/cspace", the message type is GridCells
        self.pubCspace = rospy.Publisher("/path_planner/cspace",GridCells, queue_size = 10)
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
        return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))



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
        yaw = 0

        #still need to calculate yaw

        for i in range(len(path)):
            single_pose = PoseStamped()
            pos = self.grid_to_world(mapdata, path[i][0], path[i][1])
            q = quaternion_from_euler(0, 0, yaw)
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
        if  0 <= x < mapdata.info.width-1 and mapdata.info.height-1 > y >= 0:
            "if the data in the cell is less than 0.196(threshold of the free cell)"
            if mapdata.data[self.
            self.grid_to_index(mapdata, x, y)] < 0.196:
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

        if(x < 0 || x > mapdata.info.width-1 || y < 0 || y > mapdata.info.height-1):
            raise ValueError("input cell is not within the bounds of the map")

        returnList = []

        if x != 0:
            if is_cell_walkable(mapdata,x-1,y):
                returnList.append(x-1,y)
        if x != mapdata.info.width-1:
            if is_cell_walkable(mapdata,x+1,y):
                returnList.append(x+1,y)
        if y != 0:
            if is_cell_walkable(mapdata,x,y-1):
                returnList.append(x,y-1)
        if y != mapdata.info.height-1:
            if is_cell_walkable(mapdata,x,y+1):
                returnList.append(x,y+1)

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
        #This already checks for in-boundness
        returnList = neighbors_of_4(mapdata, x, y)

        if x != 0 and y != 0:
            if is_cell_walkable(mapdata,x-1,y-1):
                returnList.append(x-1,y-1)
        if x != mapdata.info.width-1 and y != 0:
            if is_cell_walkable(mapdata,x+1,y-1):
                returnList.append(x+1,y-1)
        if y != mapdata.info.height-1 and x != 0:
            if is_cell_walkable(mapdata,x-1,y+1):
                returnList.append(x-1,y-1)
        if x != mapdata.info.width-1 and y != mapdata.info.height-1:
            if is_cell_walkable(mapdata,x+1,y+1):
                returnList.append(x+1,y+1)

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
    def force_inbound(self,mapdata,curr_x,curr_y):
        maxY = mapdata.info.height-1
        maxX = mapdata.info.width-1
        minX = 0
        minY = 0

        newX = max(minX,min(curr_x, maxX))
        newY = max(minY,min(curr_y, maxY))

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

        paddedArray = mapdata.data

        ## Go through each cell in the occupancy grid
        for y in range(mapdata.info.height):
            for x in range(mapdata.info.width):
                ## Inflate the obstacles where necessary
                if mapdata.data[self.grid_to_index(mapdata,y,x)] > OBSTACLE_THRESH:

                    for y2 in range(mapdata.info.height-padding,mapdata.info.height-padding):
                        for x2 in range(mapdata.info.width-padding,mapdata.info.width-padding):
                            x3, y3 = self.force_inbound(mapdata,x2,y2)
                            paddedArray[self.grid_to_index(mapdata,x3,y3)] = 100

        gridCellsList = []

        for y in range(mapdata.info.height):
            for x in range(mapdata.info.width):
                ## Inflate the obstacles where necessary
                if paddedArray[self.grid_to_index(mapdata,y,x)] > OBSTACLE_THRESH:
                    world_point = self.grid_to_world(mapdata,x,y)
                    gridCellsList.append(world_point)

        ## Create a GridCells message and publish it
        msg = GridCells()
        msg.cell_width = 0.03
        msg.cell_height = 0.03
        msg.cells = gridCellsList
        msg.header.frame_id = "map"
        self.pubCspace.publish(msg)

        ## Return the C-space
        # for i, cellValue in enumerate(paddedArray):
        #     mapdata.data[i] = cellValue
        mapdata.data = paddedArray

        return mapdata



    def a_star(self, mapdata, start, goal):
        ### REQUIRED CREDIT
        rospy.loginfo("Executing A* from (%d,%d) to (%d,%d)" % (start[0], start[1], goal[0], goal[1]))



    @staticmethod
    def optimize_path(path):
        """
        Optimizes the path, removing unnecessary intermediate nodes.
        :param path [[(x,y)]] The path as a list of tuples (grid coordinates)
        :return     [[(x,y)]] The optimized path as a list of tuples (grid coordinates)
        """
        ### EXTRA CREDIT
        rospy.loginfo("Optimizing path")



    def path_to_message(self, mapdata, path):
        """
        Takes a path on the grid and returns a Path message.
        :param path [[(int,int)]] The path on the grid (a list of tuples)
        :return     [Path]        A Path message (the coordinates are expressed in the world)
        """
        ### REQUIRED CREDIT
        rospy.loginfo("Returning a Path message")
        path_message = Path()
        pose_array = []
        yaw = 0;
        for i in range(len(path)):
            pose_message = PoseStamped()
            point = self.grid_to_world(mapdata, path[i][0], path[i][1])
            #calc yaw using round(inverseTan(angle between i and i+1))
            q = quaternion_from_euler(0, 0, yaw)
            orientation = Quaternion(q[0], q[1], q[2], q[3])
            pose_message.pose.position = point
            pose_message.pose.orientation = orientation
            pose_array.append(pose_message)
        path_message.poses = pose_array
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
        cspacedata = self.calc_cspace(mapdata, 1)
        # ## Execute A*
        # start = PathPlanner.world_to_grid(mapdata, msg.start.pose.position)
        # goal  = PathPlanner.world_to_grid(mapdata, msg.goal.pose.position)
        # path  = self.a_star(cspacedata, start, goal)
        # ## Optimize waypoints
        # waypoints = PathPlanner.optimize_path(path)
        # ## Return a Path message
        returnObj = GetPlan()
        waypoints = 0
        returnObj.plan = self.path_to_message(mapdata, waypoints)
        returnObj.start = PoseStamped()
        returnObj.goal = PoseStamped()
        returnObj.tolerance = 0.1
        return returnObj.plan



    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        rospy.spin()



if __name__ == '__main__':
    PathPlanner().run()
