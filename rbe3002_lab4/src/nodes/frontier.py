#!/usr/bin/env python2

import rospy
import random

from geometry_msgs.msg import Point
from nav_msgs.msg import GridCells, OccupancyGrid
from nav_msgs.srv import GetMap
from rbe3002_lab4.srv._frontiers import frontiers, frontiersResponse, frontiersRequest


class Frontier:

    def __init__(self):
        rospy.loginfo("Started Frontier node")
        rospy.init_node("frontier")

        rospy.Subscriber("/map", OccupancyGrid, self.update_map)

        # Create a publisher for the C-space (the enlarged occupancy grid)
        # The topic is "/path_planner/cspace", the message type is GridCells
        self.pubCspace = rospy.Publisher("/cspace", GridCells, queue_size=10)

        rospy.sleep(1.0)

        #Do we have to make a custom message or can we use a premade one?
        self.frontierService = rospy.Service('frontierTopic', frontiers, self.return_frontier)

        self.cspaceService = rospy.Service('cspace', GetMap, self.calc_cspace())

        rospy.sleep(1.0)
        rospy.loginfo("Frontier node ready")

    def update_map(self, occupancyGrid):
        self.map = occupancyGrid


    #checks if the cell is an edge cell
    def is_edge_cell(self, cell):
        if cell == 100:
            return True
        else:
            return False

    #checks if the cell has been assigned previously
    def is_not_assigned(self, cell, cluster_list):
        for cluster in cluster_list:
            for coord in cluster:
                if cell == coord:
                    return True
                    break
                else:
                    return False

    #recursive exploration, when a cell is edged and not assigned, we group all the connected cells as one cluster.
    def recursive_explore(self, point_to_explore, list_to_build):
        if self.is_edge_cell(point_to_explore):
            list_to_build.append(point_to_explore)
            # I am not sure if this is the correct way of assigning a starting point for neighbors_of_8 to use
            starting_point = point_to_explore
            for neighbor in self.neighbors_of_8(starting_point):
                self.recursive_explore(neighbor, list_to_build)

    #dilate and erode the grid with a given number of times to dilate and erode
    def dilated_and_eroded_grid(self, set_number):
        set_num = set_number
        Obstacle_thresh = 90

        # original array
        grid_array = self.map.data
        coppy_array = list(grid_array)
        # Dilationthe cell is not assigned
        for count in range(set_num):
            for x in range(self.map.info.height):
                for y in range(self.map.info.width):
                    ## Inflate the obstacles where necessary
                    if self.map.data[self.grid_to_index(x, y)] >= Obstacle_thresh:
                        coppy_array[self.grid_to_index(x, y)] = 100
                        for neighbor in self.neighbors_of_8(x, y):
                            newX, newY = self.force_inbound(neighbor[0], neighbor[1])
                            coppy_array[self.grid_to_index(newX, newY)] = 100
            grid_array = coppy_array

        # Erosion
        for count in range(set_num):
            for x in range(self.map.info.height):
                for y in range(self.map.info.width):
                    ## Inflate the obstacles where necessary
                    if self.map.data[self.grid_to_index(x, y)] >= Obstacle_thresh:
                        coppy_array[self.grid_to_index(x, y)] = 100
                        for neighbor in self.neighbors_of_8(x, y):
                            newX, newY = self.force_inbound(neighbor[0], neighbor[1])
                            coppy_array[self.grid_to_index(newX, newY)] = 0
            grid_array = coppy_array
        return grid_array

    def return_frontier(self, msg):

        egde_occupancy_grid = self.get_frontier_cells()
        #Josh - this is the occupancy grid with all the edge with the value of 100

        #Kohmei, I believe here is where the calls to josh's new clustering functions will be, I don't have them yet so
        # I'm not sure what comes next. I assume the clustering algorithm output will already be a list of frontiers, does it include sizes
        # or is more computation necessary here?

        # #Jason, I think we discuessed that we are sending the raw cluster data in this format:
        # List(
        #     List(coordinate1,cord2,cord3 .... cordN) #Cluster 1
        #     List(coordinate1,cord2,cord3 .... cordN) #Cluster 2
        #     List(coordinate1,cord2,cord3 .... cordN) #Cluster 3
        # ) #coordinate is a tuple (x, y)the cell is not assigned
        # So we don't need any more processing, but idea is that the clustering function will be written here

        #Josh, The rest is for you:
        # Again, you are given a occupancy grid with cells that are either 0 or 100
        # 0 means the cell is not a frontier (egde) cell, 100 means it is
        #You want to output a list of lists of corrdinates (look above)
        #Where each element in the outer list is one cluster

        #Given the occupanyGrid with edges marked as 100, dilate and erode
            # egde_occupancy_grid = dilate(egde_occupancy_grid)
            # egde_occupancy_grid = erode(egde_occupancy_grid)
        # (Very similar to C-space so please write these two functions)

        #Start a loop here that runs until all cells have been assigned
        # (hint: how do you know if all cells have been assinged? Count the number of cells you have to assign and
        # keep track of how many you have assigned so far)



        # counting cells that need to be assigned
        assigned_so_far = 0
        need_assignment = 0
        for cells in self.dilated_and_eroded_grid(3):
            if cells == 100:
                need_assignment += 1

        #frontier_list is a list of clusters
        frontier_list = list()

        while (need_assignment > 0):
            #Now pick a random point
            #Find the height and widtht of the occupancy grid
                #height = self.map.info....
                #width = ...
            occ_height = self.map.info.height
            occ_width = self.map.info.width
            #use the max values for x and y to find a random value in grid space
            x_random_value = random.randint(0, occ_width)
            y_random_value = random.randint(0, occ_height)


            #First check to make sure this is a edge cell
            #Then Check if this random cell (x_random_value, y_random_value) is already assinged one cluster or not
            #(hint: loop through the return list you have and see if any coordinates match your random value)
            #Ok so now you have a random cell that has not been assigned yet.
            #Use reccursion (or not) to go through each of the neighboors and see if they are a edge cell (with value 100)
            #or not. If they are a edge, check the neighbors for this cell as well.
            #Each time you see a a new neighbor that is a edge cell, add it to the cluster you are currently building
            #Repeat the reccurtion until you have no more neighbors that are edges left.
            #End of the loop that find a new random cell
            #This should loop until you have catergorized every cell
            random_point = (occ_height, occ_width)

            if self.is_not_assigned(random_point, frontier_list) and self.is_edge_cell(random_point):
                # cluster is a list of tuples that represents points/cells that are unassigned and edged
                cluster = list()
                cluster = self.recursive_explore(random_point, cluster)
                frontier_list.append(cluster)

            need_assignment -= 1

        retVal = frontiersResponse(frontier_list)
        return retVal


    def get_frontier_cells(self):
        #Assuming that the map node has been split off into it's own node already
        #Assumign update Map has already given us the newest map

        OBSTACLE_THRESH = 90
        rospy.loginfo("Calculating edge")

        frontier_map = self.map

        ## Go through each cell in the occupancy grid
        for x in range(self.map.info.height):
            for y in range(self.map.info.width):
                #These helper functions have to be imported or the two classes "Frontier" and "map" have to be combined
                if self.is_cell_unknown(x, y):
                    #This function is written in map.py!!!!
                    # is_cell_unknown()
                    # is_cell_wall()
                    # is_cell_walkable()
                    #If any neighbor of a unknown cell is walkable, make the unknow a frontier
                    for neighbor in self.neighbors_of_4(x, y):
                        if self.is_cell_walkable(neighbor[0],neighbor[1]):
                            frontier_map.data[self.grid_to_index(x, y)] = 100

        #This returns a occupancy grid with the edge cells only
        return frontier_map

    def force_inbound(self, curr_x, curr_y):
        new_x = max(0, min(curr_x, self.map.info.width - 1))
        new_y = max(0, min(curr_y, self.map.info.height - 1))

        return new_x, new_y

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
        if x != self.map.info.width - 1 and self.is_cell_walkable(x + 1, y):
            return_list.append((x + 1, y))
        if y != 0 and self.is_cell_walkable(x, y - 1):
            return_list.append((x, y - 1))
        if y != self.map.info.height - 1 and self.is_cell_walkable(x, y + 1):
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
        if x != self.map.info.width - 1 and y != 0 and self.is_cell_walkable(x + 1, y - 1):
            returnList.append((x + 1, y - 1))
        if y != self.map.info.height - 1 and x != 0 and self.is_cell_walkable(x - 1, y + 1):
            returnList.append((x - 1, y + 1))
        if x != self.map.info.width - 1 and y != self.map.info.height - 1 and self.is_cell_walkable(x + 1, y + 1):
            returnList.append((x + 1, y + 1))

        return returnList

    def is_cell_in_bounds(self, x, y):
        return 0 <= x < self.map.info.width - 1 and self.map.info.height - 1 > y >= 0

    def grid_to_index(self, x, y):
        """
        Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
        :param x [int] The cell X coordinate.
        :param y [int] The cell Y coordinate.
        :return  [int] The index.
        """
        return y * self.map.info.width + x

    def get_cell_value(self, x, y):
        if not self.is_cell_in_bounds(x, y):
            raise IndexError("The cell index (%d, %d) is outside of this map (size %dx%d)" % (
                x, y, self.map.info.width, self.map.info.height))
        return self.map.data[self.grid_to_index(x, y)]

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

    def is_cell_wall(self, x, y):
        return self.get_cell_value(x, y) > 0.55

    def is_cell_unknown(self, x, y):
        return self.is_cell_in_bounds(x, y) and self.get_cell_value(x, y) == 0.5

    def calc_cspace(self):
        """
        Published GridCells and returns (to the service) a OccupancyGrid
        """
        OBSTACLE_THRESH = 90
        rospy.loginfo("Calculating C-Space")

        paddedArray = list(self.map.data)

        ## Go through each cell in the occupancy grid
        for x in range(self.map.info.height):
            for y in range(self.map.info.width):
                ## Inflate the obstacles where necessary
                if self.map.data[self.grid_to_index(x, y)] > OBSTACLE_THRESH:
                    paddedArray[self.grid_to_index(x, y)] = 100
                    for neighbor in self.neighbors_of_8(x, y):
                        x3, y3 = self.force_inbound(neighbor[0], neighbor[1])
                        paddedArray[self.grid_to_index(x3, y3)] = 100

        paddedArray = tuple(paddedArray)
        gridCellsList = []

        for x in range(self.map.info.height):
            for y in range(self.map.info.width):
                ## Inflate the obstacles where necessary
                if paddedArray[self.grid_to_index(x, y)] > OBSTACLE_THRESH:
                    world_point = self.grid_to_world(x, y)
                    gridCellsList.append(world_point)

        ## Create a GridCells message and publish it
        msg = GridCells()
        msg.cell_width = self.map.info.resolution
        msg.cell_height = self.map.info.resolution
        msg.cells = gridCellsList
        msg.header = self.map.header
        self.pubCspace.publish(msg)


        return paddedArray

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

    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        # mapdata = PathPlanner.request_map()
        # self.calc_cspace(mapdata,1)
        rospy.loginfo("Frontier is running")
        rospy.spin()

if __name__ == '__main__':
    Frontier().run()