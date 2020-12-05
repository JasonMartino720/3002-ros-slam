#!/usr/bin/env python2

import rospy
from nav_msgs.msg import GridCells
from nav_msgs.srv import GetMap
from sklearn import metrics
from sklearn.cluster import KMeans


class Frontier:

    def __init__(self):
        rospy.loginfo("Started Frontier node")
        rospy.init_node("frontier")

        #Do we have to make a custom message or can we use a premade one?
        self.frontierService = rospy.Service('frontier_list', list, self.return_frontier)

        rospy.sleep(1.0)
        rospy.loginfo("Frontier node ready")

        self.refresh_map()
        self.map = None


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
        # ) #coordinate is a tuple (x, y)
        # So we don't need any more processing, but idea is that the clustering function will be written here

        #Josh, The rest is for you:
        # Again, you are given a occupancy grid with cells that are either 0 or 100
        # 0 means the cell is not a frontier (egde) cell, 100 means it is
        #You want to output a list of lists or corrdinates (look above)
        #Where each element in the outer list is one cluster

        #Given the occupanyGrid with edges marked as 100, dilate and erode
            # egde_occupancy_grid = dilate(egde_occupancy_grid)
            # egde_occupancy_grid = erode(egde_occupancy_grid)
        # (Very similar to C-space so please write these two functions)

        #Start a loop here that runs until all cells have been assigned
        # (hint: how do you know if all cells have been assinged? Count the number of cells you have to assign and
        # keep track of how many you have assigned so far)
        #Now pick a random point
        #Find the height and widtht of the occupany grid
            #height = self.map.info....
            #width = ...

        #use the max values for x and y to find a random value in grid space
            #x_random_value = ...
            #y_random_value = ... (hint: use randint(min,max))

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

        frontier_list = [()]

        return frontier_list

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
                if is_cell_unknown(x, y):
                    #This function is written in map.py!!!!
                    # is_cell_unknown()
                    # is_cell_wall()
                    # is_cell_walkable()
                    #If any neighbor of a unknown cell is walkable, make the unknow a frontier
                    for neighbor in self.neighbors_of_4(x, y):
                        if is_cell_walkable(neighbor[0],neighbor[1]):
                            frontier_map.data[grid_to_index(x, y)] = 100

        #This returns a occupancy grid with the edge cells only
        return frontier_map

    def refresh_map(self):
        occupancy_grid = self.request_GMap() #Changed the call here, all good?
        if occupancy_grid is None:
            raise TypeError("Tried to create Map object but got None for occupancy_grid")
        self.header = occupancy_grid.header
        self.info = occupancy_grid.info
        self.data = occupancy_grid.data
        self.calc_cspace()

    def request_map(self):
        """
        Requests the map from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
                                None in case of error.
        """
        ### REQUIRED CREDIT
        rospy.loginfo("Requesting the map")
        rospy.wait_for_service('static_map', timeout=None)
        try:
            map_server = rospy.ServiceProxy('static_map', GetMap)
            return map_server().map
        except rospy.ServiceException, e:
            return None

    def request_GMap(self):
        """
        Requests the map from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
                                None in case of error.
        """
        ### REQUIRED CREDIT
        rospy.loginfo("Requesting the map")
        rospy.wait_for_service('dynamic_map', timeout=None)
        try:
            map_server = rospy.ServiceProxy('dynamic_map', GetMap)
            return map_server().map
        except rospy.ServiceException, e:
            return None

    def force_inbound(self, curr_x, curr_y):
        new_x = max(0, min(curr_x, self.info.width - 1))
        new_y = max(0, min(curr_y, self.info.height - 1))

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

    def is_cell_in_bounds(self, x, y):
        return 0 <= x < self.info.width - 1 and self.info.height - 1 > y >= 0

    def grid_to_index(self, x, y):
        """
        Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
        :param x [int] The cell X coordinate.
        :param y [int] The cell Y coordinate.
        :return  [int] The index.
        """
        return y * self.info.width + x

    def get_cell_value(self, x, y):
        if not self.is_cell_in_bounds(x, y):
            raise IndexError("The cell index (%d, %d) is outside of this map (size %dx%d)" % (
                x, y, self.info.width, self.info.height))
        return self.data[self.grid_to_index(x, y)]

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
        Calculates the C-Space, i.e., makes the obstacles in the map thicker.
        Publishes the list of cells that were added to the original map.
        :param mapdata [OccupancyGrid] The map data.
        :param padding [int]           The number of cells around the obstacles.
        :return        [OccupancyGrid] The C-Space.self.pubCspace.publish(msg)
        """
        OBSTACLE_THRESH = 90
        rospy.loginfo("Calculating C-Space")

        paddedArray = list(self.data)

        ## Go through each cell in the occupancy grid
        for x in range(self.info.height):
            for y in range(self.info.width):
                ## Inflate the obstacles where necessary
                if self.data[self.grid_to_index(x, y)] > OBSTACLE_THRESH:
                    paddedArray[self.grid_to_index(x, y)] = 100
                    for neighbor in self.neighbors_of_8(x, y):
                        x3, y3 = self.force_inbound(neighbor[0], neighbor[1])
                        paddedArray[self.grid_to_index(x3, y3)] = 100

        paddedArray = tuple(paddedArray)
        gridCellsList = []

        for x in range(self.info.height):
            for y in range(self.info.width):
                ## Inflate the obstacles where necessary
                if paddedArray[self.grid_to_index(x, y)] > OBSTACLE_THRESH:
                    world_point = self.grid_to_world(x, y)
                    gridCellsList.append(world_point)

        ## Create a GridCells message and publish it
        msg = GridCells()
        msg.cell_width = self.info.resolution
        msg.cell_height = self.info.resolution
        msg.cells = gridCellsList
        msg.header = self.header
        self.pubCspace.publish(msg)
        self.cspace_data = paddedArray

    def update_map(self, occupancyGrid):
        self.map = occupancyGrid

    def tuple_distance_between(self, tup1, tup2):
        #Wrapper for equalidant distance function that takes in tuples directly
        return(distance_between(tup1[0],tup1[1],tup2[0],tup2[1]))


