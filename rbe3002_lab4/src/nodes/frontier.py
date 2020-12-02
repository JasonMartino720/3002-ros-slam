#!/usr/bin/env python2

import rospy
from nav_msgs.msg import GridCells
from nav_msgs.srv import GetMap


class Frontier:

    def __init__(self):
        rospy.loginfo("Started Frontier node")
        rospy.init_node("frontier")

        #Do we have to make a custom message or can we use a premade one?
        self.pathService = rospy.Service('frontier_list', "???", self.return_frontier)

        ### Tell ROS that this node subscribes to map which is from Gmapping
        # same as above, is there a map udpate message of some kind? I have a internet outage btw
        #that's why I'm guessing these things
        rospy.Subscriber("/map", "???", self.update_map)

        rospy.sleep(1.0)
        rospy.loginfo("Frontier node ready")

        self.map = None

    def update_map(self, occupancyGrid):
        self.map = occupancyGrid

    def tuple_distance_between(self, tup1, tup2):
        #Wrapper for equalidant distance function that takes in tuples directly
        return(distance_between(tup1[0],tup1[1],tup2[0],tup2[1]))

    def return_frontier(self, msg):
        #I have a feeling this is gonna get super complex, we might need to use somethink like
        #K-means clusting instead, please google clustering algrotyhms so we have a good starting point
        edge_cells = self.get_frontier_cells()

        NEIGHBOR_DISTANCE = 2 #How far aparat can cells in one cluster be?

        cell_iterator = iter(edge_cells) #Loop through all the cells but you can also get the next one with next()
        clustered_cells = list() #Should end up with N list insdie this where each list is it's own cluster
        current_cluster_index = 0 #For keeping track of which index we are adding to
        current_cluster_builder = list() #Builids up the current cluster before submitting it to the main list
        current_cluster_builder.append(next(cell_iterator)) #add the first cell to the first cluster
        for cells_that_need_assignment in cell_iterator:
            for new_cell in edge_cells:
                for currentCell in current_cluster_builder:
                    if(self.tuple_distance_between(currentCell, new_cell) < NEIGHBOR_DISTANCE):
                        current_cluster_builder.append(new_cell) #This cell was assigned to this cluster since it was close enough
                        pass #Move on to next cell
                #After checking every cell in the current cluster, if no cell is close enough to the new cell
                current_cluster_index += 1
                clustered_cells.append(current_cluster_builder)
                #Make a new cluster with this cell as a the start
                current_cluster_builder = list()
                current_cluster_builder.append(new_cell)


    def get_frontier_cells(self):
        #Assuming that the map node has been split off into it's own node already
        #Assumign update Map has already given us the newest map

        OBSTACLE_THRESH = 90
        rospy.loginfo("Calculating edge")

        frontier_cells = list()

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
                        this_is_a_frontier = False
                        if is_cell_walkable(neighbor[0],neighbor[1]):
                            #thee is probably a better way of doing this that is either 0 or 1, not 0 or 100
                            this_is_a_frontier = True
                        if(this_is_a_frontier):
                            frontier_cells.append((x, y))

        return frontier_cells
