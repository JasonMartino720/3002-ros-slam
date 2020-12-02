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

        edge_cells = self.get_frontier_cells()
        #Assume inpuit is a list of tuples [(X,Y),(X,Y)]
        #Where each cell is a edge between walkable and unknown

        #I'm not sure but you might need to convert this to np array of pandas DF
        # edge_cells = convert_to_np(edge_cells)

        #Try changing this
        NUMBER_OF_PREDICTED_CLUSTERS = 3

        kmeans = KMeans(n_clusters=NUMBER_OF_PREDICTED_CLUSTERS, n_init=10)
        kmeans.fit(edge_cells)

        #expect a int list with the value that reprenets the cluster number
        sorted_index = kmeans.labels_

        #This is what you get (from the docs)
            # >> > X = np.array([[1, 2], [1, 4], [1, 0],
            #                    ...[10, 2], [10, 4], [10, 0]])
            # >> > kmeans = KMeans(n_clusters=2, random_state=0).fit(X)
            # >> > kmeans.labels_
            # array([1, 1, 1, 0, 0, 0], dtype=int32)

        # Return two lists, a centeroids list and size (number of cells) list

        # Expectting two lists
        # Centeroid => (x,y),(x,y)...
        # Size => (size),(size)...
        centeroid_list = kmeans.cluster_centers_

        #size_list = count of number of cells in each cluster
        # you should end up with [Number of cells in cluster1, #of cells in cluster2, #of cells in 3]
        #if you have 3 cells


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
