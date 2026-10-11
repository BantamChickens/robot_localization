""" An implementation of an occupancy field that you can use to implement
    your particle filter """

import rclpy
from nav_msgs.srv import GetMap
import numpy as np
from sklearn.neighbors import NearestNeighbors

class OccupancyField(object):
    """ Stores an occupancy field for an input map.  An occupancy field returns
        the distance to the closest obstacle for any coordinate in the map
        Attributes:
            map: the map to localize against (nav_msgs/OccupancyGrid)
            closest_occ: the distance for each entry in the OccupancyGrid to
            the closest obstacle
    """

    def __init__(self, node):
        #-------------------Grab the map from the map server------------------------------------------------
        """
        A ROS2 service client sends a request to a server and recieves a response.
        Unlike a publisher/subscriber which is a one way continous channel, a service 
        is an on demand two way channel.
        """
        # Create a client for 'GetMap' to request an occupancy grid
        self.cli = node.create_client(GetMap, 'map_server/map')               #initialize connection to the map server

        while not self.cli.wait_for_service(timeout_sec=1.0):                 #if server has not responded within 1 second...
            node.get_logger().info('service not available, waiting again...') #print message and try again

        # Request the map
        self.future = self.cli.call_async(GetMap.Request())                                         #request and store response 
        rclpy.spin_until_future_complete(node, self.future)                                         #listen for server response until it arrives
        self.map = self.future.result().map                                                         #
        node.get_logger().info("map received width: {0} height: {1}".format(self.map.info.width, 
                                                                            self.map.info.height)) 
        
        # -------------------Setup the map and obstacles--------------------------------------------------------
        # Create a grid of all the cells in the map
        X = np.zeros((self.map.info.width*self.map.info.height, 2))   #initialize an empty grid the same width and height as map

        total_occupied = 0   #starting point for current number of occupied map cells                                          
        curr = 0             #starting row
        
        for i in range(self.map.info.width):                 #for each column...          
            for j in range(self.map.info.height):            #...loop through each row...
                ind = i + j*self.map.info.width              #...set up index system to track cells
                
                if self.map.data[ind] > 0:                   #if cell contains data...
                    total_occupied += 1                      #...add cell to total occupied count
                
                X[curr, 0] = float(i)                        # store the cell's x coordinate                    
                X[curr, 1] = float(j)                        # store the cell's y coordinate
                curr += 1                                    # move on to the next row

        # Create a list of just the occupied cells
        occupied = np.zeros((total_occupied, 2))             # create an empty array to store two values per item (x and y corrdinates for each occupied cell)
        curr = 0                                             #initalize current row to 0
        
        for i in range(self.map.info.width):                 #for each column...
            for j in range(self.map.info.height):            #...loop through each row...
                
                # occupancy grids are stored in row major order
                ind = i + j*self.map.info.width              #...set up index system to track cells
                
                if self.map.data[ind] > 0:                   #if cell contains data...   
                    occupied[curr, 0] = float(i)             # store the cell's x coordinate    
                    occupied[curr, 1] = float(j)             # store the cell's y coordinate
                    curr += 1                                # move on to the next row    
        
        node.get_logger().info("building ball tree")         #print

        #-------Determine how close object is to obstacle using nearest neighbors algorithm-------
        nbrs = NearestNeighbors(n_neighbors=1,
                                algorithm="ball_tree").fit(occupied)        #search for the closest occupied cell from specified point using ball tree method
        node.get_logger().info("finding neighbors")                         #print
        
        distances, indices = nbrs.kneighbors(X)                             # search for closest occupied cell for each point in X and return distance and index
        node.get_logger().info("populating occupancy field")                #print
        
        self.closest_occ = np.zeros((self.map.info.width, self.map.info.height)) #create empty grid same width and height as map
        
        for i in range(self.map.info.width):                                     #for each column...
            for j in range(self.map.info.height):                                #...loop through each row...  
                self.closest_occ[i, j] = \
                    distances[curr][0]*self.map.info.resolution                  #...convert distance to meters and store
                curr += 1                                                        # move on to the next row
        
        self.occupied = occupied                                            #store to reference later
        node.get_logger().info("occupancy field ready")                     #print

    def get_obstacle_bounding_box(self):
        """
        Returns: the upper and lower bounds of x and y such that the resultant
        bounding box contains all of the obstacles in the map.
        """
        lower_bounds = self.occupied.min(axis=0) #find the smallest x and y cordinates
        upper_bounds = self.occupied.max(axis=0) #find the largest x and y cordinates
        r = self.map.info.resolution             #cell size in meters
        
        # 
        return ((lower_bounds[0]*r + self.map.info.origin.position.x,   #return (x_lower, x_upper), (y_lower, y_upper) in meters
                 upper_bounds[0]*r + self.map.info.origin.position.x),
                (lower_bounds[1]*r + self.map.info.origin.position.y,
                 upper_bounds[1]*r + self.map.info.origin.position.y))

    def get_closest_obstacle_distance(self, x, y):
        """ Compute the closest obstacle to the specified (x,y) coordinate in
            the map.  If the (x,y) coordinate is out of the map boundaries, nan
            will be returned. """
        x_coord = (x - self.map.info.origin.position.x)/self.map.info.resolution
        y_coord = (y - self.map.info.origin.position.y)/self.map.info.resolution
        
        if type(x) is np.ndarray:
            x_coord = x_coord.astype(np.int)
            y_coord = y_coord.astype(np.int)
        else:
            x_coord = int(x_coord)
            y_coord = int(y_coord)

        is_valid = (x_coord >= 0) & (y_coord >= 0) & (x_coord < self.map.info.width) & (y_coord < self.map.info.height)
        if type(x) is np.ndarray:
            distances = np.float('nan')*np.ones(x_coord.shape)
            distances[is_valid] = self.closest_occ[x_coord[is_valid], y_coord[is_valid]]
            return distances
        else:
            return self.closest_occ[x_coord, y_coord] if is_valid else float('nan')
