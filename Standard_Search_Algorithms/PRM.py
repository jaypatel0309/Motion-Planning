# Standard Algorithm Implementation
# Sampling-based Algorithms PRM

import matplotlib.pyplot as plt
import numpy as np
import networkx as nx
from scipy import spatial


# Class for PRM
class PRM:
    # Constructor
    def __init__(self, map_array):
        self.map_array = map_array            # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]    # map size
        self.size_col = map_array.shape[1]    # map size

        self.samples = []                     # list of sampled points
        self.graph = nx.Graph()               # constructed graph
        self.path = []                        # list of nodes of the found path

        self.kdtree = None

    def check_collision(self, p1, p2):
        '''Check if the path between two points collide with obstacles
        arguments:
            p1 - point 1, [row, col]
            p2 - point 2, [row, col]

        return:
            True if there are obstacles between two points
        '''
        line_pts = zip(np.linspace(p1[0],p2[0],dtype= int), np.linspace(p1[1],p2[1],dtype = int))
        
        for pt in line_pts:
            if self.map_array[pt[0]][pt[1]] == 0:
                return True

        return False

    def dis(self, point1, point2):
        '''Calculate the euclidean distance between two points
        arguments:
            p1 - point 1, [row, col]
            p2 - point 2, [row, col]

        return:
            euclidean distance between two points
        '''
        euc_dist = np.sqrt((point1[0]-point2[0])**2 + (point1[1]-point2[1])**2)
        
        return euc_dist                            

    def uniform_sample(self, n_pts):
        '''Use uniform sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()
        
        x = int(np.sqrt(n_pts * self.size_row / self.size_col))
        y = int(n_pts / x)

        grid_row = np.linspace(0, self.size_row-1, x, dtype=int)
        grid_col = np.linspace(0, self.size_col-1, y, dtype=int)
        rows, cols = np.meshgrid(grid_row, grid_col)
        rows = rows.flatten()
        cols = cols.flatten()

        for row, col in zip(rows, cols):
            if self.map_array[row][col] == 1:
                self.samples.append((row, col))
 
    def random_sample(self, n_pts):
        '''Use random sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''

        rows = np.random.randint(0, self.size_row-1, n_pts, dtype=int)
        cols = np.random.randint(0, self.size_col-1, n_pts, dtype=int)

        for row, col in zip(rows, cols):
            if self.map_array[row][col] == 1:
                self.samples.append((row, col))

    def gaussian_sample(self, n_pts):
        '''Use gaussian sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        p1_x = np.random.randint(0, self.size_row-1, n_pts, dtype=int)
        p1_y = np.random.randint(0, self.size_col-1, n_pts, dtype=int)

        p2_x = p1_x + np.random.normal(0.0, 10, n_pts).astype(int)
        p2_y = p1_y + np.random.normal(0.0, 10, n_pts).astype(int)

        for row1, col1, row2, col2 in zip(p1_x, p1_y, p2_x, p2_y):
            if not(0 <= row2 < self.size_row) or not(0 <= col2 < self.size_col):
                continue
            
            if self.map_array[row1][col1] == 1 and self.map_array[row2][col2] == 0:
                self.samples.append((row1, col1))
            elif self.map_array[row1][col1] == 0 and self.map_array[row2][col2] == 1:
                self.samples.append((row2, col2))

    def bridge_sample(self, n_pts):
        '''Use bridge sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''

        p1_x = np.random.randint(0, self.size_row-1, n_pts, dtype=int)
        p1_y = np.random.randint(0, self.size_col-1, n_pts, dtype=int)
        

        p2_x = p1_x + np.random.normal(0.0, 15, n_pts).astype(int)
        p2_y = p1_y + np.random.normal(0.0, 15, n_pts).astype(int)
        
        for row1, col1, row2, col2 in zip(p1_x, p1_y, p2_x, p2_y):
            if ((not(0 <= col2 < self.size_col) or not(0 <= row2 < self.size_row)) or self.map_array[row2][col2] == 0):
               if self.map_array[row1][col1] == 0:
                    mid_row, mid_col = int(0.5*(row1+row2)), int(0.5*(col1+col2))
                    if 0 <= mid_row < self.size_row and 0 <= mid_col < self.size_col and self.map_array[mid_row][mid_col] == 1:
                        self.samples.append((mid_row, mid_col))

    def draw_map(self):
        '''Visualization of the result
        '''
        # Create empty map
        fig, ax = plt.subplots()
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw graph
        # get node position (swap coordinates)
        node_pos = np.array(self.samples)[:, [1, 0]]
        pos = dict( zip( range( len(self.samples) ), node_pos) )
        pos['start'] = (self.samples[-2][1], self.samples[-2][0])
        pos['goal'] = (self.samples[-1][1], self.samples[-1][0])
        
        # draw constructed graph
        nx.draw(self.graph, pos, node_size=3, node_color='y', edge_color='y' ,ax=ax)

        # If found a path
        if self.path:
            # add temporary start and goal edge to the path
            final_path_edge = list(zip(self.path[:-1], self.path[1:]))
            nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=self.path, node_size=8, node_color='b')
            nx.draw_networkx_edges(self.graph, pos=pos, edgelist=final_path_edge, width=2, edge_color='b')

        # draw start and goal
        nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=['start'], node_size=12,  node_color='g')
        nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=['goal'], node_size=12,  node_color='r')

        # show image
        plt.axis('on')
        ax.tick_params(left=True, bottom=True, labelleft=True, labelbottom=True)
        plt.show()


    def sample(self, n_pts=1000, sampling_method="uniform"):
        '''Construct a graph for PRM
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points
            sampling_method - name of the chosen sampling method

        Sample points, connect, and add nodes and edges to self.graph
        '''
        self.samples = []
        self.graph.clear()
        self.path = []

        if sampling_method == "uniform":
            knearest = 15
            self.uniform_sample(n_pts)
        elif sampling_method == "random":
            knearest = 18
            self.random_sample(n_pts)
        elif sampling_method == "gaussian":
            knearest = 20
            self.gaussian_sample(n_pts)
        else:
            knearest = 20            
            self.bridge_sample(n_pts)

        knearest = 15

        self.kdtree = spatial.cKDTree(list(self.samples))
        pairs = self.kdtree.query_pairs(knearest)

        self.graph.add_nodes_from(range(len(self.samples)))
        self.edges(pairs)

        n_nodes = self.graph.number_of_nodes()
        n_edges = self.graph.number_of_edges()
        print("The constructed graph has %d nodes and %d edges" %(n_nodes, n_edges))


    def edges(self, pairs):

        for pair in pairs:
            firstElement = pair[0]
            if firstElement == "start":
                pt1 = self.samples[-2]
            elif firstElement == "goal":
                pt1 = self.samples[-1]
            else:
                pt1 = self.samples[firstElement]
            pt2 = self.samples[pair[1]]

            if not self.check_collision(pt1, pt2):
                d = self.dis(pt1, pt2)
                edge = [(firstElement, pair[1], d)]
                self.graph.add_weighted_edges_from(edge)

    def search(self, start, goal, knearest =15):
        '''Search for a path in graph given start and goal location
        arguments:
            start - start point coordinate [row, col]
            goal - goal point coordinate [row, col]

        Temporary add start and goal node, edges of them and their nearest neighbors
        to graph for self.graph to search for a path.
        '''
        self.path = []

        self.samples.append(start)
        self.samples.append(goal)
        self.graph.add_nodes_from(['start', 'goal'])

        sg_kdtree = spatial.cKDTree([start, goal])
        sg_neighbors = sg_kdtree.query_ball_tree(self.kdtree, knearest)
        start_pairs = ([['start', neighbor] for neighbor in sg_neighbors[0]])
        goal_pairs = ([['goal', neighbor] for neighbor in sg_neighbors[1]])

        self.edges(start_pairs)
        self.edges(goal_pairs)
    
        try:
            self.path = nx.algorithms.shortest_paths.weighted.dijkstra_path(self.graph, 'start', 'goal')
            path_length = nx.algorithms.shortest_paths.weighted.dijkstra_path_length(self.graph, 'start', 'goal')
            print("The path length is %.2f" %path_length)
        except nx.exception.NetworkXNoPath:
            print("No path found")
        
        self.draw_map()

        self.samples.pop(-1)
        self.samples.pop(-1)
        self.graph.remove_nodes_from(['start', 'goal'])
        self.graph.remove_edges_from(start_pairs)
        self.graph.remove_edges_from(goal_pairs)