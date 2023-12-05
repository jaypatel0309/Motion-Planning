# Standard Algorithm Implementation
# Sampling-based Algorithms RRT and RRT*

from scipy import spatial
import matplotlib.pyplot as plt
import numpy as np

# Class for each tree node
class Node:
    def __init__(self, row, col):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.parent = None    # parent node
        self.cost = 0.0       # cost

# Class for RRT
class RRT:
    # Constructor
    def __init__(self, map_array, start, goal):
        self.map_array = map_array            # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]    # map size
        self.size_col = map_array.shape[1]    # map size

        self.start = Node(start[0], start[1]) # start node
        self.goal = Node(goal[0], goal[1])    # goal node
        self.vertices = []                    # list of nodes
        self.found = False                    # found flag
        
        self.extension_distance = 10
        self.goal_bias = 0.05

    def init_map(self):
        '''Intialize the map before each search
        '''
        self.found = False
        self.vertices = []
        self.vertices.append(self.start)

    def dis(self, node1, node2):
        '''Calculate the euclidean distance between two nodes
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            euclidean distance between two nodes
        '''
        euc_dist = np.sqrt((node1.row - node2.row)**2 + (node1.col - node2.col)**2)
        return euc_dist
    
    def check_collision(self, node1, node2):
        '''Check if the path between two nodes collide with obstacles
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            True if the new node is valid to be connected
        '''

        line_pts = zip(np.linspace(node1.row, node2.row, dtype= int), np.linspace(node1.col,node2.col, dtype = int))
        
        for pt in line_pts:
            if self.map_array[pt[0]][pt[1]] == 0:
                return True

        return False

    def get_new_point(self, goal_bias):
        '''Choose the goal or generate a random point
        arguments:
            goal_bias - the possibility of choosing the goal instead of a random point

        return:
            point - the new point
        '''

        if np.random.random() < goal_bias:
            newpoint = [self.goal.col, self.goal.row]
        else:
            newpoint = [np.random.randint(0, self.size_col-1), np.random.randint(0, self.size_row-1)]
        
        return newpoint
    
    def get_nearest_node(self, point):
        '''Find the nearest node in self.vertices with respect to the new point
        arguments:
            point - the new point

        return:
            the nearest node
        '''
        verticeslist = [[vertex.row, vertex.col] for vertex in self.vertices]
        vkdtree = spatial.cKDTree(verticeslist)
        point, idx = vkdtree.query(point)

        return self.vertices[idx]

    def get_neighbors(self, new_node, neighbor_size):
        '''Get the neighbors that are within the neighbor distance from the node
        arguments:
            new_node - a new node
            neighbor_size - the neighbor distance

        return:
            neighbors - a list of neighbors that are within the neighbor distance 
        '''
        verticeslist = [[vertex.row, vertex.col] for vertex in self.vertices]
        vkdtree = spatial.cKDTree(verticeslist)
        idx = vkdtree.query_ball_point([new_node.row, new_node.col], neighbor_size)
        neighbors = [self.vertices[_] for _ in idx]
        neighbors.remove(new_node)

        return neighbors

    def rewire(self, new_node, neighbors):
        '''Rewire the new node and all its neighbors
        arguments:
            new_node - the new node
            neighbors - a list of neighbors that are within the neighbor distance from the node

        Rewire the new node if connecting to a new neighbor node will give least cost.
        Rewire all the other neighbor nodes.
        '''

        if neighbors == []:
            return

        dist = [self.dis(new_node, neighbor) for neighbor in neighbors]
        distances = [distance + self.pathdistance(self.start, neighbors[idx]) for idx, distance in enumerate(dist)]
        ind = np.argsort(np.array(distances))

        for idx in ind:
            if not self.check_collision(new_node, neighbors[idx]):
                new_node.parent = neighbors[idx]
                new_node.cost = dist[idx]
                break

        for ind, neighbor in enumerate(neighbors):

            new_distance = self.pathdistance(self.start, new_node) + dist[ind]

            if self.pathdistance(self.start, neighbor) > new_distance:
                if not self.check_collision(neighbor, new_node):
                    neighbor.parent = new_node
                    neighbor.cost = dist[ind]

    def draw_map(self):
        '''Visualization of the result
        '''
        # Create empty map
        fig, ax = plt.subplots(1)
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw Trees or Sample points
        for node in self.vertices[1:-1]:
            plt.plot(node.col, node.row, markersize=3, marker='o', color='y')
            plt.plot([node.col, node.parent.col], [node.row, node.parent.row], color='y')
        
        # Draw Final Path if found
        if self.found:
            cur = self.goal
            while cur.col != self.start.col or cur.row != self.start.row:
                plt.plot([cur.col, cur.parent.col], [cur.row, cur.parent.row], color='b')
                cur = cur.parent
                plt.plot(cur.col, cur.row, markersize=3, marker='o', color='b')

        # Draw start and goal
        plt.plot(self.start.col, self.start.row, markersize=5, marker='o', color='g')
        plt.plot(self.goal.col, self.goal.row, markersize=5, marker='o', color='r')

        # show image
        plt.show()


    def RRT(self, n_pts=1000):
        '''RRT main search function
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        In each step, extend a new node if possible, and check if reached the goal
        '''
        # Remove previous result
        self.init_map()

        # In each step,
        # get a new point, 
        # get its nearest node, 
        # extend the node and check collision to decide whether to add or drop,
        # if added, check if reach the neighbor region of the goal.

        for _ in range(n_pts):
            newpt = self.extension(self.extension_distance, self.goal_bias)
            
            if self.found:
                break

        # Output
        if self.found:
            self.goal.cost = self.pathdistance(self.start, self.goal)

            steps = len(self.vertices) - 2
            length = self.goal.cost

            print("It took %d nodes to find the current path" %steps)
            print("The path length is %.2f" %length)

        else:
            print("No path found")
        
        # Draw result
        self.draw_map()

    def RRT_star(self, n_pts=1000, neighbor_size=20):
        '''RRT* search function
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points
            neighbor_size - the neighbor distance
        
        In each step, extend a new node if possible, and rewire the node and its neighbors
        '''
        # Remove previous result
        self.init_map()

        # In each step,
        # get a new point, 
        # get its nearest node, 
        # extend the node and check collision to decide whether to add or drop,
        # if added, rewire the node and its neighbors,
        # and check if reach the neighbor region of the goal if the path is not found.

        for _ in range(n_pts):
            newpt = self.extension(self.extension_distance, self.goal_bias)
            
            if newpt is not None:
                neighbors = self.get_neighbors(newpt, neighbor_size)
                self.rewire(newpt, neighbors)

        # Output
        if self.found:
            self.goal.cost = self.pathdistance(self.start, self.goal)

            steps = len(self.vertices) - 2
            length = self.goal.cost

            print("It took %d nodes to find the current path" %steps)
            print("The path length is %.2f" %length)
        else:
            print("No path found")

        # Draw result
        self.draw_map()
    
    def extension(self, distance, goal_bias):

        newpt = self.get_new_point(goal_bias)
        closestnode = self.get_nearest_node(newpt)

        lineslope = self.line_slope(newpt,[closestnode.row,closestnode.col])

        newclosept_row, newclosept_col, newclosept = self.smallclose_pt(closestnode.row, closestnode.col, distance, lineslope)

        if (0 <= newclosept_col < self.size_col) and (0 <= newclosept_row < self.size_row):
            if not self.check_collision(closestnode, newclosept):

                newclosept.parent = closestnode
                newclosept.cost = distance
                self.vertices.append(newclosept)

                if not self.found:
                    dis = self.dis(newclosept, self.goal)

                    if dis < distance:
                        self.goal.cost = dis
                        self.goal.parent = newclosept
                        self.vertices.append(self.goal)
                        self.found = True

                return newclosept
        else:
            return None

    def pathdistance(self, point1, point2):

        pathdist = 0
        presentpt = point2
        
        while point1.row != presentpt.row or point1.col != presentpt.col:

            parent = presentpt.parent            
            pathdist += presentpt.cost
            presentpt = parent
        
        return pathdist

    def line_slope(self,point1, point2):
        return np.arctan2(point1[1] - point2[1], point1[0] - point2[0])

    def smallclose_pt(self, closenoderow, closenodecol, distance, lineslope):
        newclosept_row = closenoderow + distance*np.cos(lineslope)
        newclosept_col = closenodecol + distance*np.sin(lineslope)
        newclosept = Node(int(newclosept_row), int(newclosept_col))

        return newclosept_row, newclosept_col, newclosept