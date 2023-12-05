# Basic searching algorithms

# Node Class for storing node information
class Node:
    def __init__(self, row, col, parent, action, is_obs):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.is_obs = is_obs  # obstacle?
        self.g = None         # cost to come (previous g + moving cost)
        self.h = None            # heuristic
        self.cost = None      # total cost (depend on the algorithm)
        self.parent = parent    # previous node
        self.action = action  # action taken to reach a state
        self.state = (self.row,self.col)  # store row and column as tuple

# Class for creating stack frontier for DFS to implement LIFO stack
class StackFrontier():

    def __init__(self):
        self.frontier = []

    # Function to remove last state of list
    def remove(self):
        node = self.frontier[-1]
        self.frontier = self.frontier[:-1]
        return node
    
    # Function to append state to frontier list
    def add(self, node):
        return self.frontier.append(node)
    
    # Function to check if state is present in the frontier list
    def contains_state(self,state):
        return any(node.state == state for node in self.frontier)

    # Function to check if the frontier list is empty
    def empty(self):
        return len(self.frontier) == 0

# Class for creating Queue frontier for implementing FIFO queue mostly inherited from StackFrontier class
class QueueFrontier(StackFrontier):

    # Function to remove first state of frontier list
    def remove(self):
        node = self.frontier[0]
        self.frontier = self.frontier[1:]
        return node

# Function for creating neighbors for BFS, Dijkstra and A star searches
def bfsNeighbors(state, grid):

    row,col = state
    neighborcells = [('right', (row, col+1)),('down', (row+1, col)),('left', (row, col-1)),('up', (row -1,col))]

    neighborlist = []
    for action, (r,c) in neighborcells:
        if 0 <= r < len(grid) and 0 <= c < len(grid[0]) and grid[r][c] != 1:
            neighborlist.append((action, (r, c)))
    return neighborlist

# Function for creating neighbors for DFS search    
def dfsNeighbors(state, grid):

    row,col = state
    neighborcells = [('up', (row -1,col)),('left', (row, col-1)),('down', (row+1, col)),('right', (row, col+1))]

    neighborlist = []
    for action, (r,c) in neighborcells:
        if 0 <= r < len(grid) and 0 <= c < len(grid[0]) and grid[r][c] != 1:
            neighborlist.append((action, (r, c)))
    return neighborlist

# Heurestic function for calculating manhattan distance between two states
def heuristic(state1,state2):
    h = abs(state1[0] - state2[0]) + abs(state1[1] - state2[1])
    return h

# Function to check if the current state is the goal and return path from start to goal and actions taken to goal
def checkGoal(start, currentnode):
    actions = []
    cells = []

    while currentnode.parent is not None:
        actions.append (currentnode.action)
        cells.append (currentnode.state)
        currentnode = currentnode.parent

    actions.append(start.action)
    actions.reverse()

    cells.append(start.state)
    cells.reverse()
    actions, path = (actions, cells)

    return actions,path

# BFS code implementation function
def bfs(grid, start, goal):
    '''Return a path found by BFS alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> bfs_path, bfs_steps = bfs(grid, start, goal)
    It takes 10 steps to find a path using BFS
    >>> bfs_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False

    # creating starting node
    start = Node(row = start[0],col = start[1],parent = None, action = None, is_obs=grid[start[0]][start[1]])
    
    # Creating frontier instance with QueueFrontier class for FIFO queue
    frontier = QueueFrontier()

    # Add popped node to frontier list
    frontier.add(start)

    # create explored set to store visited nodes
    explored = set()

    # Run till goal is found or frontier becomes empty    
    while True:
        # Break if the frontier list is empty
        if frontier.empty():
            break

        # Remove first state of frontier and add to explored set    
        node = frontier.remove()
        explored.add(node.state)

        # Return path and actions to goal from start if the current state is goal
        if list(node.state) == goal:
            found = True
            actions, path = checkGoal(start, node)
            break

        # Creating node for neighbors of popped state if the neighbors don't exist in explored set and if the neighbor isn't a obstacle        
        for action, state in bfsNeighbors(node.state, grid):
            if state not in explored and grid[state[0]][state[1]] != 1:
                child = Node(row = state[0],col = state[1], parent = node, action = action, is_obs = grid[state[0]][state[1]])
                frontier.add(child)

    # Counting steps for calculating the number of explored nodes
    steps = len(explored)

    #Path returns the path from start to goal
    path = [list(ele) for ele in path]

    if found:
        print(f"It takes {steps} steps to find a path using BFS")
    else:
        print("No path found")
    return path, steps

# DFS code implementation function
def dfs(grid, start, goal):
    '''Return a path found by DFS alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> dfs_path, dfs_steps = dfs(grid, start, goal)
    It takes 9 steps to find a path using DFS
    >>> dfs_path
    [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2], [2, 3], [3, 3], [3, 2], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False

    # creating starting node    
    start = Node(row = start[0],col = start[1],parent = None, action = None, is_obs= grid[start[0]][start[1]])

    # Creating frontier instance with StackFrontier class for LIFO queue
    frontier = StackFrontier()

    # Add popped node to frontier list
    frontier.add(start)

    # create explored set to store visited nodes
    explored = set()

    # Run till goal is found or frontier becomes empty      
    while True:
        # Break out of loop if the frontier list is empty
        if frontier.empty():
            break
        
        # Remove last state of frontier and add to explored set         
        node = frontier.remove()      
        explored.add(node.state)

        # Return path and actions to goal from start if the current state is goal and break out of the loop
        if list(node.state) == goal:
            found = True
            actions, path = checkGoal(start, node)
            break

        # Creating node for neighbors of popped state if the neighbors don't exist in explored set and if the neighbor isn't a obstacle
        for action, state in dfsNeighbors(node.state, grid):
            if state not in explored and grid[state[0]][state[1]] != 1:
                child = Node(row = state[0],col = state[1], parent = node, action = action, is_obs= grid[state[0]][state[1]])
                frontier.add(child)

    # Path returns the path from start to goal
    path = [list(ele) for ele in path]
    
    # Counting steps for calculating the number of explored nodes
    steps = len(explored)

    if found:
        print(f"It takes {steps} steps to find a path using DFS")
    else:
        print("No path found")
    return path, steps

def dijkstra(grid, start, goal):
    '''Return a path found by Dijkstra alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> dij_path, dij_steps = dijkstra(grid, start, goal)
    It takes 10 steps to find a path using Dijkstra
    >>> dij_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False

    # creating starting node and setting initial cost to come for start node
    start = Node(row = start[0],col = start[1],parent = None, action = None, is_obs= grid[start[0]][start[1]])
    start.g = 0

    # Creating frontier instance with QueueFrontier class for FIFO queue and adding start to frontier list
    frontier = QueueFrontier()
    frontier.add(start)

    # create explored set to store visited nodes
    explored = set()

    # Run till goal is found or frontier becomes empty
    while True:
    # Break out of loop if the frontier list is empty
        if frontier.empty():
            break
            
        #sorting the queue to pop closest node (cost to come) first
        frontier.frontier.sort(key = lambda x: x.g)

        # Remove first state of sorted frontier and add to explored set 
        node = frontier.remove()
        explored.add(node.state)

        # Return path and actions to goal from start if the current state is goal and break out of the loop
        if list(node.state) == goal:
            found = True
            actions, path = checkGoal(start, node)
            break
        
        # Creating node for neighbors of popped state if the neighbors don't exist in explored set and if the neighbor isn't a obstacle        
        for action, state in bfsNeighbors(node.state, grid):
            if state not in explored and grid[state[0]][state[1]] != 1:
                child = Node(row = state[0],col = state[1], parent = node, action = action, is_obs = grid[state[0]][state[1]])
                
                # Setting cost to come for each created child node
                child.g = node.g + 1
                frontier.add(child)            

    # Path returns the path from start to goal
    path = [list(ele) for ele in path]

    # Counting steps for calculating the number of explored nodes
    steps = len(explored)

    if found:
        print(f"It takes {steps} steps to find a path using Dijkstra")
    else:
        print("No path found")
    return path, steps

def astar(grid, start, goal):
    '''Return a path found by A* alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> astar_path, astar_steps = astar(grid, start, goal)
    It takes 7 steps to find a path using A*
    >>> astar_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False

    # creating starting node and setting initial cost parameters
    start = Node(row = start[0],col = start[1],parent = None, action = None, is_obs= grid[start[0]][start[1]])
    start.h = heuristic(start.state,goal)
    start.g = 0
    start.cost = start.g + start.h

    # Creating frontier instance with QueueFrontier class for FIFO queue and adding start to frontier list
    frontier = QueueFrontier()
    frontier.add(start)

    # create explored set to store visited nodes
    explored = set()

    # Run till goal is found or frontier becomes empty
    while True:
        # Break out of loop if the frontier list is empty
        if frontier.empty():
            break

        #sorting the queue to pop closest (cost to come and heuristics) node first
        frontier.frontier.sort(key = lambda x: x.cost)

        # Remove first state of sorted frontier and add to explored set 
        node = frontier.remove()
        explored.add(node.state)

        # Return path and actions to goal from start if the current state is goal and break out of the loop
        if list(node.state) == goal:
            found = True
            actions, path = checkGoal(start, node)
            break
        
        # Creating node for neighbors of popped state if the neighbors don't exist in explored set and if the neighbor isn't a obstacle 
        for action, state in bfsNeighbors(node.state, grid):
            if state not in explored and grid[state[0]][state[1]] != 1 :
                child = Node(row = state[0],col = state[1], parent = node, action = action, is_obs = grid[state[0]][state[1]])   
                # Set cost parameters for each created child node
                child.g = node.g + 1
                child.h = heuristic(child.state, goal)
                child.cost = child.h + child.g
                frontier.add(child)            

    # Path returns the path from start to goal
    path = [list(ele) for ele in path]

    # Counting steps for calculating the number of explored nodes
    steps = len(explored)

    if found:
        print(f"It takes {steps} steps to find a path using A*")
    else:
        print("No path found")
    return path, steps

# Doctest
if __name__ == "__main__":
    # load doc test
    from doctest import testmod, run_docstring_examples
    # Test all the functions
    testmod()

