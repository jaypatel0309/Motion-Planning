## Overview

In this project, advanced planning algorithms **D*** and **Informed RRT*** are implemented. 


## Instruction

For the **Informed RRT***, the **main.py** loads the map image **WPI_map.jpg** and calls classes and functions to run planning tasks. 

For the **D***,  the **main.py** loads two map, a static one and a dynamic one. The static one is for the initial planning, while the dynamic one tells where the new obstacles are. 

## D*

1. The **run** function is the main function of D* algorithm, which includes two main steps. The first step is to search from goal to start in the static map. The second step is to move from start to goal. 
2. The **process_state** function that pops node from the open list, and process the node and its neighbors based on the state. 
3. The **prepare_repair** function that senses the neighbors of the given node and locate the new obstacle nodes for cost modification.
4. The **modify_cost** function that modifies the cost from the obstacle node and its neighbor, and put them in the Open list for future search.
5. The **repair_replan** that replans a path from the current node to the goal

## Informed RRT*

1. c_best function - best length of the path when a path is found. 
2. Different sampling function based on the c_best value. 
3. ellipsoid sampling **get_new_point_in_ellipsoid**, so that when a path is found, the samples will be cast within an ellipsoid area for faster convergence.
