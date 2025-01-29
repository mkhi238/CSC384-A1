#   Look for #IMPLEMENT tags in this file. These tags indicate what has
#   to be implemented to complete the warehouse domain.

#   You may add only standard python imports
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

import os  # for time functions
import math  # for infinity
from search import *  # for search engines
from sokoban import sokoban_goal_state, SokobanState, Direction, PROBLEMS  # for Sokoban specific classes and problems

# SOKOBAN HEURISTICS
def heur_alternate(state):
    # IMPLEMENT
    '''a better heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    # heur_manhattan_distance has flaws.
    # Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
    # Your function should return a numeric value for the estimate of the distance to the goal.
    # EXPLAIN YOUR HEURISTIC IN THE COMMENTS. Please leave this function (and your explanation) at the top of your solution file, to facilitate marking.


    #1. SETTING PARAMETERS
    width = state.width             #width of sokoban board
    height = state.height           #height of sokoban board
    obstacles = state.obstacles     #location of obstacles on board
    wallset = wall_set(width, height, obstacles)       #function that creates coordinates of all walls
    total_distance = 0              #total distance of box to goal
    boxes = state.boxes             #location of boxes on board
    unassigned_goals = set(state.storage)   #list of all goals that are unassigned (all goals initally)


    #2. CHECKING FOR DEAD STATES
    #First check if a box will be stuck in a corner by walls and NOT in Storage
    for i in state.boxes:
        if i not in state.storage and corner_detection(i, wallset, width, height) == True:
                return float('inf') #if stuck (dead state), impossible to solve
    
    #Second check if a box will be stuck in a corner by another box and NOT in Storage
    for i in state.boxes:
        if i not in state.storage and boxes_stuck(i, boxes, wallset, state.storage) == True:
            return float('inf')     #if stuck (dead state), impossible to solve
    
    #Third check if a box will be stuck in a corner by an obstacle and NOT in Storage
    for i in state.boxes:
        if i not in state.storage and obs_stuck(i, wallset, obstacles) == True:
            return float('inf')     #if stuck (dead state), impossible to solve

    #3. CALCULATING MANHATTAN DISTANCE
    for box in state.boxes:
        min_distance = 100000000    #Set the minimum distance to large number
        closest_goal = None         #Set closest goal to None

        for goal in unassigned_goals:
            dist = abs(box[0] - goal[0]) + abs(box[1] - goal[1])    #Calculate Manhattan distance

            #Greedily assign boxes to the nearest goal state
            if dist < min_distance:                                 
                min_distance = dist
                closest_goal = goal     
        
        #if a closest goal is found (i.e we are not in a dead or solution state)
        if closest_goal:
            unassigned_goals.remove(closest_goal)  # Remove a goal state from the set of unassigned goals 
            total_distance += min_distance         # Add the smallest distance to the total distance required to be moved

    #4. UPDATE TOTAL DISTANCE WITH ROBOT DISTANCES
    total_distance += robot_box_distance(state)    #Find distance from the robot to the box and add to total distance (just updating the heuristic value, still greedily only accounting for box to goal distance in calculation)
    total_distance += box_clustering_penalty(boxes, state.storage) #Add a cluster score to disincentive box clustering (to lower likelihood of entering infeasible states)
    return total_distance 


#BOX CLUSTERING PENALTY CALCULATION
def box_clustering_penalty(boxes, storage):
    unplaced_boxes = [box for box in boxes if box not in storage]   #Creates a list of unplaced boxes
    penalty = 0
    #Iterate through the unplaced boxes and for any two unique boxes, if they are within 1 then assign a large penalty
    for i in unplaced_boxes:
        for j in unplaced_boxes:
                if i != j:
                    if abs(i[0] - j[0]) <= 1 and abs(i[1] - j[1]) <= 1:
                        penalty += 100000000
    return penalty

#FUNCTION TO ENUMERATE THE LOCATION OF ALL WALLS AND OBSTACLES
def wall_set(width, height, obstacles):
    walls = list(obstacles) #Treat Obstacles as walls

    #Playable area in Sokoban is width + 2 (wall on either side parallel verticlaly) and height + 2 (wall on either side parallel horizonally)
    #For example, a 5x5 arena is 7x7 with the walls included

    # Eunumerate horizontal walls
    for i in range(width+1): 
        walls.append((i, -1))    #In 5x5 example, this would be like (0,-1), (1,-1), ...
        walls.append((i,height)) #In 5x5 example, this would be like (0,5), (1,5), ...

    #Enumerate vertical walls
    for j in range(height+1):   
        walls.append((-1,j))    #In 5x5 example, this would be like (-1,0), (-1,1), ...
        walls.append((width,j)) #In 5x5 example, this would be like (5,0), (5,1), ...
    
    #Give a tuple of the unique wall enumerations
    return tuple(set(walls))


#FUNCTION TO DETERMINE IF 2 BOXES ARE CREATING A DEAD STATE WITH A CORNER OR OBSTACLE
def boxes_stuck(box, boxes, wall_set, storage):
    x = box[0]  #x coordinate
    y = box[1]  #y coordinate
    
    #CORNER CHECKING DUE TO BOXES

    #Left is (x-1, y), if there is a box to the left, cannot move to the left
    #Check if there is wall directly below and to the left diagonal, if so we are in a dead state
    left_box = (x-1, y) in boxes and (x, y-1) in wall_set and (x-1, y-1) in wall_set

    #Right is (x+1, y), if there is a box to the right, cannot move to the right
    #Check if there is wall directly below and to the right diagonal, if so we are in a dead state
    right_box = (x+1, y) in boxes and (x, y-1) in wall_set and (x+1, y-1) in wall_set

    #Top is (x, y-1), if there is a box to the top, cannot move to the top
    #Check if there is wall to the left and to the left diagonal, if so we are in a dead state
    top_box = (x, y-1) in boxes and (x-1, y) in wall_set and (x-1, y-1) in wall_set

    #Bottom is (x, y+1), if there is a box to the bottom, cannot move to the bottom
    #Check if there is wall to the left and to the left diagonal, if so we are in a dead state
    bottom_box = (x, y+1) in boxes and (x-1, y) in wall_set and (x-1, y+1) in wall_set

    return (left_box or right_box or top_box or bottom_box)

def obs_stuck(box, wall_set, obstacles):
    x = box[0]  #x coordinate
    y = box[1]  #y coordinate
    
    #CORNER CHECKING DUE TO BOXES

    #Left is (x-1, y), if there is an obstacle to the left, cannot move to the left
    #Check if there is wall directly below and to the left diagonal, if so we are in a dead state
    left_box = (x-1, y) in obstacles and (x, y-1) in wall_set and (x-1, y-1) in wall_set

    #Right is (x+1, y), if there is an obstacle to the right, cannot move to the right
    #Check if there is wall directly below and to the right diagonal, if so we are in a dead state
    right_box = (x+1, y) in obstacles and (x, y-1) in wall_set and (x+1, y-1) in wall_set

    #Top is (x, y-1), if there is an obstacle to the top, cannot move to the top
    #Check if there is wall to the left and to the left diagonal, if so we are in a dead state
    top_box = (x, y-1) in obstacles and (x-1, y) in wall_set and (x-1, y-1) in wall_set

    #Bottom is (x, y+1), if there is an obstacle to the bottom, cannot move to the bottom
    #Check if there is wall to the left and to the left diagonal, if so we are in a dead state
    bottom_box = (x, y+1) in obstacles and (x-1, y) in wall_set and (x-1, y+1) in wall_set

    return (left_box or right_box or top_box or bottom_box)


#FUNCTION TO DETECT IF A BOX IS NEAR A CORNER (CREATED BY WALLS)
def corner_detection(box, wall_set, width, height):

    x = box[0]
    y = box[1]

    #Similar to above checks, but simple check to see if there the box is generally near a corner to the top left
    top_left = (x-1, y) in wall_set and (x, y-1) in wall_set    
    #Similar to above checks, but simple check to see if there the box is generally near a corner to the top right
    top_right = (x+1, y) in wall_set and (x, y-1) in wall_set   
    #Similar to above checks, but simple check to see if there the box is generally near a corner to the bottom left
    bottom_left = (x-1, y) in wall_set and (x, y+1) in wall_set  
    #Similar to above checks, but simple check to see if there the box is generally near a corner to the bottom right
    bottom_right = (x+1, y) in wall_set and (x, y+1) in wall_set 
    
    return top_left or top_right or bottom_left or bottom_right

#FUNCTION 
def robot_box_distance(state):

    unplaced_boxes = [box for box in state.boxes if box not in state.storage] #Creates a list of unplaced boxes

    if len(unplaced_boxes) == 0:
        return 0
    
    min_dist = float('inf')
    for robot in state.robots:
        for box in unplaced_boxes:
            #Manhattan Distance for Robot To Box - identical calculation to above
            dist = abs(robot[0] - box[0]) + abs(robot[1] - box[1])
            if dist < min_dist:
                min_dist = dist
    return min_dist


def heur_zero(state):
    '''Zero Heuristic can be used to make A* search perform uniform cost search'''
    return 0

def heur_manhattan_distance(state):
    # IMPLEMENT
    '''admissible sokoban puzzle heuristic: manhattan distance'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    manhattan_dist = 0
    for i in state.boxes:
        dist = min(abs(i[0]-j[0]) + abs(i[1]-j[1]) for j in state.storage)
        manhattan_dist += dist
    return manhattan_dist
    # We want an admissible heuristic, which is an optimistic heuristic.
    # It must never overestimate the cost to get from the current state to the goal.
    # The sum of the Manhattan distances between each box that has yet to be stored and the storage point nearest to it is such a heuristic.
    # When calculating distances, assume there are no obstacles on the grid.
    # You should implement this heuristic function exactly, even if it is tempting to improve it.
    # Your function should return a numeric value; this is the estimate of the distance to the goal.



def fval_function(sN, weight):
    # IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
    fval = sN.gval + weight*sN.hval
    return fval

# SEARCH ALGORITHMS
def weighted_astar(initial_state, heur_fn, weight, timebound):
   
    '''Provides an implementation of weighted a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False as well as a SearchStats object'''
    '''implementation of weighted astar algorithm'''
    search_engine = SearchEngine(strategy='custom',cc_level='full')
    wrapped_fval_function = (lambda sN: fval_function(sN, weight))
    search_engine.init_search(initial_state, sokoban_goal_state, heur_fn, wrapped_fval_function)
    goal_found, stats = search_engine.search(timebound=timebound)

    return goal_found, stats  



def iterative_astar(initial_state, heur_fn, weight=1, timebound=5):  # uses f(n), see how autograder initializes a search line 88
    # IMPLEMENT
    '''Provides an implementation of realtime a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False as well as a SearchStats object'''
    '''implementation of iterative astar algorithm'''
    search_engine = SearchEngine(strategy = 'custom', cc_level = 'full')
    wrapped_fval_function = (lambda sN: fval_function(sN, weight))
    search_engine.init_search(initial_state, sokoban_goal_state, heur_fn, wrapped_fval_function)
    costbound = (100000000,100000000,100000000)
    start_time = os.times()[0]
    best = None
    best_stats = None

    while timebound > os.times()[0] - start_time and weight >= 1:
        goal_found, stats = search_engine.search(timebound = timebound, costbound=costbound)
        if goal_found and goal_found.gval < costbound[0]:
            costbound = (goal_found.gval, heur_fn(goal_found), goal_found.gval + weight*heur_fn(goal_found))
            best = goal_found
            best_stats = stats
    
    if best == None:
        return False, None
    
    else:
        return best, best_stats
    

def iterative_gbfs(initial_state, heur_fn, timebound=5):  # only use h(n)
    # IMPLEMENT
    '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    '''implementation of iterative gbfs algorithm'''
    greedy_search_engine = SearchEngine(strategy = 'best_first' )
    greedy_search_engine.init_search(initial_state,sokoban_goal_state,heur_fn)
    costbound = (100000000, 100000000, 100000000)
    start_time = os.times()[0]
    best = None
    best_stats = None

    while timebound > os.times()[0] - start_time:
        goal_found, stats = greedy_search_engine.search(timebound=timebound, costbound=costbound)
        if goal_found and goal_found.gval < costbound[0]:
            costbound = (goal_found.gval, costbound[1], costbound[2])
            best = goal_found
            best_stats = stats
            
    if best == None:
        return False, None
    
    else:
        return best, best_stats


