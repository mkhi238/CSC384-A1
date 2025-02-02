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

    '''
    EXPLANATION OF MY HEURISTIC:
    The way my heurisitic works is that I try to check 3 different potential dead states first:
    1. If a box is getting stuck in a corner while not being in a storage space
    2. If a box is getting stuck in a corner made by another box and 2 walls
    3. If a box is getting stuck in a corner made by an obstacle and 2 walls

    To get the walls themselves, I use a helper function to enumerate the coordinates of all the walls. I then use the coordinates of the boxes and just do equivalence checks
    to see if the coordintes of a space neigbouring a box is either a box, a dead corner, or an obstacle. If so, my heuristic returns 1000000000, which ensures that it does not 
    make said move. 

    Following this, my function will find the Manhattan distance from all remaining boxes, and select the box to move with the closest space (similar to the regular Manhattan distance
    we made before)

    Finally, my function adds in the distance to move the robot to the box, such that although the idea is to greedily select the closest box, boxes too far away will be discouraged. 

    I believe my heuristic to be admissable since there is never a point where I am overestimating the cost of the solution such that it would be greater than the optimal cost. 
    Every check only checks for infeasibly states and adds in distances to a goal state, and doesnt include anything else. 
    '''

    #1. SETTING PARAMETERS
    width = state.width             #width of sokoban board
    height = state.height           #height of sokoban board
    obstacles = state.obstacles     #location of obstacles on board
    wallset = wall_set(width, height, obstacles)       #function that creates coordinates of all walls
    storage = state.storage
    total_distance = 0              #total distance of box to goal
    boxes = state.boxes             #location of boxes on board
    unassigned_goals = set(state.storage)   #list of all goals that are unassigned (all goals initally)


    #2. CHECKING FOR DEAD STATES
    #First check if a box will be stuck in a corner by walls and NOT in Storage
    for i in state.boxes:
        if i not in state.storage and corner_detection(i, wallset, width, height) == True:
                return 1000000000 #if stuck (dead state), impossible to solve
    
    #Second check if a box will be stuck in a corner by another box and NOT in Storage
    for i in state.boxes:
        if i not in state.storage and boxes_stuck(i, boxes, wallset, storage) == True:
            return 1000000000     #if stuck (dead state), impossible to solve
    
    #Third check if a box will be stuck in a corner by an obstacle and NOT in Storage
    for i in state.boxes:
        if i not in state.storage and obs_stuck(i, wallset, obstacles) == True:
            return 1000000000    #if stuck (dead state), impossible to solve
        

    #3. CALCULATING MANHATTAN DISTANCE
    for box in state.boxes:
        min_distance = 1000000000    #Set the minimum distance to large number
        closest_goal = 0     

        for goal in unassigned_goals:
            dist = abs(box[0] - goal[0]) + abs(box[1] - goal[1])    #Calculate Manhattan distance

            if box_blocking_goal(box, goal, boxes, obstacles):
                dist += 1  

            #Greedily assign boxes to the nearest goal state
            if dist < min_distance:                                 
                min_distance = dist
                closest_goal = goal     
        
        #if a closest goal is found (i.e we are not in a dead or solution state)
        unassigned_goals.remove(closest_goal)  # Remove the assigned goal only if it's in the set

        total_distance += min_distance         # Add the smallest distance to the total distance required to be moved
        

    #4. UPDATE TOTAL DISTANCE WITH ROBOT DISTANCES
    total_distance += distance_from_robot_to_box(state) #Find distance from the robot to the box and add to total distance (just updating the heuristic value, still greedily only accounting for box to goal distance in calculation)
    return total_distance 

def box_blocking_goal(box, goal, boxes, obstacles):
    x1, y1 = box
    x2, y2 = goal

    if y1 == y2:
        for x in range(min(x1, x2) + 1, max(x1, x2)):  # Exclude box and goal
            if (x, y1) in boxes or (x,y1) in obstacles:
                return True

    # Check vertical block (Same column, different rows)
    if x1 == x2:
        for y in range(min(y1, y2) + 1, max(y1, y2)):  # Exclude box and goal
            if (x1, y) in boxes or (x1, y) in obstacles:
                return True

    return False  # No blocking box found


#FUNCTION TO ENUMERATE THE LOCATION OF ALL WALLS AND OBSTACLES
def wall_set(width, height, obstacles):
    walls = list(obstacles) #Treat Obstacles as walls

    #Playable area in Sokoban is width + 2 (wall on either side=]
    #  parallel verticlaly) and height + 2 (wall on either side parallel horizonally)
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


#FUNCTION TO DETERMINE IF 2 BOXES ARE CREATING A DEAD STATE WITH ANOTHER BOX
def boxes_stuck(box, boxes, wall_set, storage):
    x = box[0]  #x coordinate
    y = box[1]  #y coordinate
    
    #CORNER CHECKING DUE TO BOXES

    #LEFT BOXES (LB); (x-1, y)
    #Left is (x-1, y), if there is a box to the left, cannot move to the left

    #Check if there are walls directly below or above this left box; if so, we are in a dead state
    LB_walls_above = (x-1, y) in boxes and (x, y-1) in wall_set and (x-1, y-1) in wall_set
    LB_walls_below = (x-1, y) in boxes and (x, y+1) in wall_set and (x-1, y+1) in wall_set


    #RIGHT BOXES (RB); (x+1, y)
    #Right is (x+1, y), if there is a box to the right, cannot move to the right

    #Check if there are walls directly below or above this right box; if so, we are in a dead state
    RB_walls_above = (x+1, y) in boxes and (x, y-1) in wall_set and (x+1, y-1) in wall_set
    RB_walls_below = (x+1, y) in boxes and (x, y+1) in wall_set and (x+1, y+1) in wall_set

    #TOP BOXES (TB); (x, y-1)
    #Top is (x, y-1), if there is a box to the top, cannot move to the top

    #Check if there is wall to the left or right of this top box, if so we are in a dead state
    TB_walls_left = (x, y-1) in boxes and (x-1, y) in wall_set and (x-1, y-1) in wall_set
    TB_walls_right = (x, y-1) in boxes and (x+1, y) in wall_set and (x+1, y-1) in wall_set

    #BOTTOM BOXES (BB); (x, y+1)
    #Bottom is (x, y+1), if there is a box to the bottom, cannot move to the bottom
    #Check if there is wall to the left or right of this bottom box, if so we are in a dead state
    BB_walls_left = (x, y+1) in boxes and (x-1, y) in wall_set and (x-1, y+1) in wall_set
    BB_walls_right = (x, y+1) in boxes and (x+1, y) in wall_set and (x+1, y+1) in wall_set


    #Return true if any of these is the case, otherwise false
    return (LB_walls_above or LB_walls_below or RB_walls_above or RB_walls_below or TB_walls_left or TB_walls_right or BB_walls_left or BB_walls_right)



#FUNCTION TO DETERMINE IF A BOXES IS CREATING A DEAD STATE WITH AN OBSTACLE
def obs_stuck(box, wall_set, obstacles):
    x = box[0]  #x coordinate
    y = box[1]  #y coordinate
    
    #CORNER CHECKING DUE TO OBSTACLES

    #LEFT OBSTACLES (LO); (x-1, y)
    #Left is (x-1, y), if there is a obstacle to the left, cannot move to the left

    #Check if there are walls directly below or above this left obstacle; if so, we are in a dead state
    LO_walls_above = (x-1, y) in obstacles and (x, y-1) in wall_set and (x-1, y-1) in wall_set
    LO_walls_above2 = (x-1, y) in obstacles and (x, y-1) in obstacles and (x-1, y-1) in wall_set
    LO_walls_above3 = (x-1, y) in obstacles and (x, y-1) in obstacles and (x-1, y-1) in obstacles
    LO_walls_below = (x-1, y) in obstacles and (x, y+1) in wall_set and (x-1, y+1) in wall_set
    LO_walls_below2 = (x-1, y) in obstacles and (x, y+1) in obstacles and (x-1, y+1) in wall_set
    LO_walls_below3 = (x-1, y) in obstacles and (x, y+1) in obstacles and (x-1, y+1) in obstacles

    #RIGHT OBSTACLES (RO); (x+1, y)
    #Right is (x+1, y), if there is a obstacle to the right, cannot move to the right

    #Check if there are walls directly below or above this right obstacle; if so, we are in a dead state
    RO_walls_above = (x+1, y) in obstacles and (x, y-1) in wall_set and (x+1, y-1) in wall_set
    RO_walls_above2 = (x+1, y) in obstacles and (x, y-1) in obstacles and (x+1, y-1) in wall_set
    RO_walls_above3 = (x+1, y) in obstacles and (x, y-1) in obstacles and (x+1, y-1) in obstacles
    RO_walls_below = (x+1, y) in obstacles and (x, y+1) in wall_set and (x+1, y+1) in wall_set
    RO_walls_below2 = (x+1, y) in obstacles and (x, y+1) in obstacles and (x+1, y+1) in wall_set
    RO_walls_below3 = (x+1, y) in obstacles and (x, y+1) in obstacles and (x+1, y+1) in obstacles

    #TOP OBSTACLES (TO); (x, y-1)
    #Top is (x, y-1), if there is a obstacle to the top, cannot move to the top

    #Check if there is wall to the left or right of this top obstacle, if so we are in a dead state
    TO_walls_left = (x, y-1) in obstacles and (x-1, y) in wall_set and (x-1, y-1) in wall_set
    TO_walls_left2 = (x, y-1) in obstacles and (x-1, y) in obstacles and (x-1, y-1) in wall_set
    TO_walls_left3 = (x, y-1) in obstacles and (x-1, y) in obstacles and (x-1, y-1) in obstacles
    TO_walls_right = (x, y-1) in obstacles and (x+1, y) in wall_set and (x+1, y-1) in wall_set
    TO_walls_right2 = (x, y-1) in obstacles and (x+1, y) in obstacles and (x+1, y-1) in wall_set
    TO_walls_right3 = (x, y-1) in obstacles and (x+1, y) in obstacles and (x+1, y-1) in obstacles

    #BOTTOM OBSTACLES (BO); (x, y+1)
    #Bottom is (x, y+1), if there is a box to the bottom, cannot move to the bottom
    #Check if there is wall to the left or right of this bottom obstacle, if so we are in a dead state
    BO_walls_left = (x, y+1) in obstacles and (x-1, y) in wall_set and (x-1, y+1) in wall_set
    BO_walls_left2 = (x, y+1) in obstacles and (x-1, y) in obstacles and (x-1, y+1) in wall_set
    BO_walls_left3 = (x, y+1) in obstacles and (x-1, y) in obstacles and (x-1, y+1) in obstacles
    BO_walls_right = (x, y+1) in obstacles and (x+1, y) in wall_set and (x+1, y+1) in wall_set
    BO_walls_right2 = (x, y+1) in obstacles and (x+1, y) in obstacles and (x+1, y+1) in wall_set
    BO_walls_right3 = (x, y+1) in obstacles and (x+1, y) in obstacles and (x+1, y+1) in obstacles

    #Return true if any of these is the case, otherwise false
    return (LO_walls_above or LO_walls_below or RO_walls_above or RO_walls_below or TO_walls_left or TO_walls_right or BO_walls_left or BO_walls_right,
            LO_walls_above2 or LO_walls_below2 or RO_walls_above2 or RO_walls_below2 or TO_walls_left2 or TO_walls_right2 or BO_walls_left2 or BO_walls_right2,
            LO_walls_above3 or LO_walls_below3 or RO_walls_above3 or RO_walls_below3 or TO_walls_left3 or TO_walls_right3 or BO_walls_left3 or BO_walls_right3)

def get_minimal_detour(box, goal, obstacles, boxes):

    detour_cost = 0
    x1, y1 = box
    x2, y2 = goal
    
    # Check if any obstacle is directly in the path
    for x in range(min(x1, x2), max(x1, x2) + 1):
        for y in range(min(y1, y2), max(y1, y2) + 1):
            if (x, y) in obstacles or ((x, y) in boxes and (x, y) != box):
                # If obstacle is in direct path, must take at least 1 step detour
                detour_cost = 1
                break
    return detour_cost

#FUNCTION TO DETECT IF A BOX IS NEAR A CORNER (CREATED BY WALLS)
def corner_detection(box, wall_set, width, height):

    x = box[0]
    y = box[1]

    #im using this function for the general case we are near a corner, hence why it it simpler and also only has 4 outcomes

    #Similar to above checks, but simple check to see if there the box is generally near a corner to the top left
    top_left_corner = (x-1, y) in wall_set and (x, y-1) in wall_set and (x-1, y-1) in wall_set
    #Similar to above checks, but simple check to see if there the box is generally near a corner to the top right
    top_right_corner = (x+1, y) in wall_set and (x, y-1) in wall_set  and (x+1, y-1) in wall_set 
    #Similar to above checks, but simple check to see if there the box is generally near a corner to the bottom left
    bottom_left_corner = (x-1, y) in wall_set and (x, y+1) in wall_set  and (x-1, y+1) in wall_set
    #Similar to above checks, but simple check to see if there the box is generally near a corner to the bottom right
    bottom_right_corner = (x+1, y) in wall_set and (x, y+1) in wall_set and (x+1, y+1) in wall_set
    
    #Return true if any of these is the case, otherwise false
    return top_left_corner or top_right_corner or bottom_left_corner or bottom_right_corner


#PENALTY:
# 1. I add in the cost of moving the robot from its state to the box it is greedily required to go to; even thought we are greedily selecting by the box to goal state 
#    distance, I found that adding in this distance was helpful in reaching better states as well

#FUNCTION TO FIND ROBOT'S DISTANCE FROM BOX
def distance_from_robot_to_box(state):

    #Create a list of unplaced boxes
    unplaced_boxes = []
    for box in state.boxes:
        if box not in state.storage:
            unplaced_boxes.append(box)

    #If there is no unplaced boxes for whatever reason
    if len(unplaced_boxes) == 0:
        return 0
    
    #Iterate through robots, find the robot with the minimum distance and add it to the calculation
    min_dist = 1000000000

    for robot in state.robots: #For all robots
        for box in unplaced_boxes: #For all boxes
            #Manhattan Distance for Robot To Box - identical calculation to above
            dist = abs(robot[0] - box[0]) + abs(robot[1] - box[1])
            if dist < min_dist:
                min_dist = dist
    return min_dist

def heur_zero(state):
    # We want an admissible heuristic, which is an optimistic heuristic.
    # It must never overestimate the cost to get from the current state to the goal.
    # The sum of the Manhattan distances between each box that has yet to be stored and the storage point nearest to it is such a heuristic.
    # When calculating distances, assume there are no obstacles on the grid.
    # You should implement this heuristic function exactly, even if it is tempting to improve it.
    # Your function should return a numeric value; this is the estimate of the distance to the goal.

    '''Zero Heuristic can be used to make A* search perform uniform cost search'''
    return 0

def heur_manhattan_distance(state):
    # IMPLEMENT
    '''admissible sokoban puzzle heuristic: manhattan distance'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    manhattan_dist = 0
    #Iterate through all boxes, and find the minimum Manhattan distance between each box and the closes storage stace
    for i in state.boxes:
        dist = min(abs(i[0]-j[0]) + abs(i[1]-j[1]) for j in state.storage)
        manhattan_dist += dist #Sum all distances for all boxes
    return manhattan_dist


def fval_function(sN, weight):
    # IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
    #fval function is g(n) + weight * h(n)
    fval = sN.gval + weight*sN.hval         
    return fval

# SEARCH ALGORITHMS
def weighted_astar(initial_state, heur_fn, weight, timebound):
   
    '''Provides an implementation of weighted a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False as well as a SearchStats object'''
    '''implementation of weighted astar algorithm'''
    search_engine = SearchEngine(strategy='custom',cc_level='full') #Initialize search engine
    wrapped_fval_function = (lambda sN: fval_function(sN, weight))  #Create the wrapped fval function as specified in Section 6.0
    search_engine.init_search(initial_state, sokoban_goal_state, heur_fn, wrapped_fval_function) #Pass the wrapped  function as the f_val
    goal_found, stats = search_engine.search(timebound=timebound) #search

    return goal_found, stats  



def iterative_astar(initial_state, heur_fn, weight=1, timebound=5):  # uses f(n), see how autograder initializes a search line 88
    # IMPLEMENT
    '''Provides an implementation of realtime a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False as well as a SearchStats object'''
    '''implementation of iterative astar algorithm'''
    search_engine = SearchEngine(strategy = 'custom', cc_level = 'full')
    #Inialize costbound (g(n), h(n), f(n)) with some arbitariliy high numbers, as defined in section 3.0
    costbound = (100000000,100000000,100000000) 
    weight = 15
    start_time = os.times()[0]  #intialize start time
    best = None #initalize best solution (none so far)
    best_stats = None #initalize best stats (none so far)
    #Run iterative astar for as long as we have time and as long as the weight is greater than 1
    while timebound > os.times()[0] - start_time and weight >= 1:
        wrapped_fval_function = (lambda sN: fval_function(sN, weight))
        search_engine.init_search(initial_state, sokoban_goal_state, heur_fn, wrapped_fval_function)
        goal_found, stats = search_engine.search(timebound = timebound, costbound=costbound)    #Search for solution 
        #if the gval found is found and is better than our current gval, we want to set the costbounds g(n), h(n), and f(n) values to the found solution
        if goal_found and goal_found.gval < costbound[0]:
            #Update costbound to new values
            costbound = (goal_found.gval, heur_fn(goal_found), goal_found.gval + weight*heur_fn(goal_found))
            best = goal_found #Update best solution
            best_stats = stats #Update best solution stats
            remaining_time = timebound - (os.times()[0] - start_time)
            weight *= 0.7  # Reduce weight slower
        
        else:
            weight *= 0.5  # Reduce weight aggressively
    
    #If we done find a soluiton in time
    if best == None:
        return False, None
    
    #Otherwise, return the solution
    else:
        return best, best_stats
    

def iterative_gbfs(initial_state, heur_fn, timebound=5):  # only use h(n)
    # IMPLEMENT
    '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    '''implementation of iterative gbfs algorithm'''
    greedy_search_engine = SearchEngine(strategy = 'best_first') #strategy set to best first, meaning only h(n) - heuristic value is used
    greedy_search_engine.init_search(initial_state,sokoban_goal_state,heur_fn) #inialize search
    #Inialize costbound (g(n), h(n), f(n)) with some arbitariliy high numbers, as defined in section 3.0
    costbound = (100000000, 100000000, 100000000) 
    start_time = os.times()[0] #intialize start time
    best = None #initalize best solution (none so far)
    best_stats = None #initalize best stats (none so far)

    #while we still have time to do our search
    while timebound > os.times()[0] - start_time:
        
        goal_found, stats = greedy_search_engine.search(timebound=timebound, costbound=costbound) #search
        #if we find a goal and the goal has a better gval than our best solution
        if goal_found and goal_found.gval < costbound[0]:
            costbound = (goal_found.gval, heur_fn(goal_found), goal_found.gval+heur_fn(goal_found))   #update the gval (only the gval is important here)

            #update best solution and stats to the new best solution
            best = goal_found
            best_stats = stats

    #If we done find a soluiton in time       
    if best == None:
        return False, None
    
    #Otherwise, return the solution
    else:
        return best, best_stats


