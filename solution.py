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
    storage_list = []
    width = state.width
    height = state.height
    obstacles = state.obstacles
    wallset = wall_set(width, height, obstacles)
    total_distance = 0
    unassigned_goals = set(state.storage)  # To ensure unique assignment of goals
    for i in state.boxes:
        if i not in state.storage and corner_detection(i, wallset, width, height):
            return float('inf')
    
    for i in state.boxes:
        if i not in state.storage and box_adjacent_deadlock(i, state.boxes, wallset, state.obstacles):
            return float('inf')

    for box in state.boxes:
        min_distance = float('inf')
        closest_goal = None

        for goal in unassigned_goals:
            dist = abs(box[0] - goal[0]) + abs(box[1] - goal[1])  
            if dist < min_distance:
                min_distance = dist
                closest_goal = goal
        
        if closest_goal:
            unassigned_goals.remove(closest_goal)  # Assign goal uniquely
            total_distance += min_distance

    # heur_manhattan_distance has flaws.
    # Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
    # Your function should return a numeric value for the estimate of the distance to the goal.
    # EXPLAIN YOUR HEURISTIC IN THE COMMENTS. Please leave this function (and your explanation) at the top of your solution file, to facilitate marking.
    return total_distance  # CHANGE THIS

def box_adjacent_deadlock(box, boxes, wall_set, obstacles):

    x, y = box
    # Check adjacent boxes
    neighbors = [(x-1, y), (x+1, y), (x, y-1), (x, y+1)]

    for nx, ny in neighbors:
        if (nx, ny) in boxes:
            # If the box is adjacent to another box, check if it's stuck
            # Check if both are against a wall or obstacle
            if (x-1, y) in wall_set or (x+1, y) in wall_set:
                if (nx-1, ny) in wall_set or (nx+1, ny) in wall_set:
                    return True  # Stuck horizontally

            if (x, y-1) in wall_set or (x, y+1) in wall_set:
                if (nx, ny-1) in wall_set or (nx, ny+1) in wall_set:
                    return True  # Stuck vertically
            
    return False

def corner_detection(box, wall_set, width, height):
    x = box[0]
    y = box[1]
    if ((x-1, y) in wall_set or (x+1, y) in wall_set) and ((x, y-1) in wall_set or (x, y+1) in wall_set):
        return True
    else:
        return False

def wall_set(width, height, obstacles):
    walls = list(obstacles)
    for i in range(width+1):
        walls.append((i, -1))
        walls.append((i,height))
    for j in range(height+1):
        walls.append((-1,j))
        walls.append((width,j))
    return tuple(set(walls))



s = PROBLEMS[11]
print(heur_alternate(s))


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
        goal_found,stats = greedy_search_engine.search(timebound=timebound, costbound=costbound)
        if goal_found and goal_found.gval < costbound[0]:
            costbound = (goal_found.gval, costbound[1], costbound[2])
            best = goal_found
            best_stats = stats

            
    if best == None:
        return False, None
    
    else:
        return best, best_stats


