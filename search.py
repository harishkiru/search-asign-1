# search.py
# Assignment 1:
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
#
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).

"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem: SearchProblem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE *** (Q1)"

    # Init. state and path stacks
    state = util.Stack()
    path = util.Stack()

    # Init. state and path lists
    stateList = []
    pathList = []

    # Push start state and an empty path into their respective stacks
    state.push(problem.getStartState())
    path.push([])

    # Continue to iterate while stack is not empty
    while not state.isEmpty():
        currentState = state.pop()
        currentPath = path.pop()

        # Check is the current state is the goal state
        if problem.isGoalState(currentState):
            # Return the path if the goal state is achieved
            return currentPath

        # Check if the current state has already been visited
        if currentState not in stateList:
            stateList.append(currentState)
            pathList.append(currentPath)

            # Add the successor state and its respective path into the stack
            for successor in problem.getSuccessors(currentState):
                state.push(successor[0])
                path.push(currentPath + [successor[1]])

    #Return empty list if no valid paths are found
    return []

def breadthFirstSearch(problem: SearchProblem):
    """Search the shallowest nodes in the search tree first."""

    "*** YOUR CODE HERE *** (Q2)"
    from util import Queue
    
    q = Queue()  
    
    
    start_state = problem.getStartState()
    q.push((start_state, [])) 
    
    occupied = set()

    while not q.isEmpty():
        state, path = q.pop()
        if state in occupied:
            continue
        occupied.add(state)

        if problem.isGoalState(state):
            return path

        for successor, action, _ in problem.getSuccessors(state):

            
            if successor not in occupied:
                q.push((successor, path + [action]))

    return []

    util.raiseNotDefined()

def uniformCostSearch(problem: SearchProblem):
    """Search the node of least total cost first."""

    "*** YOUR CODE HERE *** (Q3)"


    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""

    "*** YOUR CODE HERE *** (Q4)"
    import util
    from util import PriorityQueue
    pq = PriorityQueue()
    start_state = problem.getStartState()
    pq.push((start_state, [], 0), 0) 
    
    occupied = {}
    while not pq.isEmpty():   
        state, path, cost = pq.pop()
        if state in occupied and occupied[state] <= cost:
            continue
        occupied[state] = cost 

        if problem.isGoalState(state):
            return path

        for successor, action, step_cost in problem.getSuccessors(state):
            new_cost = cost + step_cost  
            
            priority = new_cost + heuristic(successor, problem)  
            pq.push((successor, path + [action], new_cost), priority)
    return []  
#util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
