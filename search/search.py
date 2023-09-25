# search.py
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

def depthFirstSearch(problem):
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
    "*** YOUR CODE HERE ***"
    dfs_stack = util.Stack()
    visited_states = set()
    is_goal = False
    starting_tuple = (problem.getStartState(), [])
    dfs_stack.push(starting_tuple)
    goal_tuple = None
    #print("Start State: ", problem.getStartState())
    while not is_goal and dfs_stack:
        # checking if we ran out of possible nodes to check
        if dfs_stack.isEmpty():
            return "Failure"
        # popping off node of interest (Top of stack)
        parent_tuple = dfs_stack.pop()
        if problem.isGoalState(parent_tuple[0]):
            is_goal = True
            goal_tuple = parent_tuple
            # return parent's directions
        elif parent_tuple[0] not in visited_states:
            visited_states.add(parent_tuple[0])
            successors = problem.getSuccessors(parent_tuple[0])
            parent_state_path = parent_tuple[1]
            for child in successors:
                child_path = [] + parent_state_path
                child_path.append(child[1])
                child_tuple = (child[0], child_path)
                dfs_stack.push(child_tuple)

    if goal_tuple is not None:
        print("Result Actions: ", goal_tuple[1])
        return goal_tuple[1]
    else:
        print("NO RESULT")


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    bfs_queue = util.Queue()
    visited_states = set()
    is_goal = False
    starting_tuple = (problem.getStartState(), [])
    bfs_queue.push(starting_tuple)
    goal_tuple = None
    while not is_goal and bfs_queue:
        # checking if we ran out of possible nodes to check
        if bfs_queue.isEmpty():
            return "Failure"
        # popping off node of interest (Top of stack)
        parent_tuple = bfs_queue.pop()
        if problem.isGoalState(parent_tuple[0]):
            is_goal = True
            goal_tuple = parent_tuple
            # return parent's directions
        elif parent_tuple[0] not in visited_states:
            visited_states.add(parent_tuple[0])
            successors = problem.getSuccessors(parent_tuple[0])
            parent_state_path = parent_tuple[1]
            for child in successors:
                child_path = [] + parent_state_path
                child_path.append(child[1])
                child_tuple = (child[0], child_path)
                bfs_queue.push(child_tuple)

    if goal_tuple is not None:
        print("Result Actions: ", goal_tuple[1])
        return goal_tuple[1]
    else:
        print("NO RESULT")


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"

    ucs_queue = util.PriorityQueue()
    visited_states = set()
    is_goal = False
    starting_tuple = (problem.getStartState(), [], 0)
    ucs_queue.push(starting_tuple, starting_tuple[2])
    goal_tuple = None
    while not is_goal and ucs_queue:
        # checking if we ran out of possible nodes to check
        if ucs_queue.isEmpty():
            return "Failure"
        # popping off node of interest (Top of stack)
        parent_tuple = ucs_queue.pop()
        if problem.isGoalState(parent_tuple[0]):
            is_goal = True
            goal_tuple = parent_tuple
            # return parent's directions
        elif parent_tuple[0] not in visited_states:
            visited_states.add(parent_tuple[0])
            successors = problem.getSuccessors(parent_tuple[0])
            parent_state_path = parent_tuple[1]
            for child in successors:
                cost = child[2]
                child_path = [] + parent_state_path
                child_path.append(child[1])
                child_tuple = (child[0], child_path)
                ucs_queue.push(child_tuple,cost)

    if goal_tuple is not None:
        print("Result Actions: ", goal_tuple[1])
        return goal_tuple[1]
    else:
        print("NO RESULT")



def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    ucs_queue = util.PriorityQueue()
    return 0

#def aStar(node, problem, heuristic):
def aStar(aStarParams):
    # using for priority queue's function
    node = aStarParams[0]
    problem = aStarParams[1]
    heuristic = aStarParams[2]
    node_cost = node[2]
    heuristic_cost = heuristic(node, problem)
    cost = node_cost + heuristic_cost
    return cost


def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    astar_queue = util.PriorityQueueWithFunction(problem, astar)
    visited_states = set()
    is_goal = False
    starting_tuple = (problem.getStartState(), [], 0)
    astar_queue.push(starting_tuple)
    goal_tuple = None
    while not is_goal and astar_queue:
        # checking if we ran out of possible nodes to check
        if astar_queue.isEmpty():
            return "Failure"
        # popping off node of interest (Top of stack)
        parent_tuple = astar_queue.pop()
        if problem.isGoalState(parent_tuple[0]):
            is_goal = True
            goal_tuple = parent_tuple
            # return parent's directions
        elif parent_tuple[0] not in visited_states:
            visited_states.add(parent_tuple[0])
            successors = problem.getSuccessors(parent_tuple[0])
            parent_state_path = parent_tuple[1]
            for child in successors:
                child_path = [] + parent_state_path
                child_path.append(child[1])
                child_tuple = (child[0], child_path)
                astar_queue.push(child_tuple)

    if goal_tuple is not None:
        print("Result Actions: ", goal_tuple[1])
        return goal_tuple[1]
    else:
        print("NO RESULT")


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
