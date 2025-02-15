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
    return [s, s, w, s, w, w, s, w]


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
    from game import Directions
    n = Directions.NORTH
    s = Directions.SOUTH
    e = Directions.EAST
    w = Directions.WEST

    closed = list()
    open = []
    current = Node2(problem.getStartState(),None,0,None)

    count = 0

    while not problem.isGoalState(current.crd):
        if(count !=0):
            current = open.pop()
        if problem.isGoalState(current.crd):
            break
        count += 1
        if current not in closed:
            closed.append(current)
        for successor in problem.getSuccessors(current.crd):
            successor = Node2(successor[0],successor[1],successor[2]+current.cost,current)
            if successor not in closed:
                open.append(successor)
    path = list()
    path1 = list()

    while current.parent:
        path1.insert(0,current)
        dir=stateToDir(current)
        path.insert(0,current.dir)
        current = current.parent

    return path


def isAdj(current, prev):
    flag = 0
    if current[0] == prev[0] - 1 and current[1] == prev[1]:
        flag = 1
    elif (current[0] == prev[0] + 1 and current[1] == prev[1]):
        flag = 1
    elif (current[1] == prev[1] - 1 and current[0] == prev[0]):
        flag = 1
    elif (current[1] == prev[1] + 1 and current[0] == prev[0]):
        flag = 1

    return flag


class Node:
    def __init__(self, state, action, cost, parent):
        self.state = state
        self.action = action
        self.cost = cost
        self.parent = parent


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    from util import Queue
    current = Node(problem.getStartState(), '', 0, None)  # initialize state to start state in problem
    closed = []
    open = Queue()  # use queue data structure from util.py for BFS
    while not problem.isGoalState(current.state):  # loop until current state is the goal state
        if current.state not in closed:
            closed.append(current.state)  # add current state to closed list
        successors = problem.getSuccessors(current.state)  # retrieve successor state list
        for s in successors:  # add successors not in closed to open
            if s[0] not in closed:
                new = Node(s[0], s[1], s[2] + current.cost, current)
                open.push(new)
        current = open.pop()
        while current.state in closed:
            current = open.pop()
    path = list()
    while current.parent:
        path.insert(0, current.action)
        current = current.parent
    return path
    util.raiseNotDefined()


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    from util import PriorityQueue
    current = Node(problem.getStartState(), '', 0, None)  # initialize state to start state in problem
    closed = []
    open = PriorityQueue()  # use queue data structure from util.py for BFS
    while not problem.isGoalState(current.state):  # loop until current state is the goal state
        if current.state not in closed:
            closed.append(current.state)  # add current state to closed list
        successors = problem.getSuccessors(current.state)  # retrieve successor state list
        for s in successors:  # add successors not in closed to open
            if s[0] not in closed:
                new = Node(s[0], s[1], s[2] + current.cost, current)
                open.push(new, new.cost)
        current = open.pop()
        while current.state in closed:
            current = open.pop()
    path = list()
    while current.parent:
        path.insert(0, current.action)
        current = current.parent
    return path

    util.raiseNotDefined()


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def stateToDir(current):
    from game import Directions
    n = Directions.NORTH
    s = Directions.SOUTH
    e = Directions.EAST
    w = Directions.WEST
    r = []
    if current.dir == "West":
        r = w
    elif current.dir == "East":
        r = e
    elif current.dir == "South":
        r = s
    elif current.dir == "North":
        r = n
    else:
        r = "NA"

    return r


class Node2:

    def __init__(self, crd, dir, cost, parent):
        self.crd = crd
        self.dir = dir
        self.cost = cost
        self.parent = parent

    def __eq__(self, other):
        return (self.crd == other.crd)

    def __lt__(self, other):
        return (self.cost < other.cost)

    def __str__(self):
        r = " " + str(self.crd)
        return r


def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    from util import PriorityQueue
    closed = list()
    open = PriorityQueue()
    current = Node2(problem.getStartState(), None, 0, None)
    count = 0
    while not problem.isGoalState(current.crd):
        if (count != 0):
            current = open.pop()
        if (problem.isGoalState(current.crd)):
            break
        count += 1
        if current not in closed:
            closed.append(current)
            for successor in problem.getSuccessors(current.crd):
                successor = Node2(successor[0], successor[1], successor[2] + current.cost, current)
                priority = int(heuristic(successor.crd, problem)) + successor.cost
                if successor not in closed:
                    open.push(successor, priority)

    path = list()
    while current.parent:
        dir = stateToDir(current)
        path.insert(0, current.dir)
        current = current.parent

    return path


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
