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
    "*** YOUR CODE HERE ***"
    
    res = {}
    res['res'] = []
    def dfs(path,location,visited):
        if visited.get(location,-1) != -1:
            return
        visited[location] = 1
        if problem.isGoalState(location):
            res['res'] = path
           
            return 
        tempStack = util.Stack()
        for successor in problem.getSuccessors(location):
            if visited.get(successor[0],-1) != -1:
                continue
            tempStack.push(successor)
 
        while not tempStack.isEmpty():
            if visited.get(tempStack.list[-1],-1) != -1:
                continue
            action = tempStack.pop()
            nPath = path[:]
            nPath.append(action[1])
            
            problem.startState = action[0]
            if res['res'] != []:
                return
            dfs(nPath,action[0],visited)
        return
        
    dfs([],problem.getStartState(),{})
    return res['res']

    
def breadthFirstSearch(problem: SearchProblem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"

    fringe = util.Queue()
    state = problem.getStartState()
    visited = {} 
    fringe.push((state, "", [])) # each element is (state, action, path so far)

    while not fringe.isEmpty():
        action = fringe.pop()
        state, stepAction, pathSoFar = action[0], action[1], action[2]
        if visited.get(state, -1) != -1: # skip visited states
            continue
        visited[state] = 1 # mark as visited
        if stepAction != "": # appending action
            pathSoFar.append(stepAction) 
        if problem.isGoalState(state): 
            return pathSoFar
        for suc in problem.getSuccessors(state):
            if visited.get(suc[0], -1) != -1: # skip visited states
                continue
            copy = pathSoFar[:]
            fringe.push((suc[0], suc[1], copy))
    return []

def uniformCostSearch(problem: SearchProblem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    visited = {}
    nexQ = util.PriorityQueue()
    nexQ.push((problem.getStartState(),"",[],0),0)

    while not nexQ.isEmpty():
        action = nexQ.pop()
        if visited.get(action[0],-1) != -1:
            continue
        visited[action[0]] = 1
        nPath = action[2]
        if action[1] != "":
            nPath.append(action[1])
        if problem.isGoalState(action[0]):
            return nPath
        for suc in problem.getSuccessors(action[0]):
            if visited.get(suc[0],-1) != -1:
                continue
            cop = nPath[:]
            nexQ.push((suc[0],suc[1],cop,suc[2]+action[3]) , suc[2]+action[3])
    return []
    
def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    visited = {}
    nexQ = util.PriorityQueue()
    nexQ.push((problem.getStartState(),"",[],0),0)

    while not nexQ.isEmpty():
        action = nexQ.pop()
        if visited.get(action[0],-1) != -1:
            continue
        visited[action[0]] = 1
        nPath = action[2]
        if action[1] != "":
            nPath.append(action[1])
        if problem.isGoalState(action[0]):
            return nPath
        for suc in problem.getSuccessors(action[0]):
            if visited.get(suc[0],-1) != -1:
                continue
            cop = nPath[:]
            nexQ.push((suc[0],suc[1],cop,suc[2]+action[3]) , suc[2]+action[3]+heuristic(suc[0],problem))
    return []

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
