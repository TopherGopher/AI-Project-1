# Coauthored by: Chris Sterling - Caveat4U
# Coauthored by: Spencer Comerford - Spencevail


# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for 
# educational purposes provided that (1) you do not distribute or publish 
# solutions, (2) you retain this notice, and (3) you provide clear 
# attribution to UC Berkeley, including a link to 
# http://inst.eecs.berkeley.edu/~cs188/pacman/pacman.html
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero 
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and 
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called
by Pacman agents (in searchAgents.py).
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
        Returns the start state for the search problem
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples,
        (successor, action, stepCost), where 'successor' is a
        successor to the current state, 'action' is the action
        required to get there, and 'stepCost' is the incremental
        cost of expanding to that successor
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.  The sequence must
        be composed of legal moves
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other
    maze, the sequence of moves will be incorrect, so only use this for tinyMaze
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s,s,w,s,w,w,s,w]

def depthFirstSearch(problem):
    from game import Directions
    """
    Search the deepest nodes in the search tree first

    Your search algorithm needs to return a list of actions that reaches
    the goal.  Make sure to implement a graph search algorithm

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:
    """
    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    
    "*** YOUR CODE HERE ***"
    #print "IS it empty?" + str(s.pisEpmptyp))
    path = []
    visited = []
    depth = 0
    s = util.Stack()
    s.push(Node(problem.getStartState(), depth))

    while not s.isEmpty():
        tempNode = s.pop()

        if problem.isGoalState(tempNode.getCoordinates(problem)):
            print "GOAL FOUND! at " + str(tempNode.getCoordinates(problem))
            dirs = []
            for i in path:
                dirs.append(i.getDirection())
            print "ACTIONS: " + str(dirs)
            print "PATH" + str(path)
            return dirs
       # if tempState not visited:
        if tempNode.getCoordinates(problem) not in visited:
            print "TEMPNODE: "+str(tempNode.s)
            if tempNode.s != problem.getStartState():
                visited.append(tempNode.getCoordinates(problem))
                path.append(tempNode)
            print "VISITED: "+str(visited)

            
            successors = problem.getSuccessors(tempNode.getCoordinates(problem))
            print "SUCCESSORS: "+str(successors)

            for nextState in successors:
                print "PUSHING: "+str(nextState)
                s.push(Node(nextState, depth))
            depth+=1
        else:
            print "NOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO"
            for i in path:
                if(i.d > depth):
                    path.remove(i)

    util.raiseNotDefined()

class Node:
    d = 0
    def __init__(self, state, depth):
        self.s = state
        self.d = depth

    def __str__(self):
        return str(self.s) + ' ' + str(self.d)
    def __unicode__(self):
        return u'a'
    def __repr__(self):
        return str(self.s)+' '+str(self.d)

    def getDirection(self):
        from game import Directions
        if self.s[1] == 'North':
            return Directions.NORTH
        elif self.s[1] == 'East':
            return Directions.EAST    
        elif self.s[1] == 'South':
            return Directions.SOUTH
        elif self.s[1] == 'West':
            return Directions.WEST
        else:
            return 0    

    def getCoordinates(self, problem):
        if self.s != problem.getStartState():
            return self.s[0]
        else:
            return self.s

#def inPath(tempState, path):
#    for state in path:
#        if state[0] == tempState[0]:
#            return True
#    return False


def recursiveDFS(problem, path):

    print "Problem: " + str(problem) + "  Path: " + str(path)

    startState = problem.getStartState()

    successors = problem.getSuccessors()
    
    if len(successors) == 0:
        if problem.isGoalState(startState):
            return path
        else:
            return None
    else:
        for successor in successors:
            newPath = list(path)
            newPath.append(successor[1])
            result = recursiveDFS(successor, newPath)
            if result:
                return result

def breadthFirstSearch(problem):
    """
    Search the shallowest nodes in the search tree first.
    """
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

def uniformCostSearch(problem):
    "Search the node of least total cost first. "
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    "Search the node that has the lowest combined cost and heuristic first."
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
