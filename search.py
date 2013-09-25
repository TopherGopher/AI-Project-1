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
	# print "Start:", problem.getStartState()
	# print "Is the start a goal?", problem.isGoalState(problem.getStartState())
	# print "Start's successors:", problem.getSuccessors(problem.getStartState())
	
	"*** YOUR CODE HERE ***"

	# import pdb
	# pdb.set_trace()


	path = []

	result = recursiveDFS(problem, None, None)

	result.reverse()

	return result

	util.raiseNotDefined()

def recursiveDFS(problem, state, visitedStates):

	# print "Problem: " + str(problem) + "  Path: " + str(path)

	if visitedStates == None:
		visitedStates = []

	if state == None:
		currentState = problem.getStartState()
	else:
		currentState = state[0]
	

	if problem.isGoalState(currentState):
		return [state[1]]

	successors = problem.getSuccessors(currentState)

	if successors == None or len(successors) == 1 or (currentState in visitedStates):
		return None
	else:

		result = None

		visitedStates.append(currentState)

		for successor in successors:

			result = recursiveDFS(problem, successor, visitedStates)
		
			if result:

				if state:
					result.append(state[1])

				# print "Result: " + str(result)

				break

		visitedStates.pop()

	return result


		# if problem.isGoalState(startState):
		# 	return path
		# else:
		# 	return None
		# else:
		# 	for successor in successors:
		# 		newPath = list(path)
		# 		newPath.append(successor[1])
		# 		result = recursiveDFS(successor, newPath)
		# 		if result:
		# 			return result

def breadthFirstSearch(problem):
	"""
	Search the shallowest nodes in the search tree first.
	"""
	"*** YOUR CODE HERE ***"
	util.raiseNotDefined()

def uniformCostSearch(problem):
    "Search the node of least total cost first. "
    pq = util.PriorityQueue()
    #Insert the root into the queue
    root = Node(problem.getStartState())
    pq.push(root, priority)
    #While the queue is not empty
    while(!pq.isEmpty()):
    #  Dequeue the maximum priority element from the queue
        tempNode = pq.pop()
    #  (If priorities are same, alphabetically smaller path is chosen)

    #  If the path is ending in the goal state, print the path and exit
        if problem.isGoalState(tempNode.s):
            return path
    #  Else
        else:
    #       Insert all the children of the dequeued element, with the cumulative costs as priority
            successors = problem.getSuccessors()
            for successor in successors:
                n = Node(successor)
                pq.push(n, priority)

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


class Tree:
	
	def __init__(self, rootValue):
		self.root = None
		self.root = Node(self, rootValue)

	def __str__(self):
			return self.root.visualize()

class Node:
	
	def __init__(self, parent, value):
		if isinstance(parent, Tree):
			self.initializer(parent, None, value, 0)
		else:
			self.initializer(parent.tree, parent, value, parent.depth + 1)


	def initializer(self, tree, parent, value, depth):
		self.tree = tree
		self.value = value
		self.depth = depth
		self.children = []

		if parent:
			parent.addChildNode(self)


	def addChildNode(self, node):
		assert node.depth == self.depth + 1, "Child node depth is" + node.depth + " and can't be added to node with depth " + self.depth

		node.parent = self
		self.children.append(node)
	

	def visualize(self):
		rep = str(self)

		indent = '--->'

		rep += indent

		for child in self.children:
			rep += indent

			rep += child.visualize()

			# rep += '\n'

		return rep + '\n'


	def __str__(self):
			return 'Node(Depth:' + str(self.depth) + '   Value:' + str(self.value) + ')\n'

	# def __unicode__(self):
	# 	return u'a'

	# def __repr__(self):
	# 	return str(self.s)+' '+str(self.d)
#def inPath(tempState, path):
#    for state in path:
#        if state[0] == tempState[0]:
#            return True
#    return False


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
