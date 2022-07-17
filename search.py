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

class Node:  #Class node that helps us create instances for our search strategies
    def __init__(self,state,parent,action,pathCost,ManhattanDistance):  #constructor that takes 5 arguments
        self.state = state
        self.parent = parent
        self.action = action
        self.pathCost = pathCost  
        self.ManhattanDistance = ManhattanDistance  # Manhattan distance is better than the straight line distance in our case
        


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
    # Fringe in this case is LIFO so we used a Stack 
    DFSfringe = util.Stack()

    #Visited nodes is used to check if the state is visited or not yet to avoid loops  
    visitedNodes = [] 


    # The stack and the list here are used in the process of backtracking from the Goal to the initial state in order to get the path
    actions = util.Stack()
    path = []  #List of directions or actions that pacman will follow to reach the Goal

    
    #Creating the first instance of class Node and giving it the initial state that pacman is in
    node = Node((problem.getStartState()), None, None,0,0) 

    #Pushing the instance into the fringe 
    DFSfringe.push(node)

    #Checking if the fring is not empty and looping
    while not DFSfringe.isEmpty(): 

        #Popping the object from the fringe and storing it in current
        current = DFSfringe.pop()
        
        #Checking if the state of current is the goal state before proceeding further
        if problem.isGoalState(current.state):

            # print("Goal is" + str(current.state))

            #Push the action that led to the Goal into actions 
            actions.push(current.action)
            
            # In this loop the condition is that the parent shouldn't be None and that is the case of the initial node
            while current.parent != None:
                # Popping the action added from the stack and appending it to the list
                path.append(actions.pop())
                # Pushing the action of the previous node by using the parent to backtrack 
                actions.push(current.parent.action)
                # Changing current to its parent and looping
                current = current.parent
            
            # The action in the path are inverted so by using reverse we get the correct path and order to give it to pacman
            path.reverse()

            # print(path)

            #Returning the path to pacman (series of actions)
            return path

        #If the current state is not the goal we generate the successors of that node and we store them in successor
        successor = problem.getSuccessors(current.state)
        
        # Looping in the successor 
        for i in range(0, len(successor)):
            #Creating a new instance and passing the arguments 
            newNode = Node(successor[i][0], current, successor[i][1],0,0)

            # Pushing the new node in the fringe 
            DFSfringe.push(newNode)

            # Looping in the visited nodes list and checking if the states of the new objects are already expanded
            for j in range(0, len(visitedNodes)):
                if newNode.state == visitedNodes[j]:
                    # If they are, we pop from the stack to remove them and avoid the unwanted expension
                    DFSfringe.pop()
                    break

        #The states of the new nodes expanded are appended to the visited list 
        visitedNodes.append(current.state)
        # print("Visited States: " + str(visitedNodes))
    


    # util.raiseNotDefined()


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"

    # Fringe in this case is FIFO so we used a Queue
    BFSfringe = util.Queue()

    #Visited nodes is used to check if the state is visited or not yet to avoid loops 
    visitedNodes = []

    # The stack and the list here are used in the process of backtracking from the Goal to the initial state in order to get the path
    actions = util.Stack()
    path = []  #List of directions or actions that pacman will follow to reach the Goal
    
    #Creating the first instance of class Node and giving it the initial state that pacman is in
    node = Node((problem.getStartState()), None, None,0,0)

    #Pushing the instance into the fringe
    BFSfringe.push(node)

    #Checking if the fring is not empty and looping
    while not BFSfringe.isEmpty():

        #Popping the object from the fringe and storing it in current
        current = BFSfringe.pop()
        
        # Looping to check if the popped object's state is visited 
        i = 0
        while (i < len(visitedNodes)):
            if visitedNodes[i] == current.state:
                # If it is we pop the fringe and change current to the next node popped 
                current = BFSfringe.pop()
                i = 0
                
            else:
                i += 1
                
        
        
        #The state of node popped is appended to the visited list
        visitedNodes.append(current.state)

        # print("Visited States: " + str(visitedNodes))

        #The process of getting the goal and the path is the same as in DFS (Backtracking)
        if problem.isGoalState(current.state):

            # print("Goal is: " + str(current.state)) 
            actions.push(current.action)

            while current.parent != None:

                path.append(actions.pop())
                actions.push(current.parent.action)
                current = current.parent

            path.reverse()

            # print(path)
            return path

        # print(current.state)

        #If the current state is not the goal we generate the successors of that node and we store them in successor
        successor = problem.getSuccessors(current.state)
        # print(successor)

        # Looping in the successor 
        for i in range(0, len(successor)):

            #Creating a new instance and passing the arguments
            newNode = Node(successor[i][0], current, successor[i][1],0,0)

            # Pushing the new node in the fringe 
            BFSfringe.push(newNode)

            # print(newNode.state)
            
    # util.raiseNotDefined()


    

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    # Fringe in this case is a Priority Queue ordered by the path cost
    UCSfringe = util.PriorityQueue()

    #Visited nodes is used to check if the state is visited or not yet to avoid loops 
    visitedNodes = []

    # The stack and the list here are used in the process of backtracking from the Goal to the initial state in order to get the path
    actions = util.Stack()
    path = []  #List of directions or actions that pacman will follow to reach the Goal
    
    #Creating the first instance of class Node and giving it the initial state that pacman is in
    #Here the path cost is still 0 at the root
    node = Node((problem.getStartState()), None, None,0,0)

    #Pushing the instance into the fringe that takes a second argument which is the priority (our path cost)
    UCSfringe.push(node,node.pathCost)

    #Checking if the fringe is not empty and looping
    while not UCSfringe.isEmpty():

        #Popping the object from the fringe and storing it in current
        current = UCSfringe.pop()
        
        # Looping to check if the popped object's state is visited 
        i = 0
        while (i < len(visitedNodes)):
            if visitedNodes[i] == current.state:
                # If it is we pop the fringe and change current to the next node popped 
                current = UCSfringe.pop()
                i = 0
                
            else:
                i += 1
                
        
        
        #The state of node popped is appended to the visited list
        visitedNodes.append(current.state)

        # print("Visited States: " + str(visitedNodes))

        #The process of getting the goal and the path is the same as in DFS (Backtracking)
        if problem.isGoalState(current.state):

            # print("Goal is: " + str(current.state)) 
            actions.push(current.action)

            while current.parent != None:

                path.append(actions.pop())
                actions.push(current.parent.action)
                current = current.parent

            path.reverse()

            # print(path)
            return path

        # print(current.state)

        #If the current state is not the goal we generate the successors of that node and we store them in successor
        successor = problem.getSuccessors(current.state)
        #print(successor)

        # Looping in the successor 
        for i in range(0, len(successor)):

            #Creating a new instance and passing the arguments
            #We are taking the new successors path cost and adding it to the previous one (all passed as the path cost of the new node)
            newNode = Node(successor[i][0], current, successor[i][1],successor[i][2]+ current.pathCost,0) 
            #print(newNode.pathCost)
            #print(newNode.state)

            # Pushing the new node in the fringe 
            #Priority is the new pathCost calculated and stored in the new node above
            UCSfringe.push(newNode,newNode.pathCost)

            # print(newNode.state)
            
    # util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):

    #The only difference between ucs and astar is that the path cost has to be added to the straight line distance 


    # Fringe in this case is also a priority queue 
    ASfringe = util.PriorityQueue()

    #Visited nodes is used to check if the state is visited or not yet to avoid loops 
    visitedNodes = []

    # The stack and the list here are used in the process of backtracking from the Goal to the initial state in order to get the path
    actions = util.Stack()
    path = []  #List of directions or actions that pacman will follow to reach the Goal
    
    StartingState = problem.getStartState()
    #Creating the first instance of class Node and giving it the initial state that pacman is in
    #The ManhattanDistance of the node is getting the straight line distance using the heuristic that accepts a state and the problem 
    node = Node(StartingState, None, None,0,heuristic(StartingState,problem))
    

    #Pushing the instance into the fringe
    #Here the difference is that the priority consists of the cost and the straight line distance 
    ASfringe.push(node,node.pathCost + node.ManhattanDistance)

    #Checking if the fring is not empty and looping
    while not ASfringe.isEmpty():

        #Popping the object from the fringe and storing it in current
        current = ASfringe.pop()
        
        # Looping to check if the popped object's state is visited 
        i = 0
        while (i < len(visitedNodes)):
            if visitedNodes[i] == current.state:
                # If it is we pop the fringe and change current to the next node popped 
                current = ASfringe.pop()
                i = 0
                
            else:
                i += 1
                
        
        
        #The state of node popped is appended to the visited list
        visitedNodes.append(current.state)

        # print("Visited States: " + str(visitedNodes))

        #The process of getting the goal and the path is the same as in DFS (Backtracking)
        if problem.isGoalState(current.state):

            # print("Goal is: " + str(current.state)) 
            actions.push(current.action)

            while current.parent != None:

                path.append(actions.pop())
                actions.push(current.parent.action)
                current = current.parent

            path.reverse()

            # print(path)
            return path

        # print(current.state)

        #If the current state is not the goal we generate the successors of that node and we store them in successor
        successor = problem.getSuccessors(current.state)
        #print(successor)

        # Looping in the successor 
        for i in range(0, len(successor)):

            #Creating a new instance and passing the arguments
            #Getting the heuristic of the state of the new node
            newNode = Node(successor[i][0], current, successor[i][1],successor[i][2]+ current.pathCost,heuristic(successor[i][0],problem))
            # print("Manhattan Distance :  " + str(heuristic(successor[i][0],problem)))
            #print(newNode.pathCost)
            #print(newNode.state)

            #Defining the priority as the path cost to the node + the ManhattanDistance of that node (its f(n)=g(n)+h(n))
            priority = newNode.pathCost + newNode.ManhattanDistance
            # Pushing the new node in the fringe 
            ASfringe.push(newNode,priority)

            # print(newNode.state)
            
    # util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
