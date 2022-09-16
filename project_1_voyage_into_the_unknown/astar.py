import numpy as np
import math 
import random
from priorityq import PriorityQueue
import pprint
import sys
import time

pp = pprint.PrettyPrinter(indent=4)

def generategridworld(dim, p):
	#dim == dimensions of gridworld
	#p == probability of 1, 0
	gridworld = [[0 for i in range(dim)] for j in range(dim)]
	for row in range(len(gridworld)): 
		for col in range(len(gridworld)): 
			if (row == 0 and col == 0) or (row == dim-1 and col == dim-1):
				gridworld[row][col] = 0
			else:
				x = random.uniform(0,1)
				if (x <= p):
					gridworld[row][col] = 1
	return gridworld 

def astarRecursive(dim, curr, fringe, parents):
	return

def astar(dim, gridworld, heuristic):
	nodes_visited = 0
	start = (0,0)
	# goal = (dim-1, dim-1)
	# dictionary of parents {coordinates of curr: parents}
	parents = {}
	# parents = {key: node, value: (parent, expanded? [boolean])}

	# grid of distances where distancemap[x][y] is 
	# the length of the shortest path so far from the start to (x, y)
	gScoreMap=[[-1 for i in range(dim)] for j in range(dim)]
	gScoreMap[0][0]=0

	pastNode = (None,None)

	# first node
	# nodes are in the form (f(n), (x, y))
	if (heuristic == 'euclidean'):
		currNode = (euclidean(0, dim-1, 0, dim-1), start)
	elif (heuristic == 'chebyshev'):
		currNode = (chebyshev(0, dim-1, 0, dim-1), start)
	else:
		currNode = (manhattan(0, dim-1, 0, dim-1), start)
	currDistance = 0
	expanded = {}
	fringe = PriorityQueue() #priority queue
	fringe.push(currNode, 0)
	prevNode = None
	while (fringe.isEmpty() == False): 
		prevNode = currNode
		currNode = fringe.pop()
		nodes_visited += 1
		expanded.update({currNode:True})
		#if we are not at the goal node
		if not ((currNode[1][0] == dim-1) and (currNode[1][1] == dim-1)):
			currDistance = gScoreMap[currNode[1][0]][currNode[1][1]]
			childrenDict = generateChildren(currNode,currDistance,pastNode,gridworld,parents,heuristic)
			# priority = -1
			for key in childrenDict.keys(): 
				#TODO: what should we do if the key is already in the parents dictionary? 				
				if (childrenDict.get(key)[0] > -1) and not (childrenDict.get(key)[1] == (None, None)):
					# the distance of the path from the start to the current child node
					currPathDistance = gScoreMap[currNode[1][0]][currNode[1][1]] + 1
					neighborNode = childrenDict.get(key)[1]
					# if the child node is unexplored or if the path is shorter than the previous shortest path
					if (gScoreMap[neighborNode[0]][neighborNode[1]] == -1) or (currPathDistance < gScoreMap[neighborNode[0]][neighborNode[1]]):  
						# update parent dictionary
						parents.update({neighborNode: currNode[1]})
						expanded.update({neighborNode: False})
						# update shortest path length to child node
						gScoreMap[neighborNode[0]][neighborNode[1]] = currPathDistance
						# add child node to fringe
						tempNode = (childrenDict.get(key)[0],childrenDict.get(key)[1])
						fringe.push(tempNode,childrenDict.get(key)[0])
						
		else: 
			#at goal node
			pathNode = currNode[1]
			path = [pathNode]
			while(not ((pathNode[0] == 0) and (pathNode[1] == 0))): 
				parentNode = parents.get(pathNode)
				path.insert(0,parentNode)
				pathNode = parentNode
			#return path
			return (True, nodes_visited, len(path))
	#print("fringe is empty, gridworld not solvable")
	#return (-1,-1)
	return (False, nodes_visited, -1)

def astarwithstart(dim, gridworld, start, heuristic):
	# start = (0,0)
	# goal = (dim-1, dim-1)
	# dictionary of parents {coordinates of curr: parents}
	parents = {}
	# parents = {key: node, value: (parent, expanded? [boolean])}

	# grid of distances where distancemap[x][y] is 
	# the length of the shortest path so far from the start to (x, y)
	gScoreMap=[[-1 for i in range(dim)] for j in range(dim)]
	gScoreMap[0][0]=0

	pastNode = (None,None)
	nodes_visited = 0

	# first node
	# nodes are in the form (f(n), (x, y))
	if (heuristic == 'euclidean'):
		currNode = (euclidean(0, dim-1, 0, dim-1), start)
	elif (heuristic == 'chebyshev'):
		currNode = (chebyshev(0, dim-1, 0, dim-1), start)
	else:
		currNode = (manhattan(0, dim-1, 0, dim-1), start)
	currDistance = 0
	expanded = {}
	fringe = PriorityQueue() #priority queue
	fringe.push(currNode, 0)
	prevNode = None
	while (fringe.isEmpty() == False): 
		prevNode = currNode
		currNode = fringe.pop()
		nodes_visited += 1
		expanded.update({currNode:True})
		#if we are not at the goal node
		if not ((currNode[1][0] == dim-1) and (currNode[1][1] == dim-1)):
			currDistance = gScoreMap[currNode[1][0]][currNode[1][1]]
			childrenDict = generateChildren(currNode,currDistance,pastNode,gridworld,parents,heuristic)
			# priority = -1
			for key in childrenDict.keys(): 
				#TODO: what should we do if the key is already in the parents dictionary? 				
				if (childrenDict.get(key)[0] > -1) and not (childrenDict.get(key)[1] == (None, None)):
					# the distance of the path from the start to the current child node
					currPathDistance = gScoreMap[currNode[1][0]][currNode[1][1]] + 1
					neighborNode = childrenDict.get(key)[1]
					# if the child node is unexplored or if the path is shorter than the previous shortest path
					if (gScoreMap[neighborNode[0]][neighborNode[1]] == -1) or (currPathDistance < gScoreMap[neighborNode[0]][neighborNode[1]]):  
						# update parent dictionary
						parents.update({neighborNode: currNode[1]})
						expanded.update({neighborNode: False})
						# update shortest path length to child node
						gScoreMap[neighborNode[0]][neighborNode[1]] = currPathDistance
						# add child node to fringe
						tempNode = (childrenDict.get(key)[0],childrenDict.get(key)[1])
						# print(tempNode)
						fringe.push(tempNode,childrenDict.get(key)[0])
		else: 
			#at goal node
			pathNode = currNode[1]
			path = [pathNode]
			while(not ((pathNode[0] == start[0]) and (pathNode[1] == start[1]))): 
				parentNode = parents.get(pathNode)
				path.insert(0,parentNode)
				pathNode = parentNode
			return (nodes_visited, path)
	#fringe is empty, didn't reach goal node
	#print("fringe is empty, gridworld not solvable")
	return (nodes_visited, [])

"""
repeated forward a* pseudocode
while (not at goal node or not unsolvable error message): 
	if currNode is blocked: 
		set node as 1 on gridworld gridworldcopy
		path = run astar from previous currNode
	else: 
		add node to final path, check children and update if blocked
	currNode = nextNode 

if (currNode[0] == dim-1) and (currNode[1] == dim-1):
	print(finalpath)
	return finalpath
else: 
	#	if (currNode[0] == -1) and (currNode[1] == dim-1):
	print("gridworld not solvable")
	return (-1,-1)
"""

def repeatedforwardastar(dim, gridworld, heuristic):
	start = (0,0)
	goal = (dim-1,dim-1)
	cells_processed = 0
	gridworldcopy = generategridworld(dim, 0)
	astarresult = astarwithstart(dim, gridworldcopy, start, heuristic)
	cells_processed += astarresult[0] 
	path = astarresult[1]
	finalpath = []
	prevNode = (-1,-1)
	currNode = (0,0)

	i = 0
	while ((currNode[0] != dim-1) and (currNode[1] != dim-1)) or ((currNode[0] != -1) and (currNode[1] != -1)): 
		if (currNode[0] == dim-1) and (currNode[1] == dim-1): 
			break
		row = currNode[0]
		col = currNode[1]
		if (gridworld[row][col] == 1):
			#cell is blocked
			#print("blocked cell (" + str(currNode[0]) + "," + str(currNode[1]) + ")")
			gridworldcopy[row][col] = 1
			path = []
			astarresult = astarwithstart(dim, gridworldcopy, prevNode, heuristic)
			cells_processed += astarresult[0]	
			path = astarresult[1] # TODO: Need astar from specific starting point 
			if (len(path) == 0):
				break
			i = 0
			currNode = prevNode
		else: 
			if not (currNode in finalpath): 
				finalpath.append(currNode)
			#TODO: generate children to see if blocked
			north = (currNode[0]-1, currNode[1]) 
			south = (currNode[0]+1, currNode[1]) 
			east = (currNode[0], currNode[1]+1) 
			west = (currNode[0], currNode[1]-1) 

			if not isChildValidRepeatedForward(north, dim):
				if not ((north[0] == dim) or (north[1] == dim) or (north[0] == -1) or (north[1] == -1)): 
					gridworldcopy[north[0]][north[1]] = 1
			if not isChildValidRepeatedForward(south, dim): 
				if not ((south[0] == dim) or (south[1] == dim) or (south[0] == -1) or (south[1] == -1)): 
					gridworldcopy[south[0]][south[1]] = 1
			if not isChildValidRepeatedForward(east, dim):
				if not ((east[0] == dim) or (east[1] == dim) or (east[0] == -1) or (east[1] == -1)): 
					gridworldcopy[east[0]][east[1]] = 1
			if not isChildValidRepeatedForward(west, dim): 
				if not ((west[0] == dim) or (west[1] == dim) or (west[0] == -1) or (west[1] == -1)): 
					gridworldcopy[west[0]][west[1]] = 1			

			prevNode = currNode
			currNode = path[i+1]
			i = i+1

	#if (len(path) == 0):
		#print("gridworld is not solvable")
		#return [] 
	#else: 
		#finalpath.append(currNode)
		#print(finalpath)
		#return finalpath
	#printAsGrid(gridworld)
	#print('')
	#printAsGrid(gridworldcopy)
	if (len(path) != 0):
		
		#print(finalpath)
		finalpath.append(currNode)
	else:
		#print("gridworld is not solvable")
		return(-1, cells_processed, gridworldcopy, finalpath)

	return(len(finalpath), cells_processed, gridworldcopy, finalpath)
		
		#	add node to final path, check children and update if blocked
		#currNode = nextNode 



def repeatedforwardastar_restricted(dim, gridworld, heuristic):
	start = (0,0)
	goal = (dim-1,dim-1)
	cells_processed = 0
	gridworldcopy = generategridworld(dim, 0)
	astarresult = astarwithstart(dim, gridworldcopy, start, heuristic)
	cells_processed += astarresult[0] 
	path = astarresult[1]
	finalpath = []
	prevNode = (-1,-1)
	currNode = (0,0)

	i = 0
	while ((currNode[0] != dim-1) and (currNode[1] != dim-1)) or ((currNode[0] != -1) and (currNode[1] != -1)): 
		if (currNode[0] == dim-1) and (currNode[1] == dim-1): 
			break
		row = currNode[0]
		col = currNode[1]
		if (gridworld[row][col] == 1):
			#cell is blocked
			#print("blocked cell (" + str(currNode[0]) + "," + str(currNode[1]) + ")")
			gridworldcopy[row][col] = 1
			path = []
			astarresult = astarwithstart(dim, gridworldcopy, prevNode, heuristic)
			cells_processed += astarresult[0]	
			path = astarresult[1] # TODO: Need astar from specific starting point 
			if (len(path) == 0):
				break
			i = 0
			currNode = prevNode
		else: 
			if not (currNode in finalpath): 
				finalpath.append(currNode)
			prevNode = currNode
			currNode = path[i+1]
			i = i+1

	#if (len(path) == 0):
		#print("gridworld is not solvable")
		#return [] 
	#else: 
		#finalpath.append(currNode)
		#print(finalpath)
		#return finalpath
	#printAsGrid(gridworld)
	#print('')
	#printAsGrid(gridworldcopy)
	if (len(path) != 0):
		
		#print(finalpath)
		finalpath.append(currNode)
	else:
		#print("gridworld is not solvable")
		return(-1, cells_processed, gridworldcopy, finalpath)

	return(len(finalpath), cells_processed, gridworldcopy, finalpath)
		
		#	add node to final path, check children and update if blocked
		#currNode = nextNode 






def generateChildren(currNode, currDistance, pastNode, gridworld, parents, heuristic): 
	# currNode (x, y)
	currNodeRow = currNode[1][0]
	currNodeCol = currNode[1][1]
	#initial values of children
	north = (currNodeRow-1, currNodeCol)
	south = (currNodeRow+1, currNodeCol)
	east = (currNodeRow, currNodeCol+1)
	west = (currNodeRow, currNodeCol-1)

	dim = len(gridworld)
	#check if each direction is blocked
	if not isChildValid(north, dim, gridworld): 
		north = (None, None)
	if not isChildValid(south, dim, gridworld):
		south = (None, None)
	if not isChildValid(east, dim, gridworld): 
		east = (None, None)
	if not isChildValid(west, dim, gridworld): 
		west = (None, None)

	northPriority = generatePriority(north, dim, currDistance, heuristic) 
	southPriority = generatePriority(south, dim, currDistance, heuristic)
	eastPriority = generatePriority(east, dim, currDistance, heuristic)
	westPriority = generatePriority(west, dim, currDistance, heuristic)

	childrenDict = {
		"north":(northPriority,north),
		"south":(southPriority,south),
		"east":(eastPriority,east),
		"west":(westPriority,west)	
	}

	return childrenDict


def generatePriority(childNode, dim, distance, heuristic): 
	if (childNode[0] == None) or (childNode[1] == None):
		return -1
	else: 
		#right now using manhattan distance by default
		if (heuristic == 'chebyshev'):
		    return chebyshev(childNode[0],childNode[1],dim-1,dim-1) + distance
		elif (heuristic == 'euclidean'):
			return euclidean(childNode[0],childNode[1],dim-1,dim-1) + distance
		else:
			return manhattan(childNode[0],childNode[1],dim-1,dim-1) + distance
	


def isChildValid(childNode, dim, gridworld): 
	row = childNode[0]
	col = childNode[1]
	if (row < 0) or (row == dim) or (col < 0) or (col == dim):
		return False
	if (gridworld[row][col] == 1):
		return False
	return True

def isChildValidRepeatedForward(childNode, dim): 
	row = childNode[0]
	col = childNode[1]
	if (row < 0) or (row == dim) or (col < 0) or (col == dim):
		return False
	return True	

def printAsGrid(gridworld):
	#prints gridworld as grid
	for i in range(len(gridworld)): 
		print(gridworld[i])

def euclidean(x1, y1, x2, y2):
	distance = math.sqrt(((x1-x2)**2) + ((y1-y2)**2)) 
	return distance

def manhattan(x1, y1, x2, y2):
	distance = (abs(x1-x2) + abs(y1-y2))
	return distance

def chebyshev(x1, y1, x2, y2):
	distance = max(abs(x1-x2),abs(y1-y2))
	return distance

def gw_density(dim, gridworld):
	blocks = 0
	for row in range(dim):
		for col in range(dim):
			if (gridworld[row][col] == 1):
				blocks+=1
	return blocks/(dim*dim)


#gridworld = generategridworld(5, 0.3)
#printAsGrid(gridworld)
#astarwithstart(5,gridworld, (0,0))

"""
gridworld = [[0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 1, 1, 1, 1, 0, 0, 0],
[0, 1, 0, 0, 0, 1, 0, 1, 0, 0],
[1, 0, 1, 0, 0, 1, 0, 0, 0, 0],
[1, 0, 0, 0, 0, 0, 1, 0, 0, 0],
[1, 0, 0, 1, 0, 0, 0, 1, 0, 0],
[1, 1, 0, 0, 0, 1, 0, 0, 1, 0],
[0, 1, 0, 1, 0, 0, 1, 0, 1, 0],
[1, 1, 0, 0, 0, 0, 1, 1, 0, 0],
[0, 0, 0, 0, 1, 0, 1, 0, 1, 0]]
"""
#gridworld = generategridworld(101, 0.3)
#printAsGrid(gridworld)
#astar(10,gridworld)
#path = astarwithstart(10, gridworld, (0,0), "manhattan")
#print(path)
#path = repeatedforwardastar_restricted(101,gridworld, "manhattan")
#print(path[0])
#print(path[1])
#print(path[3])
