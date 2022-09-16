import numpy as np
import math 
import random
from priorityq import PriorityQueue
import pprint
import sys
import time
import csv

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

def astarwithstart(dim, gridworld, start, heuristic):
	# start = (0,0)
	# goal = (dim-1, dim-1)
	# dictionary of parents {coordinates of curr: parents}
	parents = {}
	# parents = {key: node, value: (parent, expanded? [boolean])}

	# grid of distances where distancemap[x][y] is 
	# the length of the shortest path so far from the start to (x, y)
	gScoreMap=[[-1 for i in range(dim)] for j in range(dim)]
	gScoreMap[start[0]][start[1]]=0

	pastNode = (None,None)
	nodes_visited = 0

	# first node
	# nodes are in the form (f(n), (x, y))
	if (heuristic == 'euclidean'):
		currNode = (euclidean(start[0], dim-1, start[1], dim-1), start)
	elif (heuristic == 'chebyshev'):
		currNode = (chebyshev(start[0], dim-1, start[1], dim-1), start)
	else:
		currNode = (manhattan(start[0], dim-1, start[1], dim-1), start)
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

def repeatedforwardastar(dim, gridworld, heuristic, writer):
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
			gridworldcopy[row][col] = 1
			path = []
			astarresult = astarwithstart(dim, gridworldcopy, prevNode, heuristic)
			cells_processed += astarresult[0]	
			path = astarresult[1] # TODO: Need astar from specific starting point 
			if (len(path) == 0):
				break
			i = 0
			next_dir = which_direction(currNode, prevNode)
			local_grid = generate_local_grid(prevNode, gridworld, dim)
			# writer.writerow([gridworldcopy, curr_x, curr_y, prev_x, prev_y, next_direction, num_blocked, is_blocked, is_goal, goal_x, goal_y])
			writer.writerow([local_grid, 0, 0])
			#writer.writerow([gridworldcopy, currNode[0], currNode[1], prevNode[0], prevNode[1], next_dir, 0, 1, 0, dim, dim])
			currNode = prevNode

		else: 
			if not (currNode in finalpath): 
				finalpath.append(currNode)
			#TODO: generate children to see if blocked
			north = (currNode[0]-1, currNode[1]) 
			south = (currNode[0]+1, currNode[1]) 
			east = (currNode[0], currNode[1]+1) 
			west = (currNode[0], currNode[1]-1) 
			num_blocks = 0

			if isChildValidRepeatedForward(north, dim):
				if (gridworld[north[0]][north[1]]==1):
					gridworldcopy[north[0]][north[1]]=1
					num_blocks += 1
			if isChildValidRepeatedForward(south, dim):
				if (gridworld[south[0]][south[1]]==1):
					gridworldcopy[south[0]][south[1]]=1
					num_blocks += 1

			if isChildValidRepeatedForward(east, dim):
				if (gridworld[east[0]][east[1]]==1):
					gridworldcopy[east[0]][east[1]]=1
					num_blocks += 1
			if isChildValidRepeatedForward(west, dim):
				if (gridworld[west[0]][west[1]]==1):
					gridworldcopy[west[0]][west[1]]=1
					num_blocks += 1			

			next_dir = which_direction(currNode, path[i+1])
			local_grid = generate_local_grid(currNode, gridworld, dim)
			# writer.writerow([gridworldcopy, curr_x, curr_y, prev_x, prev_y, next_direction, num_blocked, is_blocked, is_goal, goal_x, goal_y])
			writer.writerow([local_grid, next_dir, 0])
			#writer.writerow([gridworldcopy, currNode[0], currNode[1], prevNode[0], prevNode[1], next_dir, num_blocks, 0, 0, dim, dim])
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
		local_grid = generate_local_grid(currNode, gridworld, dim)
		# writer.writerow([gridworldcopy, curr_x, curr_y, prev_x, prev_y, next_direction, num_blocked, is_blocked, is_goal, goal_x, goal_y])
		writer.writerow([local_grid, 0, 1])
		#writer.writerow([gridworldcopy, currNode[0], currNode[1], prevNode[0], prevNode[1], 0, num_blocks, 0, 1, dim, dim])

	else:
		#print("gridworld is not solvable")
		return(-1, cells_processed, gridworldcopy, finalpath)

	return(len(finalpath), cells_processed, gridworldcopy, finalpath)
		
		#	add node to final path, check children and update if blocked
		#currNode = nextNode 


def generate_neighbors(cell, gridworld, dim):
    northwest = (cell[0]-1, cell[1]-1)
    north = (cell[0]-1, cell[1])
    northeast = (cell[0]-1, cell[1]+1)
    west = (cell[0], cell[1]-1)
    east = (cell[0], cell[1]+1)
    southwest = (cell[0]+1, cell[1]-1)
    south = (cell[0]+1, cell[1])
    southeast = (cell[0]+1, cell[1]+1)

    num_neighbors = 0
    blocked_neighbors = 0

    precheck_neighbor_list = [northwest, north, northeast, west, east, southwest, south, southeast]
    neighbor_list = []
    for neighbor in precheck_neighbor_list:
        if neighbor[0] < 0 or neighbor[0] >= dim or neighbor[1] < 0 or neighbor[1] >= dim:
            neighbor_list.append((None, None))
        else: 
            neighbor_list.append(neighbor)
            num_neighbors+=1
            if gridworld[neighbor[0]][neighbor[1]] == 1:
                blocked_neighbors+=1

    return (num_neighbors, blocked_neighbors, neighbor_list)

def which_direction(currNode, otherNode): 
	currNodeRow = currNode[0]
	currNodeCol = currNode[1]
	otherNodeRow = otherNode[0]
	otherNodeCol = otherNode[1]

	x_val = otherNodeRow - currNodeRow
	y_val = otherNodeCol - currNodeCol

	if y_val == 0: 
		# other is east or west
		if x_val == 1: 
			# resturn east
			return 2
		else: 
			# return west
			return 4
	else: 
		# other is north or south, x_val == 0
		if y_val == 1:
			# return south 
			return 3
		else: 
			# return north
			return 1

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
	if (row == None) or (col == None) or (row < 0) or (row == dim) or (col < 0) or (col == dim):
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

def generate_local_grid(currentNode, gridworldCopy, dim): 
	local_gridworld = [[0 for i in range(3)] for j in range(3)]
	currNodeRow = currentNode[0]
	currNodeCol = currentNode[1]

	northWest = (currNodeRow-1, currNodeCol-1)
	north = (currNodeRow-1, currNodeCol)
	northEast = (currNodeRow-1, currNodeCol+1)
	southWest = (currNodeRow+1, currNodeCol-1)
	south = (currNodeRow+1, currNodeCol)
	southEast = (currNodeRow+1, currNodeCol+1)
	east = (currNodeRow, currNodeCol+1)
	west = (currNodeRow, currNodeCol-1)

	# top row
	if not isChildValid(northWest, dim, gridworldCopy):
		local_gridworld[0][0] = 1
	if not isChildValid(north, dim, gridworldCopy):
		local_gridworld[0][1] = 1
	if not isChildValid(northEast, dim, gridworldCopy):
		local_gridworld[0][2] = 1

	# middle row
	if not isChildValid(west, dim, gridworldCopy):
		local_gridworld[1][0] = 1
	if gridworldCopy[currNodeRow][currNodeCol] == 1: 
		local_gridworld[1][1] = 1
	if not isChildValid(east, dim, gridworldCopy):
		local_gridworld[1][2] = 1

	# bottom row
	if not isChildValid(southWest, dim, gridworldCopy):
		local_gridworld[2][0] = 1
	if not isChildValid(south, dim, gridworldCopy):
		local_gridworld[2][1] = 1
	if not isChildValid(southEast, dim, gridworldCopy):
		local_gridworld[2][2] = 1

	return local_gridworld


def generate_solvable_gridworld(dim, p):
	gridworld = generategridworld(dim, p)
	path = astarwithstart(dim, gridworld, (0,0), "manhattan")
	solvable = (path[1] != [])
	while (solvable == False):
		gridworld = generategridworld(dim, p)
		path = astarwithstart(dim, gridworld, (0,0), "manhattan")
		solvable = (path[1] != [])
	return gridworld