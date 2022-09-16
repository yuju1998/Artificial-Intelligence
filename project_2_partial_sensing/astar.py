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

			if isChildValidRepeatedForward(north, dim):
				if (gridworld[north[0]][north[1]]==1):
					gridworldcopy[north[0]][north[1]]=1
			if isChildValidRepeatedForward(south, dim):
				if (gridworld[south[0]][south[1]]==1):
					gridworldcopy[south[0]][south[1]]=1
			if isChildValidRepeatedForward(east, dim):
				if (gridworld[east[0]][east[1]]==1):
					gridworldcopy[east[0]][east[1]]=1
			if isChildValidRepeatedForward(west, dim):
				if (gridworld[west[0]][west[1]]==1):
					gridworldcopy[west[0]][west[1]]=1
						

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

def exampleinferenceagent(dim, gridworld, heuristic):
	start = (0,0)
	goal = (dim-1,dim-1)
	cells_processed = 0
	gridworldcopy = generategridworld(dim, 0)
	# gridworld_neighbor_blocks keeps track of number of neighbors that are blocked at each cell
	gridworld_neighbor_blocks = generategridworld(dim, 0)
	neighbor_info_dictionary = {}
	""" 
	dictionary keeps track of information from each cell: 
	{ 
		(row, col): [blocked, nx, cx, bx, ex, hx],
	}
	key: (row, col) 
	- blocked: boolean value, 0 unblocked, 1 blocked 
	- nx: number of neighbors
	- cx: number of neighbors sensed to be blocked
	- bx: number of neighbors confirmed to be blocked
	- ex: number of neighbors confirmed to be empty
	- hx: number of neighbors not confirmed to be blocked or empty 
	""" 
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
		# blocked, nx, cx always recalculated
		blocked = None
		nx = 0
		cx = 0
		# bx, ex, hx come from dictionary if available
		bx = None
		ex = None
		hx = None

		# check if currnode is blocked
		if (gridworld[currNode[0]][currNode[1]] == 1): 
			blocked = 1
			gridworldcopy[row][col] = 1
		else: 
			blocked = 0

		neighbor_result = generate_neighbors(currNode, gridworld, dim)
		nx = neighbor_result[0]
		cx = neighbor_result[1]
		neighbors_list = neighbor_result[2]

		if (currNode in neighbor_info_dictionary.keys()): 
			bx = neighbor_info_dictionary[currNode][3]
			ex = neighbor_info_dictionary[currNode][4]
			hx = neighbor_info_dictionary[currNode][5]
		else: 
			bx = 0
			ex = 0
			hx = 0

		# check if neighbors are confirmed blocked, empty, hidden
		for neighbor in neighbors_list: 
			if (not neighbor[0] == None) and (not neighbor[1] == None):
				# if the neighbor is already in the dictionary 
				if (neighbor in neighbor_info_dictionary.keys()):
					if neighbor[0] == 1: 
						# neighbor has been visited and is blocked
						bx += 1 
					elif neighbor[0] == 0: 
						# neighbor has been visited and is empty
						ex += 1
					else: 
						# neighbor has been neighbor to another cell, but not been visited
						pass

					# if current node is blocked, update confirmed blocked cells otherwise update empty cells
					if (blocked == 1): 
						# add 1 confirmed blocked to neighbor in dictionary
						neighbor_info_dictionary[neighbor][3] += 1
					else: 
						# add 1 confirmed empty to neighbor in dictionary
						neighbor_info_dictionary[neighbor][4] += 1
				else: 
					# add neighbor to dictionary (with whether or not current cell is blocked)
					if (blocked == 1):
						neighbor_info_dictionary[neighbor] = [None, None, None, 1, 0, 0]
					else: 
						neighbor_info_dictionary[neighbor] = [None, None, None, 0, 1, 0]


		hx = nx - ex - bx
		neighbor_info_dictionary[currNode] = [blocked, nx, cx, bx, ex, hx]

		"""
		if blocked: 
			get new path with previous node as start, gridworldcopy as gridworld
			currNode is second node in new path 
		else: 
			add node to final path
			make the previous node the current nodes
			make curr node the next node in the path
		"""

		if (blocked == 1): 
			astarresult = astarwithstart(dim, gridworldcopy, prevNode, heuristic)
			cells_processed += astarresult[0]
			path = astarresult[1]
			if (len(path) == 0):
				break
			elif (len(path) == 1): 
				if (path[0][0] == dim-1) and (path[0][1] == dim-1): 
					finalpath.append(path[0])
					return(len(finalpath, cells_processed, gridworldcopy, finalpath))
			else: 
				i = 1
				currNode = path[i]
		else: 
			# cell is not blocked
			finalpath.append(currNode) 
			prevNode = currNode
			currNode = path[i+1]
			i = i+1
	if (len(path) != 0):
		finalpath.append(currNode)
	else:
		return(-1, cells_processed, gridworldcopy, finalpath)
	return(len(finalpath), cells_processed, gridworldcopy, finalpath)

def inferenceagent(dim, gridworld, heuristic):
	start = (0,0)
	goal = (dim-1,dim-1)
	cells_processed = 0
	gridworldcopy = generategridworld(dim, 0)
	# gridworld_neighbor_blocks keeps track of number of neighbors that are blocked at each cell
	gridworld_neighbor_blocks = generategridworld(dim, 0)
	neighbor_info_dictionary = {}
	""" 
	dictionary keeps track of information from each cell: 
	{ 
		(row, col): [blocked, nx, cx, bx, ex, hx],
	}
	key: (row, col) 
	- blocked: boolean value, 0 unblocked, 1 blocked 
	- nx: number of neighbors
	- cx: number of neighbors sensed to be blocked
	- bx: number of neighbors confirmed to be blocked
	- ex: number of neighbors confirmed to be empty
	- hx: number of neighbors not confirmed to be blocked or empty 
	""" 
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
		# blocked, nx, cx always recalculated
		blocked = None
		nx = 0
		cx = 0
		# bx, ex, hx come from dictionary if available
		bx = None
		ex = None
		hx = None
		cbx = None

		# check if currnode is blocked
		if (gridworld[currNode[0]][currNode[1]] == 1): 
			blocked = 1
			gridworldcopy[row][col] = 1
		else: 
			blocked = 0

		"""
		# generate coordinates for all neighbors
		north = (currNode[0]-1, currNode[1]) 
		northEast = (currNode[0]-1, currNode[1]+1)
		east =  (currNode[0], currNode[1]+1) 
		southEast = (currNode[0]+1, currNode[1]+1)
		south = (currNode[0]+1, currNode[1]) 
		southWest = (currNode[0]+1, currNode[1]-1)
		west = (currNode[0], currNode[1]-1) 
		northWest = (currNode[0]-1, currNode[1]-1)
		precheck_neighbors_list = [north, northEast, east, southEast, south, southWest, west, northWest]

		neighbors_list = []
		for neighbor in precheck_neighbors_list: # nx 
			if isChildValidRepeatedForward(neighbor, dim): 
				nx +=1
				neighbors_list.append(neighbor)
			else: 
				neighbors_list.append((None,None))
		#print(neighbors_list)

		# check how many neighbors are sensed to be blocked
		for neighbor in neighbors_list:  # cx 
			if (neighbor[0] == None) or (neighbor[1] == None) or (neighbor[0] == -1) or (neighbor[1] == -1):
				pass
			else: 
				#print(neighbor)
				if (gridworld[neighbor[0]][neighbor[1]] == 1): 
					cx += 1
		"""
		neighbor_result = generate_neighbors(currNode, gridworld, dim)
		nx = neighbor_result[0]
		cx = neighbor_result[1]
		neighbors_list = neighbor_result[2]

		if (currNode in neighbor_info_dictionary.keys()): 
			bx = neighbor_info_dictionary[currNode][3]
			ex = neighbor_info_dictionary[currNode][4]
			hx = neighbor_info_dictionary[currNode][5]
			cbx = neighbor_info_dictionary[currNode][6]
		else: 
			bx = 0
			ex = 0
			hx = 0
			cbx = 0

		# check if neighbors are confirmed blocked, empty, hidden
		for neighbor in neighbors_list: 
			if (not neighbor[0] == None) and (not neighbor[1] == None):
				# if the neighbor is already in the dictionary
				if (neighbor in neighbor_info_dictionary.keys()):
					if neighbor[0] == 1: 
						# neighbor has been visited and is blocked
						bx += 1 
					elif neighbor[0] == 0: 
						# neighbor has been visited and is empty
						ex += 1
					else: 
						# neighbor has been neighbor to another cell, but not been visited
						pass

					# if current node is blocked, update confirmed blocked cells otherwise update empty cells
					if (blocked == 1): 
						# add 1 confirmed blocked to neighbor in dictionary
						neighbor_info_dictionary[neighbor][3] += 1
					else: 
						# add 1 confirmed empty to neighbor in dictionary
						neighbor_info_dictionary[neighbor][4] += 1
				else: 
					# add neighbor to dictionary (with whether or not current cell is blocked)
					if (blocked == 1):
						neighbor_info_dictionary[neighbor] = [None, None, None, 1, 0, 0, 0]
					else: 
						neighbor_info_dictionary[neighbor] = [None, None, None, 0, 1, 0, 0]


		hx = nx - ex - bx
		cbx = cx - bx 
		neighbor_info_dictionary[currNode] = [blocked, nx, cx, bx, ex, hx, cbx]

		"""
		if blocked: 
			get new path with previous node as start, gridworldcopy as gridworld
			currNode is second node in new path 
		else: 
			add node to final path
			make the previous node the current nodes
			make curr node the next node in the path
		"""

		one_two_one_blocked_nodes, neighbor_info_dictionary = one_two_one(neighbors_list, neighbor_info_dictionary, dim)
		if (not (len(one_two_one_blocked_nodes) == 0)):
			for node in one_two_one_blocked_nodes:
				neighbor_info_dictionary[node][0] = 1 
				gridworldcopy[node[0]][node[1]] = 1 

		if (blocked == 1): 
			astarresult = astarwithstart(dim, gridworldcopy, prevNode, heuristic)
			cells_processed += astarresult[0]
			path = astarresult[1]
			if (len(path) == 0):
				break
			elif (len(path) == 1): 
				if (path[0][0] == dim-1) and (path[0][1] == dim-1): 
					finalpath.append(path[0])
					return(len(finalpath, cells_processed, gridworldcopy, finalpath))
			else: 
				i = 1
				currNode = path[i]
		else: 
			# cell is not blocked
			finalpath.append(currNode) 
			prevNode = currNode
			currNode = path[i+1]
			i = i+1
	if (len(path) != 0):
		finalpath.append(currNode)
	else:
		return(-1, cells_processed, gridworldcopy, finalpath)
	return(len(finalpath), cells_processed, gridworldcopy, finalpath)

		

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

def one_two_one(neighbors_list, neighbor_info_dictionary, dim): 
	# order neighbors_list: [north, northEast, east, southEast, south, southWest, west, northWest]
	found_mines = []
	north = neighbors_list[0]
	northEast = neighbors_list[1]
	east = neighbors_list[2]
	southEast = neighbors_list[3]
	south = neighbors_list [4]
	southWest = neighbors_list[5]
	west = neighbors_list[6]
	northWest = neighbors_list[7]

	# north edge: northwest, north, northeast
	# east edge: northeast, east, southeast
	# south edge: southwest, south, southeast
	# west edge: northwest, west, southwest 

	#TODO: give each node a cbx value such that cbx = cx - bx 
	if (isChildValidRepeatedForward(northWest, dim) and isChildValidRepeatedForward(north, dim) and isChildValidRepeatedForward(northEast, dim)):
		if ((northWest in neighbor_info_dictionary.keys()) and (north in neighbor_info_dictionary.keys()) and (northEast in neighbor_info_dictionary.keys())):
			nw_cbx = neighbor_info_dictionary[northWest][6]
			n_cbx = neighbor_info_dictionary[north][6]
			ne_cbx = neighbor_info_dictionary[northEast][6]
			if ((nw_cbx == 1) and (n_cbx == 2) and (ne_cbx == 1)):
				# if valid, blocks about nw and ne are blocked
				nw_block = (northWest[0]-1, northWest[1])
				ne_block = (northEast[0]-1, northEast[1])

				if (isChildValidRepeatedForward(nw_block, dim) and isChildValidRepeatedForward(ne_block, dim)):
					found_mines.append(nw_block)
					found_mines.append(ne_block)
					neighbor_info_dictionary = reduce_bx_cbx(cell=nw_block, nb_dict=neighbor_info_dictionary, gridworld=gridworld, dim=dim)
					neighbor_info_dictionary = reduce_bx_cbx(cell=ne_block, nb_dict=neighbor_info_dictionary, gridworld=gridworld, dim=dim)


	if (isChildValidRepeatedForward(northEast, dim) and isChildValidRepeatedForward(east, dim) and isChildValidRepeatedForward(southEast, dim)):
		if ((northEast in neighbor_info_dictionary.keys()) and (east in neighbor_info_dictionary.keys()) and (southEast in neighbor_info_dictionary.keys())):
			ne_cbx = neighbor_info_dictionary[northEast][6]
			e_cbx = neighbor_info_dictionary[east][6]
			se_cbx = neighbor_info_dictionary[southEast][6]
			if ((ne_cbx == 1) and (e_cbx == 2) and (se_cbx == 1)):
				# if valid, blocks east of ne and se are blocked
				ne_block = (northEast[0], northEast[1]+1)
				se_block = (southEast[0], southEast[1]+1)
				if (isChildValidRepeatedForward(ne_block, dim) and isChildValidRepeatedForward(se_block, dim)):
					found_mines.append(ne_block)
					found_mines.append(se_block)
					neighbor_info_dictionary = reduce_bx_cbx(cell=ne_block, nb_dict=neighbor_info_dictionary, gridworld=gridworld, dim=dim)
					neighbor_info_dictionary = reduce_bx_cbx(cell=se_block, nb_dict=neighbor_info_dictionary, gridworld=gridworld, dim=dim)

	if (isChildValidRepeatedForward(southWest, dim) and isChildValidRepeatedForward(south, dim) and isChildValidRepeatedForward(southEast, dim)):
		if ((southWest in neighbor_info_dictionary.keys()) and (south in neighbor_info_dictionary.keys()) and (southEast in neighbor_info_dictionary.keys())):
			sw_cbx = neighbor_info_dictionary[southWest][6]
			s_cbx = neighbor_info_dictionary[south][6]
			se_cbx = neighbor_info_dictionary[southEast][6]
			if ((sw_cbx == 1) and (s_cbx == 2) and (se_cbx == 1)):
				# if valid, blocks south of sw and se are blocked
				sw_block = (southWest[0]+1, southWest[1])
				se_block = (southEast[0]+1, southEast[1])
				if (isChildValidRepeatedForward(sw_block, dim) and isChildValidRepeatedForward(se_block, dim)):
					found_mines.append(sw_block)
					found_mines.append(se_block)
					neighbor_info_dictionary = reduce_bx_cbx(cell=sw_block, nb_dict=neighbor_info_dictionary, gridworld=gridworld, dim=dim)
					neighbor_info_dictionary = reduce_bx_cbx(cell=se_block, nb_dict=neighbor_info_dictionary, gridworld=gridworld, dim=dim)

	if (isChildValidRepeatedForward(northWest, dim) and isChildValidRepeatedForward(west, dim) and isChildValidRepeatedForward(southWest, dim)):
		if ((northWest in neighbor_info_dictionary.keys()) and (west in neighbor_info_dictionary.keys()) and (southWest in neighbor_info_dictionary.keys())):
			nw_cbx = neighbor_info_dictionary[northWest][6]
			w_cbx = neighbor_info_dictionary[west][6]
			sw_cbx = neighbor_info_dictionary[southWest][6]
			if ((nw_cbx == 1) and (w_cbx == 2) and (sw_cbx == 1)):
				#if valid, blocks west of nw and sw are blocked
				nw_block = (northWest[0], northWest[1]-1)
				sw_block = (southWest[0], southWest[1]-1)
				if (isChildValidRepeatedForward(nw_block, dim) and isChildValidRepeatedForward(sw_block, dim)):
					found_mines.append(nw_block)
					found_mines.append(sw_block)
					neighbor_info_dictionary = reduce_bx_cbx(cell=nw_block, nb_dict=neighbor_info_dictionary, gridworld=gridworld, dim=dim)
					neighbor_info_dictionary = reduce_bx_cbx(cell=sw_block, nb_dict=neighbor_info_dictionary, gridworld=gridworld, dim=dim)
	
	return found_mines, neighbor_info_dictionary


def one_two_two_one(current_cell, gridworld, neighbors_list, neighbor_info_dictionary, dim):
	# infer when current cell's cbx value = 2
	# order neighbors_list: [north, northEast, east, southEast, south, southWest, west, northWest]

	found_mines = []

	current_cbx = neighbor_info_dictionary[current_cell][6]

	north = neighbors_list[0]
	north2 = (north[0]-1, north[1])

	northEast = neighbors_list[1]
	northEast2 = (northEast[0], northEast[1]+1)
	north2East = (northEast[0]-1, northEast[1])

	east = neighbors_list[2]
	east2 = (east[0], east[1]+1)
	
	southEast = neighbors_list[3]
	southEast2 = (southEast[0], southEast[1]+1)
	south2East = (southEast[0]+1, southEast[1])

	south = neighbors_list [4]
	south2 = (south[0]+1, south[1])

	southWest = neighbors_list[5]
	southWest2 = (southWest[0], southWest[1]-1)
	southWest3 = (southWest[0], southWest[1]-1)
	south2West = (southWest[0]+1, southWest[1])
	
	west = neighbors_list[6]
	west2 = (west[0], west[1]-1)
	west3 = (west[0], west[1]-2)

	northWest = neighbors_list[7]
	northWest2 = (northWest[0], northWest[1]-1)
	northWest3 = (northWest[0], northWest[1]-2)
	north2West = (northWest[0]-1, northWest[1])

	# north edge: northwest, north, northeast
	# east edge: northeast, east, southeast
	# south edge: southwest, south, southeast
	# west edge: northwest, west, southwest 

	# horizontal inference from back 
	'''
	[_][_][_][@_]	
	[1][2][2][@1]
	[/][*][*][@/]

	[/][*][*][@/]	
	[1][2][2][@1]
	[_][_][_][@_]
	'''
	if current_cbx == 1:
		if (isChildValidRepeatedForward(west3, dim) and isChildValidRepeatedForward(west2, dim) and isChildValidRepeatedForward(west, dim)):
			if ((west3 in neighbor_info_dictionary.keys()) and (west2 in neighbor_info_dictionary.keys()) and (west in neighbor_info_dictionary.keys())):
				www_cbx = neighbor_info_dictionary[west3][6]
				ww_cbx = neighbor_info_dictionary[west2][6]
				w_cbx = neighbor_info_dictionary[west][6]
				if ((www_cbx == 1) and (ww_cbx == 2) and (w_cbx == 2)):
					up_neighbors = [northWest3, northWest2, northWest, north]
					low_neighbors = [southWest3, southWest2, southWest, south]
					# if up neighbors not visited yet
					if all(key not in neighbor_info_dictionary and isChildValidRepeatedForward(key, dim) for key in up_neighbors):
						# if valid, blocks about nw and nw2 are blocked
						for block in up_neighbors[1, 2]:
							found_mines.append(block)
							neighbor_info_dictionary = reduce_bx_cbx(cell=block, nb_dict=neighbor_info_dictionary, gridworld=gridworld, dim=dim)
					# if low neighbors not visted yet
					elif all(key not in neighbor_info_dictionary and isChildValidRepeatedForward(key, dim) for key in low_neighbors):
						# if valid, blocks about sw and sw2 are blocked
						for block in low_neighbors[1, 2]:
							found_mines.append(block)
							neighbor_info_dictionary = reduce_bx_cbx(cell=block, nb_dict=neighbor_info_dictionary, gridworld=gridworld, dim=dim)

	
	if current_cbx == 2:
		# horizontal inference
		'''
		[/][ *][*][/]	[_][ _][_][_]	
		[1][@2][2][1]	[1][@2][2][1]
		[_][ _][_][_]	[/][ *][*][/]
		
		[/][*][ *][/]	[_][_][ _][_]
		[1][2][@2][1]	[1][2][@2][1]
		[_][_][ _][_]	[/][*][ *][/]
		'''
		if (isChildValidRepeatedForward(west, dim) and isChildValidRepeatedForward(east, dim) and isChildValidRepeatedForward(east2, dim)):
			if ((west in neighbor_info_dictionary.keys()) and (east in neighbor_info_dictionary.keys()) and (east2 in neighbor_info_dictionary.keys())):
				w_cbx = neighbor_info_dictionary[west][6]
				e_cbx = neighbor_info_dictionary[east][6]
				ee_cbx = neighbor_info_dictionary[east2][6]
				if ((w_cbx == 1) and (e_cbx == 2) and (ee_cbx == 1)):
					up_neighbors = [northWest, north, northEast, northEast2]
					low_neighbors = [southWest, south, southEast, southEast2]
					if all(key not in neighbor_info_dictionary and isChildValidRepeatedForward(key, dim) for key in up_neighbors):
						# if valid, blocks about n and ne are blocked
						for block in up_neighbors[1, 2]:
							found_mines.append(block)
							neighbor_info_dictionary = reduce_bx_cbx(cell=block, nb_dict=neighbor_info_dictionary, gridworld=gridworld, dim=dim)
					elif all(key not in neighbor_info_dictionary and isChildValidRepeatedForward(key, dim) for key in low_neighbors):
						# if valid, blocks about s and se are blocked
						for block in low_neighbors[1, 2]:
							found_mines.append(block)
							neighbor_info_dictionary = reduce_bx_cbx(cell=block, nb_dict=neighbor_info_dictionary, gridworld=gridworld, dim=dim)

		if (isChildValidRepeatedForward(west2, dim) and isChildValidRepeatedForward(west, dim) and isChildValidRepeatedForward(east, dim)):
			if ((west2 in neighbor_info_dictionary.keys()) and (west in neighbor_info_dictionary.keys()) and (east in neighbor_info_dictionary.keys())):
				ww_cbx = neighbor_info_dictionary[west2][6]
				w_cbx = neighbor_info_dictionary[west][6]
				e_cbx = neighbor_info_dictionary[east][6]
				if ((ww_cbx == 1) and (w_cbx == 2) and (e_cbx == 1)):
					up_neighbors = [northWest2, northWest, north, northEast]
					low_neighbors = [southWest2, southWest, south, southEast]
					# if up neighbors not visited yet
					if all(key not in neighbor_info_dictionary and isChildValidRepeatedForward(key, dim) for key in up_neighbors):
						# if valid, blocks about nw and n are blocked
						for block in up_neighbors[1, 2]:
							found_mines.append(block)
							neighbor_info_dictionary = reduce_bx_cbx(cell=block, nb_dict=neighbor_info_dictionary, gridworld=gridworld, dim=dim)
					# if low neighbors not visted yet
					elif all(key not in neighbor_info_dictionary and isChildValidRepeatedForward(key, dim) for key in low_neighbors):
						# if valid, blocks about sw and s are blocked
						for block in low_neighbors[1, 2]:
							found_mines.append(block)
							neighbor_info_dictionary = reduce_bx_cbx(cell=block, nb_dict=neighbor_info_dictionary, gridworld=gridworld, dim=dim)
		
		# vertical inference
		'''
		[/][ 1][]		[][ 1][/]
		[*][@2][]		[][@2][*]
		[*][ 2][]		[][ 2][*]
		[/][ 1][]		[][ 1][/]

		[/][ 1][]		[][ 1][/]
		[*][ 2][]		[][ 2][*]
		[*][@2][]		[][@2][*]
		[/][ 1][]		[][ 1][/]	
		'''
		if (isChildValidRepeatedForward(north, dim) and isChildValidRepeatedForward(south, dim) and isChildValidRepeatedForward(south2, dim)):
			if ((north in neighbor_info_dictionary.keys()) and (south in neighbor_info_dictionary.keys()) and (south2 in neighbor_info_dictionary.keys())):
				n_cbx = neighbor_info_dictionary[north][6]
				s_cbx = neighbor_info_dictionary[south][6]
				ss_cbx = neighbor_info_dictionary[south2][6]
				if ((n_cbx == 1) and (s_cbx == 2) and (ss_cbx == 1)):
					right_neighbors = [northEast, east, southEast, south2East]
					left_neighbors = [northWest, west, southWest, south2West]
					# if right neighbors not visited yet
					if all(key not in neighbor_info_dictionary and isChildValidRepeatedForward(key, dim) for key in right_neighbors):
						# if valid, blocks about e and se are blocked
						for block in right_neighbors[1, 2]:
							found_mines.append(block)
							neighbor_info_dictionary = reduce_bx_cbx(cell=block, nb_dict=neighbor_info_dictionary, gridworld=gridworld, dim=dim)
					# if left neighbors not visted yet
					elif all(key not in neighbor_info_dictionary and isChildValidRepeatedForward(key, dim) for key in left_neighbors):
						# if valid, blocks about w and sw are blocked
						for block in left_neighbors[1, 2]:
							found_mines.append(block)
							neighbor_info_dictionary = reduce_bx_cbx(cell=block, nb_dict=neighbor_info_dictionary, gridworld=gridworld, dim=dim)	

		if (isChildValidRepeatedForward(north2, dim) and isChildValidRepeatedForward(north, dim) and isChildValidRepeatedForward(south, dim)):
			if ((north2 in neighbor_info_dictionary.keys()) and (north in neighbor_info_dictionary.keys()) and (south in neighbor_info_dictionary.keys())):
				nn_cbx = neighbor_info_dictionary[north2][6]
				n_cbx = neighbor_info_dictionary[north][6]
				s_cbx = neighbor_info_dictionary[south][6]
				if ((nn_cbx == 1) and (n_cbx == 2) and (s_cbx == 1)):
					right_neighbors = [north2East, northEast, east, southEast]
					left_neighbors = [north2West, northWest, west, southWest]
					# if right neighbors not visited yet
					if all(key not in neighbor_info_dictionary and isChildValidRepeatedForward(key, dim) for key in right_neighbors):
						# if valid, blocks about e and se are blocked
						for block in right_neighbors[1, 2]:
							found_mines.append(block)
							neighbor_info_dictionary = reduce_bx_cbx(cell=block, nb_dict=neighbor_info_dictionary, gridworld=gridworld, dim=dim)
					# if left neighbors not visted yet
					elif all(key not in neighbor_info_dictionary and isChildValidRepeatedForward(key, dim) for key in left_neighbors):
						# if valid, blocks about w and sw are blocked
						for block in left_neighbors[1, 2]:
							found_mines.append(block)
							neighbor_info_dictionary = reduce_bx_cbx(cell=block, nb_dict=neighbor_info_dictionary, gridworld=gridworld, dim=dim)

	return found_mines, neighbor_info_dictionary


def reduce_bx_cbx(cell, nb_dict, gridworld, dim):
	nb_list = generate_neighbors(cell=cell, gridworld=gridworld, dim=dim)[2]
	for nb in nb_list:
		nb_dict[nb][3] -= 1 # subtract 1 from bx value
		nb_dict[nb][6] -= 1 # subtract 1 from cbx value
	return nb_dict



#gridworld = generategridworld(10, 0.24)
#printAsGrid(gridworld)
#exampleresult = exampleinferenceagent(len(gridworld),gridworld,manhattan)
#inferenceresult = inferenceagent(len(gridworld), gridworld, manhattan)
#print("example length: " + str(exampleresult[0]))
#print("example path: " + str(exampleresult[3]))
#print("inference length: " + str(inferenceresult[0]))
#print("inference path: " + str(inferenceresult[3]))
#print(exampleresult)
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

"""
gridworldTest = [[0, 0, 0, 0, 0, 1, 0, 0, 0, 1],
[0, 1, 0, 1, 1, 0, 0, 0, 0, 0],
[0, 0, 1, 0, 0, 0, 0, 1, 0, 0],
[0, 1, 1, 0, 0, 0, 1, 0, 0, 0],
[0, 1, 0, 0, 1, 0, 0, 1, 0, 0],
[0, 1, 0, 1, 0, 1, 0, 1, 0, 0],
[0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
[0, 1, 0, 0, 1, 0, 1, 0, 0, 0],
[0, 0, 0, 1, 1, 0, 0, 0, 0, 0]]
printAsGrid(gridworldTest)
path = astarwithstart(10, gridworldTest, (0,1), "manhattan")
print("")
print("a star:")
print(path)
path = repeatedforwardastar(10, gridworldTest, "manhattan")
print("")
print("repeated forward a star:")
printAsGrid(path[2])
print(path[3])
path = exampleinferenceagent(10, gridworldTest, "manhattan")
print("")
print("example inference agent:")
printAsGrid(path[2])
print(path[3])
"""

gridworld = generategridworld(101, 0.2)
#printAsGrid(gridworld)
#astar(10,gridworld)
#path = astarwithstart(10, gridworld, (0,0), "manhattan")
#print(path)
path = repeatedforwardastar(101,gridworld, "manhattan")
print("repeated forward length: " + str(path[0]))
print("repeated forward cells processed: " + str(path[1]))
path = repeatedforwardastar_restricted(101,gridworld, "manhattan")
print("repeated forward restricted length: " + str(path[0]))
print("repeated forward restricted cells processed: " + str(path[1]))
#print(path[0])
#print(path[1])
#print(path[3])
