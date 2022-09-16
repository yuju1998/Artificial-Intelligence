import numpy as np
import math
import random
from priorityq import PriorityQueue


def generate_gridworld_with_terrain(dim, p):
    # -1: blocked
    # 1: flat
    # 2: hilly
    # 3: forest
    # TODO: need to pass in goal to make sure goal is not blocked
    terrains = [1, 2, 3]
    gridworld = [[random.choice(terrains) for i in range(dim)] for j in range(dim)]
    for row in range(len(gridworld)):
        for col in range(len(gridworld)):
            if not ((row == 0 and col == 0) or (row == dim - 1 and col == dim - 1)):
                x = random.uniform(0, 1)
                if (x <= p):
                    gridworld[row][col] = -1
    return gridworld


def generate_gridworld(dim, p):
    # dim == dimensions of gridworld
    # p == probability of 1, 0
    gridworld = [[0 for i in range(dim)] for j in range(dim)]
    for row in range(len(gridworld)):
        for col in range(len(gridworld)):
            if (row == 0 and col == 0) or (row == dim - 1 and col == dim - 1):
                gridworld[row][col] = 0
            else:
                x = random.uniform(0, 1)
                if (x <= p):
                    gridworld[row][col] = 1
    return gridworld


def astar_withgoal(dim, gridworld, heuristic, goal):
    nodes_visited = 0
    start = (0, 0)
    # goal = (dim-1, dim-1)
    # dictionary of parents {coordinates of curr: parents}
    parents = {}
    # parents = {key: node, value: (parent, expanded? [boolean])}

    # grid of distances where distancemap[x][y] is
    # the length of the shortest path so far from the start to (x, y)
    gScoreMap = [[-1 for i in range(dim)] for j in range(dim)]
    gScoreMap[0][0] = 0

    pastNode = (None, None)

    # first node
    # nodes are in the form (f(n), (x, y))
    if (heuristic == 'euclidean'):
        currNode = (euclidean(0, dim - 1, 0, dim - 1), start)
    elif (heuristic == 'chebyshev'):
        currNode = (chebyshev(0, dim - 1, 0, dim - 1), start)
    else:
        currNode = (manhattan(0, dim - 1, 0, dim - 1), start)
    currDistance = 0
    expanded = {}
    fringe = PriorityQueue()  # priority queue
    fringe.push(currNode, 0)
    prevNode = None
    while (fringe.isEmpty() == False):
        prevNode = currNode
        currNode = fringe.pop()
        nodes_visited += 1
        expanded.update({currNode: True})
        # if we are not at the goal node
        if not ((currNode[1][0] == goal[0]) and (currNode[1][1] == goal[1])):
            currDistance = gScoreMap[currNode[1][0]][currNode[1][1]]
            childrenDict = generateChildren(currNode, currDistance, pastNode, gridworld, parents, heuristic)
            # priority = -1
            for key in childrenDict.keys():
                # TODO: what should we do if the key is already in the parents dictionary?
                if (not childrenDict.get(key)[0] == -1) and not (childrenDict.get(key)[1] == (None, None)):
                    # the distance of the path from the start to the current child node
                    currPathDistance = gScoreMap[currNode[1][0]][currNode[1][1]] + 1
                    neighborNode = childrenDict.get(key)[1]
                    # if the child node is unexplored or if the path is shorter than the previous shortest path
                    if (gScoreMap[neighborNode[0]][neighborNode[1]] == -1) or (
                            currPathDistance < gScoreMap[neighborNode[0]][neighborNode[1]]):
                        # update parent dictionary
                        parents.update({neighborNode: currNode[1]})
                        expanded.update({neighborNode: False})
                        # update shortest path length to child node
                        gScoreMap[neighborNode[0]][neighborNode[1]] = currPathDistance
                        # add child node to fringe
                        tempNode = (childrenDict.get(key)[0], childrenDict.get(key)[1])
                        fringe.push(tempNode, childrenDict.get(key)[0])

        else:
            # at goal node
            pathNode = currNode[1]
            path = [pathNode]
            while (not ((pathNode[0] == 0) and (pathNode[1] == 0))):
                parentNode = parents.get(pathNode)
                path.insert(0, parentNode)
                pathNode = parentNode
            # return path
            return (True, nodes_visited, len(path))
    # print("fringe is empty, gridworld not solvable")
    # return (-1,-1)
    return (False, nodes_visited, -1)


def astarwithstart(dim, gridworld, start, heuristic):
    # start = (0,0)
    # goal = (dim-1, dim-1)
    # dictionary of parents {coordinates of curr: parents}
    parents = {}
    # parents = {key: node, value: (parent, expanded? [boolean])}

    # grid of distances where distancemap[x][y] is
    # the length of the shortest path so far from the start to (x, y)
    gScoreMap = [[-1 for i in range(dim)] for j in range(dim)]
    gScoreMap[start[0]][start[1]] = 0

    pastNode = (None, None)
    nodes_visited = 0

    # first node
    # nodes are in the form (f(n), (x, y))
    if (heuristic == 'euclidean'):
        currNode = (euclidean(start[0], dim - 1, start[1], dim - 1), start)
    elif (heuristic == 'chebyshev'):
        currNode = (chebyshev(start[0], dim - 1, start[1], dim - 1), start)
    else:
        currNode = (manhattan(start[0], dim - 1, start[1], dim - 1), start)
    currDistance = 0
    expanded = {}
    fringe = PriorityQueue()  # priority queue
    fringe.push(currNode, 0)
    prevNode = None
    while (fringe.isEmpty() == False):
        prevNode = currNode
        currNode = fringe.pop()
        nodes_visited += 1
        expanded.update({currNode: True})
        # if we are not at the goal node
        if not ((currNode[1][0] == dim - 1) and (currNode[1][1] == dim - 1)):
            currDistance = gScoreMap[currNode[1][0]][currNode[1][1]]
            childrenDict = generateChildren(currNode, currDistance, pastNode, gridworld, parents, heuristic)
            # priority = -1
            for key in childrenDict.keys():
                # TODO: what should we do if the key is already in the parents dictionary?
                if (childrenDict.get(key)[0] > -1) and not (childrenDict.get(key)[1] == (None, None)):
                    # the distance of the path from the start to the current child node
                    currPathDistance = gScoreMap[currNode[1][0]][currNode[1][1]] + 1
                    neighborNode = childrenDict.get(key)[1]
                    # if the child node is unexplored or if the path is shorter than the previous shortest path
                    if (gScoreMap[neighborNode[0]][neighborNode[1]] == -1) or (
                            currPathDistance < gScoreMap[neighborNode[0]][neighborNode[1]]):
                        # update parent dictionary
                        parents.update({neighborNode: currNode[1]})
                        expanded.update({neighborNode: False})
                        # update shortest path length to child node
                        gScoreMap[neighborNode[0]][neighborNode[1]] = currPathDistance
                        # add child node to fringe
                        tempNode = (childrenDict.get(key)[0], childrenDict.get(key)[1])
                        # print(tempNode)
                        fringe.push(tempNode, childrenDict.get(key)[0])
        else:
            # at goal node
            pathNode = currNode[1]
            path = [pathNode]
            while (not ((pathNode[0] == start[0]) and (pathNode[1] == start[1]))):
                parentNode = parents.get(pathNode)
                path.insert(0, parentNode)
                pathNode = parentNode
            return (nodes_visited, path)
    # fringe is empty, didn't reach goal node
    # print("fringe is empty, gridworld not solvable")
    return (nodes_visited, [])


def astarwithstartgoal(dim, gridworld, start, goal):
    heuristic = "manhattan"
    # start = (0,0)
    # dictionary of parents {coordinates of curr: parents}
    parents = {}
    # parents = {key: node, value: (parent, expanded? [boolean])}

    # grid of distances where distancemap[x][y] is
    # the length of the shortest path so far from the start to (x, y)
    gScoreMap = [[-1 for i in range(dim)] for j in range(dim)]
    gScoreMap[start[0]][start[1]] = 0

    pastNode = (None, None)
    nodes_visited = 0

    # first node
    # nodes are in the form (f(n), (x, y))
    if (heuristic == 'euclidean'):
        currNode = (euclidean(start[0], goal[0], start[1], goal[1]), start)
    elif (heuristic == 'chebyshev'):
        currNode = (chebyshev(start[0], goal[0], start[1], goal[1]), start)
    else:
        currNode = (manhattan(start[0], goal[0], start[1], goal[1]), start)
    currDistance = 0
    expanded = {}
    fringe = PriorityQueue()  # priority queue
    fringe.push(currNode, 0)
    prevNode = None
    while (fringe.isEmpty() == False):
        prevNode = currNode
        currNode = fringe.pop()
        nodes_visited += 1
        expanded.update({currNode: True})
        # if we are not at the goal node
        if not ((currNode[1][0] == goal[0]) and (currNode[1][1] == goal[1])):
            currDistance = gScoreMap[currNode[1][0]][currNode[1][1]]
            childrenDict = generateChildren(currNode, currDistance, pastNode, gridworld, parents, heuristic)
            # priority = -1
            for key in childrenDict.keys():
                # TODO: what should we do if the key is already in the parents dictionary?
                if (childrenDict.get(key)[0] > -1) and not (childrenDict.get(key)[1] == (None, None)):
                    # the distance of the path from the start to the current child node
                    currPathDistance = gScoreMap[currNode[1][0]][currNode[1][1]] + 1
                    neighborNode = childrenDict.get(key)[1]
                    # if the child node is unexplored or if the path is shorter than the previous shortest path
                    if (gScoreMap[neighborNode[0]][neighborNode[1]] == -1) or (
                            currPathDistance < gScoreMap[neighborNode[0]][neighborNode[1]]):
                        # update parent dictionary
                        parents.update({neighborNode: currNode[1]})
                        expanded.update({neighborNode: False})
                        # update shortest path length to child node
                        gScoreMap[neighborNode[0]][neighborNode[1]] = currPathDistance
                        # add child node to fringe
                        tempNode = (childrenDict.get(key)[0], childrenDict.get(key)[1])
                        # print(tempNode)
                        fringe.push(tempNode, childrenDict.get(key)[0])
        else:
            # at goal node
            pathNode = currNode[1]
            path = [pathNode]
            while (not ((pathNode[0] == start[0]) and (pathNode[1] == start[1]))):
                parentNode = parents.get(pathNode)
                path.insert(0, parentNode)
                pathNode = parentNode
            return (True, nodes_visited, path)
    # fringe is empty, didn't reach goal node
    # print("fringe is empty, gridworld not solvable")
    return (False, nodes_visited, [])


def agent_six(dim, gridworld, start, goal_actual):
    # generate gridworld to keep track of blocks found, terrain, etc.
    gridworld_copy = generate_gridworld(dim, 0)
    weight_dict = {}
    """
    weight_dict: {
        (x,y): {
            examined: 
            weight: 	
        }
    }
    """
    pij_weight = 1
    goal_current = start
    curr = start
    prev = start
    highest = None
    solvable = astarwithstartgoal(dim, gridworld, start, goal_actual)[0]
    blocked_cells = 0
    cells_examined = 0
    moves_made = 0
    unreachable_nodes = []
    if solvable == True:
        path = astarwithstartgoal(dim, gridworld_copy, start, goal_current)[2]
        i = 0
        while (i <= len(path) - 1):
            # print("goal_current", goal_current)
            moves_made += 1
            prev = curr
            curr = path[i]
            curr_row = curr[0]
            curr_col = curr[1]
            # if curr != goal_current
            if (not curr == goal_current):
                # if curr == blocked
                if (gridworld[curr_row][curr_col] == -1):
                    # print("found block midlist")
                    blocked_cells += 1
                    gridworld_copy[curr_row][curr_col] = -1
                    # regenerate path from prev node
                    cells_examined += 1
                    goal_current = generate_new_goal(dim, curr, weight_dict, pij_weight, gridworld_copy,
                                                     unreachable_nodes)
                    path = astarwithstartgoal(dim, gridworld_copy, prev, goal_current)[2]
                    goal_prev = goal_current
                    while (len(path) == 0):
                        # regenerate goal node
                        unreachable_nodes.append(goal_prev)
                        goal_current = generate_new_goal(dim, prev, weight_dict, pij_weight, gridworld_copy,
                                                         unreachable_nodes)
                        # print("goal_current in midlist: ", goal_current)
                        path = astarwithstartgoal(dim, gridworld_copy, prev, goal_current)[2]
                        # print("path after midlist", path)
                        goal_prev = goal_current
                    i = 0
                    # update weight_dict
                    pij_weight, weight_dict = update_pij_weight(dim, Type="block", pij_weight=pij_weight,
                                                                weight_dict=weight_dict, currNode=curr)


                # if curr == flat
                elif (gridworld[curr_row][curr_col] == 1):
                    gridworld_copy[curr_row][curr_col] = 1
                    i += 1

                # if curr == hilly
                elif (gridworld[curr_row][curr_col] == 2):
                    gridworld_copy[curr_row][curr_col] = 2
                    i += 1

                # else if curr == forest
                else:
                    gridworld_copy[curr_row][curr_col] = 3
                    i += 1
            # if curr == goal_current
            else:
                # if curr is actually goal node
                cells_examined += 1
                if (curr == goal_actual):
                    if (gridworld[curr_row][curr_col] == 1):
                        gridworld_copy[curr_row][curr_col] = 1
                        if random.random() > 0.2:
                            # did not find false negative
                            # print("found it")
                            return (curr, moves_made, cells_examined)

                        # if curr == hilly
                    elif (gridworld[curr_row][curr_col] == 2):
                        gridworld_copy[curr_row][curr_col] = 2
                        if random.random() > 0.5:
                            # did not find false negative
                            # print("found it")
                            return (curr, moves_made, cells_examined)
                    # else if curr == forest

                    else:
                        gridworld_copy[curr_row][curr_col] = 3
                        if random.random() > 0.8:
                            # did not get false negative
                            # print("found it")
                            return (curr, moves_made, cells_examined)
                # print("Goal not found.")
                # if cell has been found previously
                if (curr in weight_dict.keys()):
                    # if curr == flat
                    weight_dict[curr]["examined"] += 1
                    if (gridworld[curr_row][curr_col] == 1):
                        pij_weight, weight_dict = update_pij_weight(dim, "flat", pij_weight, weight_dict, curr)

                    # if curr == hilly
                    elif (gridworld[curr_row][curr_col] == 2):
                        pij_weight, weight_dict = update_pij_weight(dim, "hilly", pij_weight, weight_dict, curr)

                    # else if curr == forest
                    else:
                        pij_weight, weight_dict = update_pij_weight(dim, "forest", pij_weight, weight_dict, curr)

                else:
                    goal_is_block = False
                    if (gridworld[curr_row][curr_col] == -1):
                        # print("found block as goal")
                        blocked_cells += 1
                        gridworld_copy[curr_row][curr_col] = -1
                        pij_weight, weight_dict = update_pij_weight(dim, "block", pij_weight, weight_dict, curr)
                        goal_is_block = True
                    else:

                        # get terrain type of cell
                        # if curr == flat
                        if (gridworld[curr_row][curr_col] == 1):
                            gridworld_copy[curr_row][curr_col] = 1
                            pij_weight, weight_dict = update_pij_weight(dim, "flat", pij_weight, weight_dict, curr)

                        # if curr == hilly
                        elif (gridworld[curr_row][curr_col] == 2):
                            gridworld_copy[curr_row][curr_col] = 2
                            pij_weight, weight_dict = update_pij_weight(dim, "hilly", pij_weight, weight_dict, curr)

                        # else if curr == forest
                        else:
                            gridworld_copy[curr_row][curr_col] = 3
                            pij_weight, weight_dict = update_pij_weight(dim, "forest", pij_weight, weight_dict, curr)

                # generate new goal_current node
                if (goal_is_block):
                    goal_current = generate_new_goal(dim, prev, weight_dict, pij_weight, gridworld_copy,
                                                     unreachable_nodes)
                    path = astarwithstartgoal(dim, gridworld_copy, prev, goal_current)[2]
                    goal_prev = goal_current
                    while (len(path) == 0):
                        # regenerate goal node
                        unreachable_nodes.append(goal_prev)
                        goal_current = generate_new_goal(dim, prev, weight_dict, pij_weight, gridworld_copy,
                                                         unreachable_nodes)
                        path = astarwithstartgoal(dim, gridworld_copy, prev, goal_current)[2]
                        goal_prev = goal_current
                else:
                    goal_current = generate_new_goal(dim, curr, weight_dict, pij_weight, gridworld_copy,
                                                     unreachable_nodes)
                    path = astarwithstartgoal(dim, gridworld_copy, prev, goal_current)[2]
                    goal_prev = goal_current
                    while (len(path) == 0):
                        # regenerate goal node
                        unreachable_nodes.append(goal_current)
                        goal_current = generate_new_goal(dim, curr, weight_dict, pij_weight, gridworld_copy,
                                                         unreachable_nodes)
                        path = astarwithstartgoal(dim, gridworld_copy, curr, goal_current)[2]
                        goal_prev = goal_current
                i = 0
        # print("Exited loop without finding goal")
    else:
        print("Gridworld is not Solvable")
        return None


def agent_seven(dim, gridworld, start, goal_actual):
    # generate gridworld to keep track of blocks found, terrain, etc.
    gridworld_copy = generate_gridworld(dim, 0)
    weight_dict = {}
    """
    weight_dict: {
        (x,y): {
            examined: 
            weight: 	
        }
    }
    """
    pij_weight = 1
    goal_current = start
    curr = start
    prev = start
    highest = None
    solvable = astarwithstartgoal(dim, gridworld, start, goal_actual)[0]
    blocked_cells = 0
    cells_examined = 0
    moves_made = 0
    unreachable_nodes = []
    if solvable == True:
        path = astarwithstartgoal(dim, gridworld_copy, start, goal_current)[2]
        i = 0
        while (i <= len(path) - 1):
            # print("goal_current", goal_current)
            moves_made += 1
            prev = curr
            curr = path[i]
            # print(curr)
            curr_row = curr[0]
            curr_col = curr[1]
            # if curr != goal_current
            if (not curr == goal_current):
                # if curr == blocked
                if (gridworld[curr_row][curr_col] == -1):
                    # print("found block midlist")
                    blocked_cells += 1
                    gridworld_copy[curr_row][curr_col] = -1
                    # regenerate path from prev node
                    cells_examined += 1
                    goal_current = generate_new_goal_seven(dim, curr, weight_dict, pij_weight, gridworld_copy,
                                                           unreachable_nodes)
                    path = astarwithstartgoal(dim, gridworld_copy, prev, goal_current)[2]
                    goal_prev = goal_current
                    while (len(path) == 0):
                        # regenerate goal node
                        unreachable_nodes.append(goal_prev)
                        goal_current = generate_new_goal_seven(dim, prev, weight_dict, pij_weight, gridworld_copy,
                                                               unreachable_nodes)
                        # print("goal_current in midlist: ", goal_current)
                        path = astarwithstartgoal(dim, gridworld_copy, prev, goal_current)[2]
                        # print("path after midlist", path)
                        goal_prev = goal_current
                    i = 0
                    # update weight_dict
                    pij_weight, weight_dict = update_pij_weight(dim, Type="block", pij_weight=pij_weight,
                                                                weight_dict=weight_dict, currNode=curr)


                # if curr == flat
                elif (gridworld[curr_row][curr_col] == 1):
                    gridworld_copy[curr_row][curr_col] = 1
                    i += 1

                # if curr == hilly
                elif (gridworld[curr_row][curr_col] == 2):
                    gridworld_copy[curr_row][curr_col] = 2
                    i += 1

                # else if curr == forest
                else:
                    gridworld_copy[curr_row][curr_col] = 3
                    i += 1
            # if curr == goal_current
            else:
                # if curr is actually goal node
                cells_examined += 1
                if (curr == goal_actual):
                    if (gridworld[curr_row][curr_col] == 1):
                        gridworld_copy[curr_row][curr_col] = 1
                        if random.random() > 0.2:
                            # did not find false negative
                            # print("found it")
                            return (curr, moves_made, cells_examined)

                        # if curr == hilly
                    elif (gridworld[curr_row][curr_col] == 2):
                        gridworld_copy[curr_row][curr_col] = 2
                        if random.random() > 0.5:
                            # did not find false negative
                            # print("found it")
                            return (curr, moves_made, cells_examined)
                    # else if curr == forest

                    else:
                        gridworld_copy[curr_row][curr_col] = 3
                        if random.random() > 0.8:
                            # did not get false negative
                            # print("found it")
                            return (curr, moves_made, cells_examined)
                # print("Goal not found.")
                # if cell has been found previously
                if (curr in weight_dict.keys()):
                    # if curr == flat
                    weight_dict[curr]["examined"] += 1
                    if (gridworld[curr_row][curr_col] == 1):
                        pij_weight, weight_dict = update_pij_weight(dim, "flat", pij_weight, weight_dict, curr)

                    # if curr == hilly
                    elif (gridworld[curr_row][curr_col] == 2):
                        pij_weight, weight_dict = update_pij_weight(dim, "hilly", pij_weight, weight_dict, curr)

                    # else if curr == forest
                    else:
                        pij_weight, weight_dict = update_pij_weight(dim, "forest", pij_weight, weight_dict, curr)

                else:
                    goal_is_block = False
                    if (gridworld[curr_row][curr_col] == -1):
                        # print("found block as goal")
                        blocked_cells += 1
                        gridworld_copy[curr_row][curr_col] = -1
                        pij_weight, weight_dict = update_pij_weight(dim, "block", pij_weight, weight_dict, curr)
                        goal_is_block = True
                    else:
                        # get terrain type of cell
                        # if curr == flat
                        if (gridworld[curr_row][curr_col] == 1):
                            gridworld_copy[curr_row][curr_col] = 1
                            pij_weight, weight_dict = update_pij_weight(dim, "flat", pij_weight, weight_dict, curr)

                        # if curr == hilly
                        elif (gridworld[curr_row][curr_col] == 2):
                            gridworld_copy[curr_row][curr_col] = 2
                            pij_weight, weight_dict = update_pij_weight(dim, "hilly", pij_weight, weight_dict, curr)

                        # else if curr == forest
                        else:
                            gridworld_copy[curr_row][curr_col] = 3
                            pij_weight, weight_dict = update_pij_weight(dim, "forest", pij_weight, weight_dict, curr)

                # generate new goal_current node
                if (goal_is_block):
                    goal_current = generate_new_goal_seven(dim, prev, weight_dict, pij_weight, gridworld_copy,
                                                           unreachable_nodes)
                    #print("goal is block goal current:", goal_current)
                    path = astarwithstartgoal(dim, gridworld_copy, prev, goal_current)[2]
                    goal_prev = goal_current
                    while (len(path) == 0):
                        # regenerate goal node
                        unreachable_nodes.append(goal_prev)
                        goal_current = generate_new_goal_seven(dim, prev, weight_dict, pij_weight, gridworld_copy,
                                                               unreachable_nodes)
                        path = astarwithstartgoal(dim, gridworld_copy, prev, goal_current)[2]
                        goal_prev = goal_current
                else:
                    goal_current = generate_new_goal_seven(dim, curr, weight_dict, pij_weight, gridworld_copy,
                                                           unreachable_nodes)
                    #print("goal is not block goal current:", goal_current)
                    path = astarwithstartgoal(dim, gridworld_copy, prev, goal_current)[2]
                    goal_prev = goal_current
                    while (len(path) == 0):
                        # regenerate goal node
                        unreachable_nodes.append(goal_current)
                        goal_current = generate_new_goal_seven(dim, curr, weight_dict, pij_weight, gridworld_copy,
                                                               unreachable_nodes)
                        path = astarwithstartgoal(dim, gridworld_copy, curr, goal_current)[2]
                        goal_prev = goal_current
                i = 0
        # print("Exited loop without finding goal")
    else:
        print("Gridworld is not Solvable")
        return None

def agent_eight(dim, gridworld, start, goal_actual):
    # generate gridworld to keep track of blocks found, terrain, etc.
    gridworld_copy = generate_gridworld(dim, 0)
    weight_dict = {}
    """
    weight_dict: {
        (x,y): {
            examined: 
            weight: 	
        }
    }
    """
    pij_weight = 1
    goal_current = start
    curr = start
    prev = start
    highest = None
    solvable = astarwithstartgoal(dim, gridworld, start, goal_actual)[0]
    blocked_cells = 0
    cells_examined = 0
    moves_made = 0
    unreachable_nodes = []
    if solvable == True:
        path = astarwithstartgoal(dim, gridworld_copy, start, goal_current)[2]
        i = 0
        while (i <= len(path) - 1):
            # print("goal_current", goal_current)
            moves_made += 1
            prev = curr
            curr = path[i]
            curr_row = curr[0]
            curr_col = curr[1]
            # if curr != goal_current
            if (not curr == goal_current):
                # if curr == blocked
                if (gridworld[curr_row][curr_col] == -1):
                    # print("found block midlist")
                    blocked_cells += 1
                    gridworld_copy[curr_row][curr_col] = -1
                    # regenerate path from prev node
                    cells_examined += 1
                    goal_current = generate_new_goal(dim, curr, weight_dict, pij_weight, gridworld_copy,
                                                     unreachable_nodes)
                    path = astarwithstartgoal(dim, gridworld_copy, prev, goal_current)[2]
                    goal_prev = goal_current
                    while (len(path) == 0):
                        # regenerate goal node
                        unreachable_nodes.append(goal_prev)
                        goal_current = generate_new_goal(dim, prev, weight_dict, pij_weight, gridworld_copy,
                                                         unreachable_nodes)
                        # print("goal_current in midlist: ", goal_current)
                        path = astarwithstartgoal(dim, gridworld_copy, prev, goal_current)[2]
                        # print("path after midlist", path)
                        goal_prev = goal_current
                    i = 0
                    # update weight_dict
                    pij_weight, weight_dict = update_pij_weight(dim, Type="block", pij_weight=pij_weight,
                                                                weight_dict=weight_dict, currNode=curr)


                # if curr == flat
                elif (gridworld[curr_row][curr_col] == 1):
                    gridworld_copy[curr_row][curr_col] = 1
                    i += 1

                # if curr == hilly
                elif (gridworld[curr_row][curr_col] == 2):
                    gridworld_copy[curr_row][curr_col] = 2
                    i += 1

                # else if curr == forest
                else:
                    gridworld_copy[curr_row][curr_col] = 3
                    i += 1
            # if curr == goal_current
            else:
                # if curr is actually goal node
                cells_examined += 1
                if (curr == goal_actual):
                    if (gridworld[curr_row][curr_col] == 1):
                        i = 0
                        num_tests = 2
                        while i < num_tests:
                            if i != 0:
                                cells_examined += 1
                            if random.random() > 0.2:
                                # print("found it")
                                return (curr, moves_made, cells_examined)
                            else:
                                pij_weight, weight_dict = update_pij_weight(dim, "flat", pij_weight, weight_dict, curr)
                            i += 1
                        # if curr == hilly
                    elif (gridworld[curr_row][curr_col] == 2):
                        gridworld_copy[curr_row][curr_col] = 2
                        i = 0
                        num_tests = 5
                        while i < num_tests:
                            if i != 0:
                                cells_examined += 1
                            if random.random() > 0.5:
                                # print("found it")
                                return (curr, moves_made, cells_examined)
                            else:
                                pij_weight, weight_dict = update_pij_weight(dim, "hilly", pij_weight, weight_dict, curr)
                            i += 1
                    else:
                        gridworld_copy[curr_row][curr_col] = 3
                        i = 0
                        num_tests = 14
                        while i < num_tests:
                            if i != 0:
                                cells_examined += 1
                            if random.random() > 0.8:
                                # print("found it")
                                return (curr, moves_made, cells_examined)
                            else:
                                pij_weight, weight_dict = update_pij_weight(dim, "forest", pij_weight, weight_dict, curr)
                            i += 1
                # print("Goal not found.")
                # if cell has been found previously
                if (curr in weight_dict.keys()):
                    # if curr == flat
                    weight_dict[curr]["examined"] += 1
                    if (gridworld[curr_row][curr_col] == 1):
                        pij_weight, weight_dict = update_pij_weight(dim, "flat", pij_weight, weight_dict, curr)

                    # if curr == hilly
                    elif (gridworld[curr_row][curr_col] == 2):
                        pij_weight, weight_dict = update_pij_weight(dim, "hilly", pij_weight, weight_dict, curr)

                    # else if curr == forest
                    else:
                        pij_weight, weight_dict = update_pij_weight(dim, "forest", pij_weight, weight_dict, curr)

                else:
                    goal_is_block = False
                    if (gridworld[curr_row][curr_col] == -1):
                        # print("found block as goal")
                        blocked_cells += 1
                        gridworld_copy[curr_row][curr_col] = -1
                        pij_weight, weight_dict = update_pij_weight(dim, "block", pij_weight, weight_dict, curr)
                        goal_is_block = True
                    else:

                        # get terrain type of cell
                        # if curr == flat
                        if (gridworld[curr_row][curr_col] == 1):
                            gridworld_copy[curr_row][curr_col] = 1
                            pij_weight, weight_dict = update_pij_weight(dim, "flat", pij_weight, weight_dict, curr)

                        # if curr == hilly
                        elif (gridworld[curr_row][curr_col] == 2):
                            gridworld_copy[curr_row][curr_col] = 2
                            pij_weight, weight_dict = update_pij_weight(dim, "hilly", pij_weight, weight_dict, curr)

                        # else if curr == forest
                        else:
                            gridworld_copy[curr_row][curr_col] = 3
                            pij_weight, weight_dict = update_pij_weight(dim, "forest", pij_weight, weight_dict, curr)

                # generate new goal_current node
                if (goal_is_block):
                    goal_current = generate_new_goal(dim, prev, weight_dict, pij_weight, gridworld_copy,
                                                     unreachable_nodes)
                    path = astarwithstartgoal(dim, gridworld_copy, prev, goal_current)[2]
                    goal_prev = goal_current
                    while (len(path) == 0):
                        # regenerate goal node
                        unreachable_nodes.append(goal_prev)
                        goal_current = generate_new_goal(dim, prev, weight_dict, pij_weight, gridworld_copy,
                                                         unreachable_nodes)
                        path = astarwithstartgoal(dim, gridworld_copy, prev, goal_current)[2]
                        goal_prev = goal_current
                else:
                    goal_current = generate_new_goal(dim, curr, weight_dict, pij_weight, gridworld_copy,
                                                     unreachable_nodes)
                    path = astarwithstartgoal(dim, gridworld_copy, prev, goal_current)[2]
                    goal_prev = goal_current
                    while (len(path) == 0):
                        # regenerate goal node
                        unreachable_nodes.append(goal_current)
                        goal_current = generate_new_goal(dim, curr, weight_dict, pij_weight, gridworld_copy,
                                                         unreachable_nodes)
                        path = astarwithstartgoal(dim, gridworld_copy, curr, goal_current)[2]
                        goal_prev = goal_current
                i = 0
        # print("Exited loop without finding goal")
    else:
        print("Gridworld is not Solvable")
        return None

def agent_nine(dim, gridworld, start, goal_actual):
    """
    ** Target moves after Agent senses
    Initially every cell has probability 1/dim^2 that contains target.

	initialize potential_goal = [all cells]

	if curr not reach goal_current yet:

		if bump into block:
			potential_goal.remove(curr)
			replan path from previous node

		if not bump into block:
			agent sense neighbors
			if the agent sense the target:
				we obtain potential_goal as curr_neighbors
				prev_sense = True
				potential_goal = curr_neighbors
				replan path

			if the agent doesn't sense anything:
				if agent sensed target previously(prev_sense = True), means that potential goal is no longer all cells:
					update next potential_goal
					? replan path to nearest node in potential_goal ?
				else:
					agent keep moving towards current goal

		target moves

	if curr reach goal_current:
		agent examine curr

		if curr = goal_actual:
			return curr

		if bump into block:
			potential_goal.remove(curr)
			replan path from previous node

		if not bump into block:
			agent sense neighbors
			if the agent sense the target:
				we obtain potential_goal as curr_neighbors
				prev_sense = True
				potential_goal = curr_neighbors
				replan path

			if the agent doesn't sense anything:
				if agent sensed target previously(prev_sense = True), means that potential goal is no longer all cells:
					update next potential_goal
					? replan path to nearest node in potential_goal ?
				else:
					generate new goal
					replan path
		target moves


    we do not consider terrain type for this part

    """
	# initially all the cells are consider potential goals w/o starting any sensing
    potential_goals = []
    for r in range(1, dim+1):
        for c in range(1, dim+1):
            potential_goals.append((r, c))

    blocked_cells = 0

    gridworld_copy = generate_gridworld(dim, 0)
    curr = start
    prev = start
	curr_neighbors = None
	prev_sense = False
    goal_current = start
    solvable = astarwithstartgoal(dim, gridworld, start, goal)[0]
    
	#metrics
	cells_examined = 0
    moves_made = 0
    
	unreachable_nodes = []

    if solvable == True:
        path = astarwithstartgoal(dim, gridworld_copy, start, goal_current)[2]
        i = 0
        while (i <= len(path) - 1):
            moves_made += 1
            prev = curr
            curr = path[i]
            curr_row = curr[0]
            curr_col = curr[1]
            
            # if not yet reach the current goal
            if not (curr == goal_current):

                # if bump into block
                if gridworld[curr_row][curr_col] == -1:
                    blocked_cells += 1
                    gridworld_copy[curr_row][curr_col] = -1
                    cells_examined += 1
					if curr in potential_goals:
                    	potential_goals.remove(curr)
                    # replan path from prev node
                    goal_current = generate_new_goal_nine(prev, potential_goals)
                    path = astarwithstartgoal(dim, gridworld_copy, prev, goal_current)[2]
                    goal_actual = next_target(goal_actual, dim, gridworld)
					if prev_sense = True:
						potential_goals = next_potential_goals(potential_goals, dim, gridworld_copy)
                    i = 0 

                # if cell is empty, sense the neighbor
                else:
                    curr_neighbors = generate_eight_neighbors(curr, gridworld_copy)
					# if target is sensed in the neighbor
                    if goal_actual in curr_neighbors:
                        potential_goals = []
                        for cn in curr_neighbors:
                            if isChildValid(cn, dim, gridworld_copy):
                                potential_goals.append(cn)
						# replan path from curr node
						goal_current = generate_new_goal_nine(curr, potential_goals)
						path = astarwithstartgoal(dim, gridworld_copy, curr, goal_current)[2]
						goal_actual = next_target(goal_actual, dim, gridworld_copy)
						prev_sense = True
						i = 0
					# if target not sensed in the neighbor
					else:
						goal_actual = next_target(goal_actual, dim, gridworld_copy)
						if prev_sense = True:
							potential_goals = next_potential_goals(potential_goals, dim, gridworld_copy)
						i += 1
            # if reaches the current goal
            else:
				cells_examined += 1
				if curr = goal_actual:
					return (curr, moves_made, cells_examined)
				elif gridworld[curr_row][curr_col] == -1:
					blocked_cells += 1
					gridworld_copy[curr_row][curr_col] = -1
					if curr in potential_goals:
						potential_goals.remove(curr)
					# replan path from prev node
					goal_current = generate_new_goal_nine(prev, potential_goals)
					path = astarwithstartgoal(dim, gridworld_copy, prev, goal_current)[2]
					goal_actual = next_target(goal_actual, dim, gridworld_copy)
					if prev_sense = True:
						potential_goals = next_potential_goals(potential_goals, dim, gridworld_copy)
					i = 0
				# if current goal is neither goal_actual or block, sense neighbor, replan path
				else:
					curr_neighbors = generate_eight_neighbors(curr, gridworld_copy)
					# if target is sensed in the neighbor
                    if goal_actual in curr_neighbors:
                        potential_goals = []
                        for cn in curr_neighbors:
                            if isChildValid(cn, dim, gridworld_copy):
                                potential_goals.append(cn)
						# replan path from curr node
						goal_current = generate_new_goal_nine(curr, potential_goals)
						path = astarwithstartgoal(dim, gridworld_copy, curr, goal_current)[2]
						goal_actual = next_target(goal_actual, dim, gridworld_copy)
						prev_sense = True
					else:
						goal_actual = next_target(goal_actual, dim, gridworld_copy)
						if prev_sense = True:
							potential_goals = next_potential_goals(potential_goals, dim, gridworld_copy)
							goal_current = generate_new_goal_nine(curr, potential_goals)
						
						goal_current = generate_new_goal_nine(curr, potential_goals)
						path = astarwithstartgoal(dim, gridworld_copy, curr, goal_current)[2]
					i = 0

    else:
        print("Gridworld is not Solvable")
        return None

def next_potential_goals(potential_goals, dim, gridworld):
	next_potential = []
	for node in potential_goals:
		nodeRow = node[0]
		nodeCol = node[1]
		north = (nodeRow - 1, nodeCol)
		south = (nodeRow + 1, nodeCol)
		east = (nodeRow, nodeCol + 1)
		west = (nodeRow, nodeCol - 1)
		for d in [north, south, west, east]:
			if isChildValid(d, dim, gridworld):
				next_potential.append(d)
	next_potential = list(set(next_potential))
	return next_potential


def next_target(node, dim, gridworld):
	nodeRow = node[0]
	nodeCol = node[1]

	north = (nodeRow - 1, nodeCol)
    south = (nodeRow + 1, nodeCol)
    east = (nodeRow, nodeCol + 1)
    west = (nodeRow, nodeCol - 1)

	directions = []
	if isChildValid(north, dim, gridworld):
		directions.append(north)
	if isChildValid(south, dim, gridworld):
		directions.append(south)
	if isChildValid(west, dim, gridworld):
		directions.append(west)
	if isChildValid(east, dim, gridworld):
		direction.append(east)

	next_node = random.choice(direction)
	return next_node


def update_pij_weight(dim, Type, pij_weight, weight_dict, currNode):
	'''
	P = 1 / (dim**2)
	sum of all probabilities = sum of each cell's weight * P =  1
	total weight = dim**2

	curr_weight = current cell's weight
	total weight - weight = sum of all the other cell's weight

	1. If a cell is examined flat but do not find target, it's weight becomes weight*0.2
	2. If a cell is examined hilly but do not find target, it's weight becomes weight*0.5
	3. If a cell is examined forest but do not find target, it's weight becomes weight*0.8
	4. If a cell is blocked, it's weight is 0
	'''
	if not (currNode in weight_dict.keys()):
		weight_dict.update({currNode: {"examined": 1, "weight": pij_weight}})
	else:
		weight_dict[currNode]["examined"] += 1

	total_weight = dim**2
	curr_weight = weight_dict[currNode]["weight"]
	other_weight_sum = total_weight - curr_weight
	false_negative = 0

	# if terrain type is flat
	if Type == "flat":
		false_negative = 0.2
	# if terrain type is hilly
	elif Type == "hilly":
		false_negative = 0.5
	# if terrain type is forest
	elif Type == "forest":
		false_negative = 0.8
	elif Type == "block":
		false_negative = 0
	
	diff = curr_weight - (curr_weight*false_negative)
	
	for cell in weight_dict:
		if cell != currNode:
			weight_dict[cell]["weight"] = weight_dict[cell]["weight"] + (weight_dict[cell]["weight"]/other_weight_sum)*diff
		else:
			weight_dict[cell]["weight"] *=  false_negative

	pij_weight = pij_weight + (pij_weight/other_weight_sum*diff)
	
	return pij_weight, weight_dict


def success_prob(node, weight_dict, gridworld_copy, pij_weight):
    '''
	P = 1 / (dim**2)
	pij = node's weight * P

	1. If a cell is learned flat, it's success probability is pij*(1-0.2)
	2. If a cell is learned hill, it's success probability is pij*(1-0.5)
	3. If a cell is learned forest, it's success probability is pij*(1-0.8)
	'''
    node_row = node[0]
    node_col = node[1]
    if node in weight_dict:
        node_weight = weight_dict[node]["weight"]
    else:
        node_weight = pij_weight

    terrain_type = gridworld_copy[node_row][node_col]
    # if terrain type is flat
    if terrain_type == 1:
        false_negative = 0.2
    # if terrain type is hilly
    elif terrain_type == 2:
        false_negative = 0.5
    # if terrain type is forest
    elif terrain_type == 3:
        false_negative = 0.8
    else:
        false_negative = 0

    # if cell is blocked
    if terrain_type == -1:
        success_prob = 0
    # if cell is unvisited, times the expected value of 1-FNR of the three terrain type
    elif terrain_type == 0:
        success_prob = node_weight * ((0.2 + 0.5 + 0.8) / 3)
    # if cell is visited and learned terrain type
    else:
        success_prob = node_weight * (1 - false_negative)

    return success_prob

def sense_goal(node, gridworld, goal):
    neighbor_list = generate_eight_neighbors(node, gridworld)
    for neighbor in neighbor_list:
        if neighbor == goal:
            return True
    return False

def generate_eight_neighbors(node, gridworld):
    currNodeRow = node[0]
    currNodeCol = node[1]
    # initial values of children
    north = (currNodeRow - 1, currNodeCol)
    northeast = (currNodeRow - 1, currNodeCol + 1)
    northwest = (currNodeRow - 1, currNodeCol - 1)
    south = (currNodeRow + 1, currNodeCol)
    southeast = (currNodeRow + 1, currNodeCol + 1)
    southwest = (currNodeRow + 1, currNodeCol - 1)
    east = (currNodeRow, currNodeCol + 1)
    west = (currNodeRow, currNodeCol - 1)

    dim = len(gridworld)
    neighbor_list = []
    # check if each direction is blocked
    if isChildValidNotBlock(north, dim, gridworld):
        neighbor_list.append(north)
    if isChildValidNotBlock(northeast, dim, gridworld):
        neighbor_list.append(northeast)
    if isChildValidNotBlock(northwest, dim, gridworld):
        neighbor_list.append(northwest)
    if isChildValidNotBlock(south, dim, gridworld):
        neighbor_list.append(south)
    if isChildValidNotBlock(southeast, dim, gridworld):
        neighbor_list.append(southeast)
    if isChildValidNotBlock(southwest, dim, gridworld):
        neighbor_list.append(southwest)
    if isChildValidNotBlock(east, dim, gridworld):
        neighbor_list.append(east)
    if isChildValidNotBlock(west, dim, gridworld):
        neighbor_list.append(west)

    return neighbor_list

def generateChildren(currNode, currDistance, pastNode, gridworld, parents, heuristic):
    # currNode (x, y)
    currNodeRow = currNode[1][0]
    currNodeCol = currNode[1][1]
    # initial values of children
    north = (currNodeRow - 1, currNodeCol)
    south = (currNodeRow + 1, currNodeCol)
    east = (currNodeRow, currNodeCol + 1)
    west = (currNodeRow, currNodeCol - 1)

    dim = len(gridworld)
    # check if each direction is blocked
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
        "north": (northPriority, north),
        "south": (southPriority, south),
        "east": (eastPriority, east),
        "west": (westPriority, west)
    }

    return childrenDict


def generate_neighbor_list(currNode, gridworld):
    # currNode (x, y)
    currNodeRow = currNode[0]
    currNodeCol = currNode[1]
    # initial values of children
    north = (currNodeRow - 1, currNodeCol)
    south = (currNodeRow + 1, currNodeCol)
    east = (currNodeRow, currNodeCol + 1)
    west = (currNodeRow, currNodeCol - 1)

    dim = len(gridworld)
    neighbor_list = []
    # check if each direction is blocked
    if isChildValidNotBlock(north, dim, gridworld):
        neighbor_list.append(north)
    if isChildValidNotBlock(south, dim, gridworld):
        neighbor_list.append(south)
    if isChildValidNotBlock(east, dim, gridworld):
        neighbor_list.append(east)
    if isChildValidNotBlock(west, dim, gridworld):
        neighbor_list.append(west)

    return neighbor_list


def generatePriority(childNode, dim, distance, heuristic):
    if (childNode[0] == None) or (childNode[1] == None):
        return -1
    else:
        # right now using manhattan distance by default
        if (heuristic == 'chebyshev'):
            return chebyshev(childNode[0], childNode[1], dim - 1, dim - 1) + distance
        elif (heuristic == 'euclidean'):
            return euclidean(childNode[0], childNode[1], dim - 1, dim - 1) + distance
        else:
            return manhattan(childNode[0], childNode[1], dim - 1, dim - 1) + distance

def generate_new_goal_nine(curr, potential_goals): 
	#TODO: need to add unreachable nodes?
	first_node = potential_goals[0]
	goal_current = first_node
	min_dist = manhattan(curr[0], curr[1], first_node[0], first_node[1])
	for node in potential_goals:
		if node != current:
			dist = manhattan(curr[0], curr[1], node[0], node[1])
			if dist < min_dist:
				min_dist = dist
				goal_current = node
	return goal_current


def generate_new_goal(dim, curr, weight_dict, pij_weight, gridworld, unreachable_nodes):
    # generate new goal_current node
    temp_weight = 0
    highest_weight = 0
    highest_prob_list = []
    for node in weight_dict.keys():
        # if all cells are examined or the highest weight in weight_dict is > pij_weight then only need to check for weight_dict
        if weight_dict[node]["weight"] > highest_weight:
            highest_weight = weight_dict[node]["weight"]
    if highest_weight > pij_weight:
        for node in weight_dict.keys():
            if weight_dict[node]["weight"] == highest_weight:
                if not node in unreachable_nodes:
                    highest_prob_list.append(node)

        # print("length highest prob list higher > pij", len(highest_prob_list))
        if len(highest_prob_list) > 0:
            temp_highest = highest_prob_list[0]
            if (len(highest_prob_list) > 1):
                # pick the first node in highest_prob_list as
                first_node = highest_prob_list[0]
                temp_dist = manhattan(curr[0], curr[1], first_node[0], first_node[1])
                for node in highest_prob_list:
                    dist = manhattan(curr[0], curr[1], node[0], node[1])
                    if dist < temp_dist:
                        temp_dist = dist
                        temp_highest = node
                goal_current = temp_highest
            elif len(highest_prob_list) == 1:
                temp_highest = highest_prob_list[0]
                goal_current = temp_highest
        # print("new goal highest weight > pij:", goal_current)
        return goal_current
    # if not all cells are examined or pij_weight is larger
    # randomly
    else:
        curr_check = [curr]
        max_length = 2 * dim
        i = 0
        potential_goals = []
        while i < max_length:
            next_neighbors = []
            for node in curr_check:
                neighbors_list = generate_neighbor_list(node, gridworld)
                for neighbor in neighbors_list:
                    if not (neighbor in weight_dict.keys()):
                        if not (neighbor in unreachable_nodes):
                            potential_goals.append(neighbor)
                    if neighbor not in next_neighbors:
                        next_neighbors.append(neighbor)
            curr_check = next_neighbors
            if len(potential_goals) > 0:
                break
            i += 1
        if len(potential_goals) > 0:
            goal_current = random.choice(potential_goals)
            # print("len potential goals > 0", goal_current)
            return goal_current
        else:
            for node in weight_dict.keys():
                if weight_dict[node]["weight"] == highest_weight:
                    if not node in unreachable_nodes:
                        highest_prob_list.append(node)

            # print("length highest prob list", len(highest_prob_list))
            if len(highest_prob_list) > 0:
                temp_highest = highest_prob_list[0]
                if (len(highest_prob_list) > 1):
                    # pick the first node in highest_prob_list as
                    first_node = highest_prob_list[0]
                    temp_dist = manhattan(curr[0], curr[1], first_node[0], first_node[1])
                    for node in highest_prob_list:
                        dist = manhattan(curr[0], curr[1], node[0], node[1])
                        if dist < temp_dist:
                            temp_dist = dist
                            temp_highest = node
                    goal_current = temp_highest
                elif len(highest_prob_list) == 1:
                    temp_highest = highest_prob_list[0]
                    goal_current = temp_highest
            # print("new goal pij:", goal_current)
        return goal_current


def generate_new_goal_seven(dim, curr, weight_dict, pij_weight, gridworld, unreachable_nodes):
    # generate new goal_current node
    temp_weight = 0
    highest_weight = 0
    highest_prob_list = []

    for node in weight_dict.keys():
        success_probability = success_prob(node, weight_dict, gridworld, pij_weight)
        if success_probability > highest_weight:
            highest_weight = success_probability

    # if all cells are examined or the highest weight in weight_dict is > the highest pij_weight then only need to check for weight_dict
    if highest_weight > (pij_weight * 0.8):
        for node in weight_dict.keys():
            success_probability = success_prob(node, weight_dict, gridworld, pij_weight)
            if success_probability == highest_weight:
                highest_prob_list.append(node)

        # print("length highest prob list higher > pij", len(highest_prob_list))
        if len(highest_prob_list) > 0:
            temp_highest = highest_prob_list[0]
            if (len(highest_prob_list) > 1):
                # pick the first node in highest_prob_list as
                first_node = highest_prob_list[0]
                temp_dist = manhattan(curr[0], curr[1], first_node[0], first_node[1])
                for node in highest_prob_list:
                    dist = manhattan(curr[0], curr[1], node[0], node[1])
                    if dist < temp_dist:
                        temp_dist = dist
                        temp_highest = node
                goal_current = temp_highest
                return goal_current
            #elif len(highest_prob_list) == 1:
            else:
                temp_highest = highest_prob_list[0]
                goal_current = temp_highest
                return goal_current
    # if not all cells are examined or pij_weight is larger
    # randomly
    else:
        curr_check = [curr]
        max_length = 2 * dim
        i = 0
        potential_goals = []
        while i < max_length:
            next_neighbors = []
            for node in curr_check:
                neighbors_list = generate_neighbor_list(node, gridworld)
                for neighbor in neighbors_list:
                    if not (neighbor in weight_dict.keys()):
                        if not (neighbor in unreachable_nodes):
                            potential_goals.append(neighbor)
                    if neighbor not in next_neighbors:
                        next_neighbors.append(neighbor)
            curr_check = next_neighbors
            if len(potential_goals) > 0:
                break
            i += 1
        if len(potential_goals) > 0:
            goal_current = random.choice(potential_goals)
            # print("len potential goals > 0", goal_current)
            return goal_current
        else:
            for node in weight_dict.keys():
                success_probability = success_prob(node, weight_dict, gridworld, pij_weight)
                if success_probability == highest_weight:
                    highest_prob_list.append(node)

            # print("length highest prob list", len(highest_prob_list))
            if len(highest_prob_list) > 0:
                temp_highest = highest_prob_list[0]
                if (len(highest_prob_list) > 1):
                    # pick the first node in highest_prob_list as
                    first_node = highest_prob_list[0]
                    temp_dist = manhattan(curr[0], curr[1], first_node[0], first_node[1])
                    for node in highest_prob_list:
                        dist = manhattan(curr[0], curr[1], node[0], node[1])
                        if dist < temp_dist:
                            temp_dist = dist
                            temp_highest = node
                    goal_current = temp_highest
                    return goal_current
                #elif len(highest_prob_list) == 1:
                else:
                    temp_highest = highest_prob_list[0]
                    goal_current = temp_highest
                    return goal_current
            # print("new goal pij:", goal_current)


def isChildValid(childNode, dim, gridworld):
    row = childNode[0]
    col = childNode[1]
    if (row < 0) or (row == dim) or (col < 0) or (col == dim):
        return False
    if (gridworld[row][col] == -1):
        return False
    return True


def isChildValidNotBlock(childNode, dim, gridworld):
    row = childNode[0]
    col = childNode[1]
    if (row < 0) or (row == dim) or (col < 0) or (col == dim):
        return False
    return True


def euclidean(x1, y1, x2, y2):
    distance = math.sqrt(((x1 - x2) ** 2) + ((y1 - y2) ** 2))
    return distance


def manhattan(x1, y1, x2, y2):
    distance = (abs(x1 - x2) + abs(y1 - y2))
    return distance


def chebyshev(x1, y1, x2, y2):
    distance = max(abs(x1 - x2), abs(y1 - y2))
    return distance


def print_as_grid(gridworld):
    # prints gridworld as grid
    for i in range(len(gridworld)):
        print(gridworld[i])
