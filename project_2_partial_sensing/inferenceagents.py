import numpy as np
import math 
import random
from priorityq import PriorityQueue
import astar


def isChildValidRepeatedForward(childNode, dim): 
    row = childNode[0]
    col = childNode[1]
    if (row == None) or (col == None) or (row < 0) or (row == dim) or (col < 0) or (col == dim):
        return False
    return True 

def generate_neighbor_list(cell, dim):
    northwest = (cell[0]-1, cell[1]-1)
    north = (cell[0]-1, cell[1])
    northeast = (cell[0]-1, cell[1]+1)
    west = (cell[0], cell[1]-1)
    east = (cell[0], cell[1]+1)
    southwest = (cell[0]+1, cell[1]-1)
    south = (cell[0]+1, cell[1])
    southeast = (cell[0]+1, cell[1]+1)

    precheck_neighbor_list = [northwest, north, northeast, west, east, southwest, south, southeast]
    neighbor_list = []

    for neighbor in precheck_neighbor_list:
        if neighbor[0] >=0 and neighbor[0] < dim and neighbor[1] >= 0 and neighbor[1] < dim:
            neighbor_list.append(neighbor)

    #no (None, None) neighbors
    return neighbor_list

def generate_neighbor_list_with_none(cell, dim):
    northwest = (cell[0]-1, cell[1]-1)
    north = (cell[0]-1, cell[1])
    northeast = (cell[0]-1, cell[1]+1)
    west = (cell[0], cell[1]-1)
    east = (cell[0], cell[1]+1)
    southwest = (cell[0]+1, cell[1]-1)
    south = (cell[0]+1, cell[1])
    southeast = (cell[0]+1, cell[1]+1)

    precheck_neighbor_list = [northwest, north, northeast, west, east, southwest, south, southeast]
    neighbor_list = []

    for neighbor in precheck_neighbor_list:
        if neighbor[0] >=0 and neighbor[0] < dim and neighbor[1] >= 0 and neighbor[1] < dim:
            neighbor_list.append(neighbor)
        else: 
            neighbor_list.append((None, None))

    #no (None, None) neighbors
    return neighbor_list

def calculate_nx(cell, dim):
    nx = 0
    #cell = (row, col)
    top_side = (cell[0] == 0)
    bottom_side = (cell[0] == dim-1)
    left_side = (cell[1] == 0)
    right_side = (cell[1] == dim-1)
    #corner cases
    if (top_side and left_side) or (top_side and right_side) or (bottom_side and left_side) or (bottom_side and right_side):
        nx = 3
    elif (top_side and not left_side and not right_side) or (bottom_side and not left_side and not right_side):
        nx = 5
    elif (left_side and not top_side and not bottom_side) or (right_side and not top_side and not bottom_side):
        nx = 5
    elif (not left_side and not right_side and not top_side and not bottom_side):
        nx = 8
    return nx

def calculate_cx(cell, gridworld, dim):
    blocked_neighbors = 0
    neighbor_list = generate_neighbor_list(cell, dim)
    for neighbor in neighbor_list:
        #neighbor list has been validated through generate_neighbors_list function
        if gridworld[neighbor[0]][neighbor[1]] == 1:
            blocked_neighbors+=1
    return blocked_neighbors

"""
(blocked, nx, cx, bx, ex, hx)

info_grid = [[(None, None, None, 0, 0, None) for i in range(dim)] for j in range(dim)] 

info_grid[row][col] = (blocked, nx, cx, bx, ex, hx)


"""

def update_neighbors_bx(cell, info_grid, gridworld_copy, dim):
    neighbor_list = generate_neighbor_list(cell, dim)
    for neighbor in neighbor_list:
        info_grid[neighbor[0]][neighbor[1]][1] = calculate_nx(neighbor, dim)
        info_grid[neighbor[0]][neighbor[1]][3]+=1   
        #since bx of neighbor changed, we check for cx==bx type inference
        inference_result = cx_bx_inference(neighbor, info_grid, gridworld_copy, dim)
        info_grid = inference_result[0]
        gridworld_copy = inference_result[1]
    return (info_grid, gridworld_copy)

def update_neighbors_ex(cell, info_grid, gridworld_copy, dim):
    neighbor_list = generate_neighbor_list(cell, dim)
    for neighbor in neighbor_list:
        info_grid[neighbor[0]][neighbor[1]][1] = calculate_nx(neighbor, dim)
        info_grid[neighbor[0]][neighbor[1]][4]+=1
        #since ex of neighbor changed, we check for nx-cx==ex type inference
        inference_result = nx_cx_ex_inference(neighbor, info_grid, gridworld_copy, dim)
        info_grid = inference_result[0]
        gridworld_copy = inference_result[1]
    return (info_grid, gridworld_copy)

def cx_bx_inference(cell, info_grid, gridworld_copy, dim):
    #if cx = bx
    if (info_grid[cell[0]][cell[1]][2] is not None):
        if (info_grid[cell[0]][cell[1]][2] == info_grid[cell[0]][cell[1]][3]):
            #print("cx == bx inference made!")
            neighbor_list = generate_neighbor_list(cell, dim)
            for neighbor in neighbor_list:
                #all unconfirmed neighbors of cell are unblocked
                if info_grid[neighbor[0]][neighbor[1]][0] is None:
                    info_grid[neighbor[0]][neighbor[1]][0] = 0
                    gridworld_copy[neighbor[0]][neighbor[1]] = 0
                    update_result = update_neighbors_ex(neighbor, info_grid, gridworld_copy, dim)
                    info_grid = update_result[0]
                    gridworld_copy = update_result[1]
    return (info_grid, gridworld_copy)

def nx_cx_ex_inference(cell, info_grid, gridworld_copy, dim):
    #if nx - cx = ex
    if (info_grid[cell[0]][cell[1]][2] is not None):
        if (info_grid[cell[0]][cell[1]][1] - info_grid[cell[0]][cell[1]][2]) == info_grid[cell[0]][cell[1]][4]:
            #print("nx - cx == ex inference made!")
            neighbor_list = generate_neighbor_list(cell, dim)
            for neighbor in neighbor_list:
                #all unconfirmed neighbors of cell are blocked
                if info_grid[neighbor[0]][neighbor[1]][0] is None:
                    info_grid[neighbor[0]][neighbor[1]][0] = 1
                    gridworld_copy[neighbor[0]][neighbor[1]] = 1
                    update_result = update_neighbors_bx(neighbor, info_grid, gridworld_copy, dim)
                    info_grid = update_result[0]
                    gridworld_copy = update_result[1]
    return (info_grid, gridworld_copy)


"""
neighbor: [blocked, nx, cx, bx, ex, hx]

take neighbor in dict
unblocked:
if cx == bx
then all remaining neighbors of neighbor are unblocked
cx
bx
ex = nx - cx
hx = 0
generate all neighbors of the neighbor to confirm unblock

blocked:
if nx - cx == ex
then all remaining neighbors of neighbor are blocked
generate all neighbors of neighbor to confirm block


"""

def example_inference_agent(dim, gridworld, heuristic):
    start = (0,0)
    goal = (dim-1,dim-1)
    cells_processed = 0
    gridworldcopy = astar.generategridworld(dim, 0)

    info_grid = [[[None, None, None, 0, 0, None] for i in range(dim)] for j in range(dim)]
    """ 
    info_grid keeps track of information from each cell:

    info_grid[row][col]: (blocked, nx, cx, bx, ex, hx)

    - blocked: boolean value, 0 unblocked, 1 blocked, None if unvisited
    - nx: number of neighbors
    - cx: number of neighbors sensed to be blocked
    - bx: number of neighbors confirmed to be blocked
    - ex: number of neighbors confirmed to be empty
    - hx: number of neighbors not confirmed to be blocked or empty 
    """ 
    astarresult = astar.astarwithstart(dim, gridworldcopy, start, heuristic)
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
        bx = 0
        ex = 0
        hx = 0

        # check if currnode is blocked
        if (gridworld[currNode[0]][currNode[1]] == 1): 
            blocked = 1
            gridworldcopy[row][col] = 1
        else: 
            blocked = 0
        nx = calculate_nx(currNode, dim)
        cx = calculate_cx(currNode, gridworld, dim)
        neighbor_list = generate_neighbor_list(currNode, dim)

        for neighbor in neighbor_list:
            info_grid[neighbor[0]][neighbor[1]][1] = calculate_nx(neighbor, dim)
            if info_grid[neighbor[0]][neighbor[1]][0] == 1:
                bx+=1
            elif info_grid[neighbor[0]][neighbor[1]][1] == 0:
                ex+=1
        
        hx = nx - bx - ex

        info_grid[currNode[0]][currNode[1]] = [blocked, nx, cx, bx, ex, hx]
        
        update_result = (info_grid, gridworldcopy)

        if (info_grid[currNode[0]][currNode[1]][0] is None):
            if blocked == 1:
                update_result = update_neighbors_bx(currNode, info_grid, gridworldcopy, dim)
            elif blocked == 0:
                update_result = update_neighbors_ex(currNode, info_grid, gridworldcopy, dim)

        info_grid = update_result[0]
        gridworldcopy = update_result[1] 

        """
        if blocked: 
            get new path with previous node as start, gridworldcopy as gridworld
            currNode is second node in new path 
        else: 
            add node to final path
            make the previous node the current nodes
            make curr node the next node in the path
        """
        #checking all nodes in the planned path to find any blocks
        replan_path = False
        for node in path:
            if gridworldcopy[node[0]][node[1]] == 1:
                replan_path = True
                break
           
        if (blocked == 1 or replan_path == True): 
            if (blocked == 1): 
                astarresult = astar.astarwithstart(dim, gridworldcopy, prevNode, heuristic)
                cells_processed += astarresult[0]
                path = astarresult[1]
            else: 
                # cell not blocked, but still need to replan
                astarresult = astar.astarwithstart(dim, gridworldcopy, currNode, heuristic)
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

def inference_agent(dim, gridworld, heuristic):
    start = (0,0)
    goal = (dim-1,dim-1)
    cells_processed = 0
    gridworldcopy = astar.generategridworld(dim, 0)

    info_grid = [[[None, None, None, 0, 0, None, None] for i in range(dim)] for j in range(dim)]
    """ 
    info_grid keeps track of information from each cell:

    info_grid[row][col]: (blocked, nx, cx, bx, ex, hx)

    - blocked: boolean value, 0 unblocked, 1 blocked, None if unvisited
    - nx: number of neighbors
    - cx: number of neighbors sensed to be blocked
    - bx: number of neighbors confirmed to be blocked
    - ex: number of neighbors confirmed to be empty
    - hx: number of neighbors not confirmed to be blocked or empty 
    - cbx: cx - bx 
    """ 
    astarresult = astar.astarwithstart(dim, gridworldcopy, start, heuristic)
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
        blocked = None
        nx = 0
        cx = 0
        bx = 0
        ex = 0
        hx = 0
        cbx= 0

        # check if currnode is blocked
        if (gridworld[currNode[0]][currNode[1]] == 1): 
            blocked = 1
            gridworldcopy[row][col] = 1
        else: 
            blocked = 0
        nx = calculate_nx(currNode, dim)
        cx = calculate_cx(currNode, gridworld, dim)
        neighbor_list = generate_neighbor_list(currNode, dim)

        for neighbor in neighbor_list:
            info_grid[neighbor[0]][neighbor[1]][1] = calculate_nx(neighbor, dim)
            if info_grid[neighbor[0]][neighbor[1]][0] == 1:
                bx+=1
            elif info_grid[neighbor[0]][neighbor[1]][1] == 0:
                ex+=1
        
        hx = nx - bx - ex
        cbx = cx-bx

        info_grid[currNode[0]][currNode[1]] = [blocked, nx, cx, bx, ex, hx, cbx]
        
        update_result = (info_grid, gridworldcopy)

        if (info_grid[currNode[0]][currNode[1]][0] is None):
            if blocked == 1:
                update_result = update_neighbors_bx(currNode, info_grid, gridworldcopy, dim)
            elif blocked == 0:
                update_result = update_neighbors_ex(currNode, info_grid, gridworldcopy, dim)

        info_grid = update_result[0]
        gridworldcopy = update_result[1] 

        """
        if blocked: 
            get new path with previous node as start, gridworldcopy as gridworld
            currNode is second node in new path 
        else: 
            add node to final path
            make the previous node the current nodes
            make curr node the next node in the path
        """
        #checking all nodes in the planned path to find any blocks

        one_two_one_result = one_two_one(currNode, info_grid, gridworldcopy, dim)
        found_mines = one_two_one_result[0]
        info_grid = one_two_one_result[1]
        gridworldcopy = one_two_one_result[2]


        one_two_two_one_result = one_two_two_one(currNode, info_grid, gridworldcopy, dim)
        found_mines_two_two = one_two_two_one_result[0]
        info_grid = one_two_two_one_result[1]
        gridworldcopy = one_two_two_one_result[2]

        replan_path = False
        for node in path:
            if gridworldcopy[node[0]][node[1]] == 1:
                replan_path = True
                break

        if (len(found_mines) > 0):
            replan_path = True

        if (len(found_mines_two_two) > 0): 
            replan_path = True
           
        if (blocked == 1 or replan_path == True): 
            if(blocked == 1):
                astarresult = astar.astarwithstart(dim, gridworldcopy, prevNode, heuristic)
                cells_processed += astarresult[0]
                path = astarresult[1]
            else: 
                # cell not blocked, but still need to replan
                astarresult = astar.astarwithstart(dim, gridworldcopy, currNode, heuristic)
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

def one_two_one(currNode, info_grid, gridworld_copy, dim): 
    # order neighbors_list: [north, northEast, east, southEast, south, southWest, west, northWest]
    found_mines = []
    neighbors_list = generate_neighbor_list_with_none(currNode, dim)

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

    if (isChildValidRepeatedForward(northWest, dim) and isChildValidRepeatedForward(north, dim) and isChildValidRepeatedForward(northEast, dim)):
        nw_cbx = info_grid[northWest[0]][northWest[1]][6]
        n_cbx = info_grid[north[0]][north[1]][6]
        ne_cbx = info_grid[northEast[0]][northEast[1]][6]
        if ((nw_cbx == 1) and (n_cbx == 2) and (ne_cbx == 1)):
            # if valid, blocks about nw and ne are blocked
            nw_block = (northWest[0]-1, northWest[1])
            ne_block = (northEast[0]-1, northEast[1])
            if (isChildValidRepeatedForward(nw_block, dim) and isChildValidRepeatedForward(ne_block, dim)):
                found_mines.append(nw_block)
                found_mines.append(ne_block)

                # add blocks to info grid and gridworld copy 
                info_grid[nw_block[0]][nw_block[1]][0] = 1
                info_grid[ne_block[0]][ne_block[1]][0] = 1
                gridworld_copy[nw_block[0]][nw_block[1]] = 1
                gridworld_copy[ne_block[0]][ne_block[1]] = 1
                    
                # update neighbors northwest block
                nw_block_changes = update_neighbors_bx(nw_block, info_grid, gridworld_copy, dim)
                info_grid = nw_block_changes[0]
                gridworld_copy = nw_block_changes[1]

                #update neighbors northeast block
                ne_block_changes = update_neighbors_bx(ne_block, info_grid, gridworld_copy, dim)
                info_grid = ne_block_changes[0]
                gridworld_copy = ne_block_changes[1]

    if (isChildValidRepeatedForward(northEast, dim) and isChildValidRepeatedForward(east, dim) and isChildValidRepeatedForward(southEast, dim)):
        ne_cbx = info_grid[northEast[0]][northEast[1]][6]
        e_cbx = info_grid[east[0]][east[1]][6]
        se_cbx = info_grid[southEast[0]][southEast[1]][6]
        if ((ne_cbx == 1) and (e_cbx == 2) and (se_cbx == 1)):
            # if valid, blocks east of ne and se are blocked
            ne_block = (northEast[0], northEast[1]+1)
            se_block = (southEast[0], southEast[1]+1)
            if (isChildValidRepeatedForward(ne_block, dim) and isChildValidRepeatedForward(se_block, dim)):
                found_mines.append(ne_block)
                found_mines.append(se_block)

                # add blocks to info grid and gridworld copy 
                info_grid[ne_block[0]][ne_block[1]][0] = 1
                info_grid[se_block[0]][se_block[1]][0] = 1
                gridworld_copy[ne_block[0]][ne_block[1]] = 1
                gridworld_copy[se_block[0]][se_block[1]] = 1
                    
                # update neighbors northwest block
                ne_block_changes = update_neighbors_bx(ne_block, info_grid, gridworld_copy, dim)
                info_grid = ne_block_changes[0]
                gridworld_copy = ne_block_changes[1]

                #update neighbors northeast block
                se_block_changes = update_neighbors_bx(se_block, info_grid, gridworld_copy, dim)
                info_grid = se_block_changes[0]
                gridworld_copy = se_block_changes[1]

    if (isChildValidRepeatedForward(southWest, dim) and isChildValidRepeatedForward(south, dim) and isChildValidRepeatedForward(southEast, dim)):
        sw_cbx = info_grid[southWest[0]][southWest[1]][6]
        s_cbx = info_grid[south[0]][south[1]][6]
        se_cbx = info_grid[southEast[0]][southEast[1]][6]
        if ((sw_cbx == 1) and (s_cbx == 2) and (se_cbx == 1)):
                # if valid, blocks south of sw and se are blocked
            sw_block = (southWest[0]+1, southWest[1])
            se_block = (southEast[0]+1, southEast[1])
            if (isChildValidRepeatedForward(sw_block, dim) and isChildValidRepeatedForward(se_block, dim)):
                found_mines.append(sw_block)
                found_mines.append(se_block)

                # add blocks to info grid and gridworld copy 
                info_grid[sw_block[0]][sw_block[1]][0] = 1
                info_grid[se_block[0]][se_block[1]][0] = 1
                gridworld_copy[sw_block[0]][sw_block[1]] = 1
                gridworld_copy[se_block[0]][se_block[1]] = 1
                    
                # update neighbors northwest block
                sw_block_changes = update_neighbors_bx(sw_block, info_grid, gridworld_copy, dim)
                info_grid = sw_block_changes[0]
                gridworld_copy = sw_block_changes[1]

                #update neighbors northeast block
                se_block_changes = update_neighbors_bx(se_block, info_grid, gridworld_copy, dim)
                info_grid = se_block_changes[0]
                gridworld_copy = se_block_changes[1]


    if (isChildValidRepeatedForward(northWest, dim) and isChildValidRepeatedForward(west, dim) and isChildValidRepeatedForward(southWest, dim)):
        nw_cbx = info_grid[northWest[0]][northWest[1]][6]
        w_cbx = info_grid[west[0]][west[1]][6]
        sw_cbx = info_grid[southWest[0]][southWest[1]][6]
        if ((nw_cbx == 1) and (w_cbx == 2) and (sw_cbx == 1)):
            #if valid, blocks west of nw and sw are blocked
            nw_block = (northWest[0], northWest[1]-1)
            sw_block = (southWest[0], southWest[1]-1)
            if (isChildValidRepeatedForward(nw_block, dim) and isChildValidRepeatedForward(sw_block, dim)):
                found_mines.append(nw_block)
                found_mines.append(sw_block)

                # add blocks to info grid and gridworld copy 
                info_grid[nw_block[0]][nw_block[1]][0] = 1
                info_grid[sw_block[0]][sw_block[1]][0] = 1
                gridworld_copy[nw_block[0]][nw_block[1]] = 1
                gridworld_copy[sw_block[0]][sw_block[1]] = 1
        
                # update neighbors northwest block
                nw_block_changes = update_neighbors_bx(nw_block, info_grid, gridworld_copy, dim)
                info_grid = nw_block_changes[0]
                gridworld_copy = nw_block_changes[1]

                #update neighbors northeast block
                sw_block_changes = update_neighbors_bx(sw_block, info_grid, gridworld_copy, dim)
                info_grid = sw_block_changes[0]
                gridworld_copy = sw_block_changes[1]


    return (found_mines, info_grid, gridworld_copy)


def one_two_two_one(curr_Node, info_grid, gridworldcopy, dim):
    # infer when current cell's cbx value = 2
    # order neighbors_list: [north, northEast, east, southEast, south, southWest, west, northWest]
    
    
    found_mines = []
    current_cbx = info_grid[curr_Node[0]][curr_Node[1]][6]
    if curr_Node[0]>=3 and curr_Node[0]<=dim-3 and curr_Node[1]>=4 and curr_Node[1]<=dim-3:
      if current_cbx >= 1:
        neighbors_list = generate_neighbor_list_with_none(curr_Node, dim)
        #print(curr_Node)
        #print(neighbors_list)

        north = neighbors_list[0]
        north2 = (north[0]-1, north[1])
        north3 = (north[0]-2, north[1])
        northEast = neighbors_list[1]
        northEast2 = (northEast[0], northEast[1]+1)
        north2East = (northEast[0]-1, northEast[1])
        north3East = (northEast[0]-2, northEast[1])
        
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
        north3West = (northWest[0]-2, northWest[1])
        if current_cbx == 1:
            if (isChildValidRepeatedForward(west3, dim) and isChildValidRepeatedForward(west2, dim) and isChildValidRepeatedForward(west, dim)): 
                www_cbx = info_grid[west3[0]][west3[1]][6]
                ww_cbx = info_grid[west2[0]][west2[1]][6]
                w_cbx = info_grid[west[0]][west[1]][6]
                
                if ((www_cbx == 1) and (ww_cbx == 2) and (w_cbx == 2)):
                    above_neighbors = [northWest3, northWest2, northWest, north]
                    below_neighbors = [southWest3, southWest2, southWest, south]
                    """
                    [ ][ ][ ][ ]
                    [1][2][2][x]
                    [ ][ ][ ][ ]
                    """
                    # if up neighbors not visited yet
                    if ifValidAndNone(above_neighbors, dim, info_grid):
                        # if valid, blocks about nw and nw2 are blocked
                        found_mines.append(northWest)
                        found_mines.append(northWest2)

                        nw_block_changes = update_neighbors_bx(northWest,info_grid, gridworldcopy, dim)
                        info_grid = nw_block_changes[0]
                        gridworld_copy = nw_block_changes[1]

                        nw2_block_changes = update_neighbors_bx(northWest2,info_grid,gridworldcopy,dim)
                        info_grid = nw2_block_changes[0]
                        gridworld_copy = nw2_block_changes[1]
                        
                    # if low neighbors not visted yet
                    elif ifValidAndNone(below_neighbors, dim, info_grid):
                        # if valid, blocks about sw and sw2 are blocked
                        found_mines.append(southWest)
                        found_mines.append(southWest2)

                        sw_block_changes = update_neighbors_bx(southWest,info_grid, gridworldcopy, dim)
                        info_grid = sw_block_changes[0]
                        gridworld_copy = sw_block_changes[1]

                        sw2_block_changes = update_neighbors_bx(southWest2,info_grid,gridworldcopy,dim)
                        info_grid = sw2_block_changes[0]
                        gridworld_copy = sw2_block_changes[1]

            if (isChildValidRepeatedForward(north3, dim) and isChildValidRepeatedForward(north2, dim) and isChildValidRepeatedForward(north, dim)): 
                nnn_cbx = info_grid[north3[0]][north3[1]][6]
                nn_cbx = info_grid[north2[0]][north2[1]][6]
                n_cbx = info_grid[north[0]][north[1]][6]
                if (nnn_cbx == 1 and nn_cbx == 2 and n_cbx == 2):
                    left_neighbors = [north3West, north2West, northWest, north]
                    right_neighbors = [north3East, north2East, northEast, east]
                    """
                    [][1][]
                    [][2][]
                    [][2][]
                    [][x][]
                    """
                    if ifValidAndNone(right_neighbors, dim, info_grid):
                        # if valid, blocks about ne and nne are blocked
                        found_mines.append(northEast) 
                        found_mines.append(north2East) 

                        ne_block_changes = update_neighbors_bx(northEast,info_grid,gridworldcopy,dim)
                        info_grid = ne_block_changes[0]
                        gridworld_copy = ne_block_changes[1]

                        n2e_block_changes = update_neighbors_bx(north2East,info_grid, gridworldcopy, dim)
                        info_grid = n2e_block_changes[0]
                        gridworld_copy = n2e_block_changes[1]
                    elif ifValidAndNone(left_neighbors, dim, info_grid):
                        found_mines.append(northWest) 
                        found_mines.append(north2West) 

                        nw_block_changes = update_neighbors_bx(northWest,info_grid,gridworldcopy,dim)
                        info_grid = nw_block_changes[0]
                        gridworld_copy = nw_block_changes[1]

                        n2w_block_changes = update_neighbors_bx(north2West,info_grid, gridworldcopy, dim)
                        info_grid = n2w_block_changes[0]
                        gridworld_copy = n2w_block_changes[1]

        if current_cbx == 2:
            # horizontal inference
            if (isChildValidRepeatedForward(west, dim) and isChildValidRepeatedForward(east, dim) and isChildValidRepeatedForward(east2, dim)):
                w_cbx = info_grid[west[0]][west[1]][6]
                e_cbx = info_grid[east[0]][east[1]][6]
                ee_cbx = info_grid[east2[0]][east2[1]][6]
                if ((w_cbx == 1) and (e_cbx == 2) and (ee_cbx == 1)):
                    above_neighbors = [northWest, north, northEast, northEast2]
                    below_neighbors = [southWest, south, southEast, southEast2]
                    """
                    [ ][ ][ ][ ]
                    [1][x][2][1]
                    [ ][ ][ ][ ]
                    """
                    if ifValidAndNone(above_neighbors, dim, info_grid):
                        # if valid, blocks about n and ne are blocked
                        found_mines.append(northEast)
                        found_mines.append(north)

                        ne_block_changes = update_neighbors_bx(northEast,info_grid, gridworldcopy, dim)
                        info_grid = ne_block_changes[0]
                        gridworld_copy = ne_block_changes[1]

                        n_block_changes = update_neighbors_bx(north,info_grid,gridworldcopy,dim)
                        info_grid = n_block_changes[0]
                        gridworld_copy = n_block_changes[1]

                    elif ifValidAndNone(below_neighbors, dim, info_grid):
                        # if valid, blocks about s and se are blocked
                        found_mines.append(southEast)
                        found_mines.append(south)

                        se_block_changes = update_neighbors_bx(southEast,info_grid, gridworldcopy, dim)
                        info_grid = se_block_changes[0]
                        gridworld_copy = se_block_changes[1]

                        s_block_changes = update_neighbors_bx(south,info_grid,gridworldcopy,dim)
                        info_grid = s_block_changes[0]
                        gridworld_copy = s_block_changes[1]

            
            if (isChildValidRepeatedForward(west2, dim) and isChildValidRepeatedForward(west, dim) and isChildValidRepeatedForward(east, dim)):
                ww_cbx = info_grid[west2[0]][west2[1]][6]
                w_cbx = info_grid[west[0]][west[1]][6]
                e_cbx = info_grid[east[0]][east[1]][6]
                if ((ww_cbx == 1) and (w_cbx == 2) and (e_cbx == 1)):
                    above_neighbors = [northWest2, northWest, north, northEast]
                    below_neighbors = [southWest2, southWest, south, southEast]
                    """
                    [ ][ ][ ][ ]
                    [1][2][x][1]
                    [ ][ ][ ][ ]
                    """
                    # if above neighbors not visited yet
                    if ifValidAndNone(above_neighbors, dim, info_grid):
                        # if valid, blocks about nw and n are blocked
                        found_mines.append(northWest)
                        found_mines.append(north)

                        nw_block_changes = update_neighbors_bx(northWest,info_grid, gridworldcopy, dim)
                        info_grid = nw_block_changes[0]
                        gridworld_copy = nw_block_changes[1]

                        n_block_changes = update_neighbors_bx(north,info_grid,gridworldcopy,dim)
                        info_grid = n_block_changes[0]
                        gridworld_copy = n_block_changes[1]

                    # if below neighbors not visted yet
                    elif ifValidAndNone(below_neighbors, dim, info_grid):
                        # if valid, blocks about sw and s are blocked
                        found_mines.append(southWest)
                        found_mines.append(south)

                        sw_block_changes = update_neighbors_bx(southWest,info_grid, gridworldcopy, dim)
                        info_grid = sw_block_changes[0]
                        gridworld_copy = sw_block_changes[1]

                        s_block_changes = update_neighbors_bx(south,info_grid,gridworldcopy,dim)
                        info_grid = s_block_changes[0]
                        gridworld_copy = s_block_changes[1]
                        
            
                
            # vertical inference
            if (isChildValidRepeatedForward(north, dim) and isChildValidRepeatedForward(south, dim) and isChildValidRepeatedForward(south2, dim)):
                n_cbx = info_grid[north[0]][north[1]][6]
                s_cbx = info_grid[south[0]][south[1]][6]
                ss_cbx = info_grid[south2[0]][south[1]][6]
                if ((n_cbx == 1) and (s_cbx == 2) and (ss_cbx == 1)):
                    """
                    [][1][]
                    [][x][]
                    [][2][]
                    [][1][]
                    """
                    right_neighbors = [northEast, east, southEast, south2East]
                    left_neighbors = [northWest, west, southWest, south2West]

                    # if right neighbors not visited yet
                    if ifValidAndNone(right_neighbors, dim, info_grid):
                        # if valid, blocks about e and se are blocked
                        found_mines.append(east) 
                        found_mines.append(southEast) 

                        e_block_changes = update_neighbors_bx(east,info_grid,gridworldcopy,dim)
                        info_grid = e_block_changes[0]
                        gridworld_copy = e_block_changes[1]

                        se_block_changes = update_neighbors_bx(southEast,info_grid, gridworldcopy, dim)
                        info_grid = se_block_changes[0]
                        gridworld_copy = se_block_changes[1]


                    # if low neighbors not visted yet
                    elif ifValidAndNone(left_neighbors, dim, info_grid):
                        # if valid, blocks about w and sw are blocked
                        found_mines.append(west) 
                        found_mines.append(southWest) 

                        w_block_changes = update_neighbors_bx(west,info_grid,gridworldcopy,dim)
                        info_grid = w_block_changes[0]
                        gridworld_copy = w_block_changes[1]

                        sw_block_changes = update_neighbors_bx(southWest,info_grid, gridworldcopy, dim)
                        info_grid = sw_block_changes[0]
                        gridworld_copy = sw_block_changes[1]
                        
            
            if (isChildValidRepeatedForward(north2, dim) and isChildValidRepeatedForward(north, dim) and isChildValidRepeatedForward(south, dim)):
                nn_cbx = info_grid[north2[0]][north2[1]][6]
                n_cbx = info_grid[north[0]][north[1]][6]
                s_cbx = info_grid[south[0]][south[1]][6]
                if ((nn_cbx == 1) and (n_cbx == 2) and (s_cbx == 1)):
                    """
                    [][1][]
                    [][2][]
                    [][x][]
                    [][1][]
                    """
                    right_neighbors = [north2East, northEast, east, southEast]
                    left_neighbors = [north2West, northWest, west, southWest]

                    # if right neighbors not visited yet
                    if ifValidAndNone(right_neighbors, dim, info_grid):
                        # if valid, blocks about ne and e are blocked
                        found_mines.append(northEast) 
                        found_mines.append(east) 
                        ne_block_changes = update_neighbors_bx(northEast,info_grid, gridworldcopy, dim)
                        info_grid = ne_block_changes[0]
                        gridworld_copy = ne_block_changes[1]

                        e_block_changes = update_neighbors_bx(east,info_grid,gridworldcopy,dim)
                        info_grid = e_block_changes[0]
                        gridworld_copy = e_block_changes[1]

                    # if low neighbors not visted yet
                    elif ifValidAndNone(left_neighbors, dim, info_grid):
                        # if valid, blocks about w and nw are blocked
                        found_mines.append(northWest) 
                        found_mines.append(west) 

                        nw_block_changes = update_neighbors_bx(northWest,info_grid, gridworldcopy, dim)
                        info_grid = nw_block_changes[0]
                        gridworld_copy = nw_block_changes[1]

                        w_block_changes = update_neighbors_bx(west,info_grid,gridworldcopy,dim)
                        info_grid = w_block_changes[0]
                        gridworld_copy = w_block_changes[1]
    
    return (found_mines, info_grid, gridworldcopy)

def ifValidAndNone(neighbor_list, dim, info_grid):
    for neighbor in neighbor_list: 
        if (info_grid[neighbor[0]][neighbor[1]][0] == 1 or info_grid[neighbor[0]][neighbor[1]][0] == 1 or not isChildValidRepeatedForward(neighbor,dim)): 
            return False
    return True

#neighbor_dict = {}
#neighbor_dict[(1,1)] = [1, 3, 1, 0, 0, 3]
#neighbor_info = neighbor_dict.get((1,1))
#neighbor_info[3] = 1
#print(neighbor_dict)
#print(neighbor_info)



#gridworld = astar.generategridworld(101, 0.20)
#astar.printAsGrid(gridworld)
#rfas_restricted_result = astar.repeatedforwardastar_restricted(101, gridworld, "manhattan")
#print("repeated forward astar restricted: " + str(rfas_restricted_result[0]))
#exampleresult = example_inference_agent(101, gridworld, "manhattan")
#print("example length: " + str(exampleresult[0]))

#inferenceresult = inference_agent(101,gridworld,"manhattan")
#inferenceresult = inferenceagent(len(gridworld), gridworld, manhattan)
#print("inference length: " + str(inferenceresult[0]))
#print("example path: " + str(exampleresult[3]))
'''
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

#astar.printAsGrid(gridworldTest)
#ret = generate_neighbors((0, 3), gridworldTest, 10)
#print(ret)

#dim = 3
#info_grid = [[(None, None, None, 0, 0, None) for i in range(dim)] for j in range(dim)] 
#astar.printAsGrid(info_grid)

#print(calculate_nx((9,0), 10))
'''
