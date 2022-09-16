import sys
import time
import astar

#dim_list = {10, 25, 50, 75, 101}
dim_list = {101}
#p_list = {0.21, 0.22, 0.23, 0.24, 0.25, 0.26, 0.27}
#p_list = {0, 0.02, 0.04, 0.06, 0.08, 0.10, 0.12, 0.14, 0.16, 0.18, 0.20, 0.22, 0.24}
p_list = {0, 0.04, 0.08, 0.12, 0.16, 0.20, 0.24}
#p = 0.25
num_test = 10
#heuristic_list = {"manhattan", "chebyshev", "euclidean"}
heuristic_list = {"manhattan"}
astar_type = "repeated_restricted"

def run_test(dim, p, test_count, heuristic, astar_type):
    
    unsolvable = 0
    solvable = 0
    total_time_unsolvable = 0
    total_time_solvable = 0
    total_visited_unsolvable = 0
    total_visited_solvable = 0
    total_density = 0
    total_trajectory_length_unsolvable = 0
    total_trajectory_length_solvable = 0
    total_shortest_path_discovered = 0
    total_shortest_path_full = 0
    print("dim: " + str(dim) + "x" + str(dim))
    print("heuristic: " + heuristic)
    print("p = " + str(p) + ": ")
    total_time_start = time.time()
    for i in range(test_count):
        #print("test: " + str(i))
        gw = astar.generategridworld(dim, p)
        gw_density = astar.gw_density(dim, gw)
        total_density += gw_density
        start = time.time()
        # path = astar.astar(dim, gw, heuristic)
        if (astar_type == "repeated"):
            path = astar.repeatedforwardastar(dim, gw, heuristic)
        elif (astar_type == "repeated_restricted"):
            path = astar.repeatedforwardastar_restricted(dim, gw, heuristic)
        else:
            path = astar.astar(dim, gw, heuristic)
        end = time.time()
        #print(path)
        # if (path[0] == False):
        if (path[0] == -1 or path[0] == False):
            unsolvable += 1
            total_time_unsolvable += (end-start)
            total_visited_unsolvable += path[1]
            if (astar_type == "repeated" or astar_type == "repeated_restricted"):
                total_trajectory_length_unsolvable += len(path[3])
        else:
            total_time_solvable += (end-start)
            total_visited_solvable += path[1]
            if (astar_type == "repeated" or astar_type == "repeated_restricted"):
                total_trajectory_length_solvable += path[0]
                gridworldcopy = path[2]
                copypath = astar.astar(dim, gridworldcopy, heuristic)
                total_shortest_path_discovered += copypath[2]
                originalpath = astar.astar(dim, gw, heuristic)
                total_shortest_path_full += originalpath[2]
        
        
        #printPath(dim, path[2], path[3])
        
        #print("density: " + str(gw_density))
        #print("")
        # print("time for 5x5 gridworld, p = "+str(p)+ " : ")
        # print(end-start)
        # print('' )

    total_time_end = time.time()
    solvable = test_count-unsolvable
    print("A* type: "+astar_type)
    print("solvable/total: " + str(test_count-unsolvable) + "/" + str(test_count))

    #avoid dividing by 0
    if (solvable == 0):
        solvable = 1
    elif (unsolvable == 0):
        unsolvable = 1
    
    print("average time for solvable: " + str(total_time_solvable/solvable))
    print("average time for unsolvable: " + str(total_time_unsolvable/unsolvable))
    print('')
    print("average nodes visited for solvable: " + str(total_visited_solvable/solvable))
    print("average nodes visited for unsolvable: " + str(total_visited_unsolvable/unsolvable))
    print('')
    if (astar_type == "repeated" or astar_type == "repeated_restricted"):
        print("average trajectory length for solvable: " + str(total_trajectory_length_solvable/solvable))
        print("average trajectory length for unsolvable: " + str(total_trajectory_length_unsolvable/unsolvable))
        print('' )
        print("average shortest path in discovered gridworld: " + str(total_shortest_path_discovered/solvable))
        print("average shortest path in full gridworld: " + str(total_shortest_path_full/solvable))
        print('')
    print("average density: " + str(total_density/test_count))
    print("total time: "+str(total_time_end-total_time_start))
    print('' )
    
def printPath(dim, gridworld, path):
    gridworldcopy = [[0 for i in range(dim)] for j in range(dim)]
    for row in range(dim):
        for col in range(dim):
            if gridworld[row][col] == 1:
                gridworldcopy[row][col] = 1
            if (row,col) in path:
                gridworldcopy[row][col] = 8
    astar.printAsGrid(gridworldcopy)
        
    



for dim in dim_list:
    for heuristic in heuristic_list:
        for p in p_list:
            file_name = "tests" + str(dim) + "x" + str(dim) + "_p=" + str(p) +"_"+heuristic+"_"+astar_type+".txt"
            sys.stdout = open(file_name, "w")
            run_test(dim, p, num_test, heuristic, astar_type)
            sys.stdout.close()




