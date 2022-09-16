import sys
import astar
import time
import random


def generate_solvable_gridworld_positions(dim):
    gridworld = astar.generate_gridworld_with_terrain(dim, 0.3)
    start = (random.randint(0, dim - 1), random.randint(0, dim - 1))
    goal = (random.randint(0, dim - 1), random.randint(0, dim - 1))

    while gridworld[start[0]][start[1]] == -1:
        start = (random.randint(0, dim - 1), random.randint(0, dim - 1))
    while gridworld[goal[0]][goal[1]] == -1:
        goal = (random.randint(0, dim - 1), random.randint(0, dim - 1))

    solvable = astar.astarwithstartgoal(dim, gridworld, start, goal)[0]
    while not solvable or gridworld[start[0]][start[1]] == -1 or gridworld[goal[0]][goal[1]] == -1:
        start = (random.randint(0, dim - 1), random.randint(0, dim - 1))
        goal = (random.randint(0, dim - 1), random.randint(0, dim - 1))
        solvable = astar.astarwithstartgoal(dim, gridworld, start, goal)[0]
    return gridworld, start, goal


def test_function(dim):

    gridworld, start, goal = generate_solvable_gridworld_positions(dim)
    print gridworld
    """
    start_time6 = time.time()
    ret6 = astar.agent_six(dim, gridworld, start, goal)
    end_time6 = time.time()
    total_time6 = end_time6-start_time6

    start_time7 = time.time()
    ret7 = astar.agent_seven(dim, gridworld, start, goal)
    end_time7 = time.time()
    total_time7 = end_time7-start_time7

    start_time8 = time.time()
    ret8 = astar.agent_eight(dim, gridworld, start, goal)
    end_time8 = time.time()
    total_time8 = end_time8-start_time8
    """
    start_time9 = time.time()
    ret9 = astar.agent_nine(dim, gridworld, start, goal)
    end_time9 = time.time()
    total_time9 = end_time9-start_time9


    #return total_time6, ret6, total_time7, ret7, total_time8, ret8, total_time9, ret9, start, goal
    return total_time9, ret9, start, goal



def run_tests(dim, num_tests):
    """
    total_time_6 = 0
    total_moves_6 = 0
    total_cells_examined_6 = 0

    total_time_7 = 0
    total_moves_7 = 0
    total_cells_examined_7 = 0

    total_time_8 = 0
    total_moves_8 = 0
    total_cells_examined_8 = 0
    """
    total_time_9 = 0
    total_cells_examined_9 = 0

    total_dist = 0

    for i in range(num_tests):
        """
        time6, result6, time7, result7, time8, result8, time9, result9, start, goal = test_function(dim)
        print("test " + str(i) + ": ")
        print("time taken agent 6: " + str(time6))
        print("moves made agent 6: " + str(result6[1]))
        total_moves_6 += result6[1]
        print("cells examined agent 6: " + str(result6[2]))
        total_cells_examined_6 += result6[2]
        total_time_6 += time6
        print("")

        print("time taken agent 7: " + str(time7))
        print("moves made agent 7: " + str(result7[1]))
        total_moves_7 += result7[1]
        print("cells examined agent 7: " + str(result7[2]))
        total_cells_examined_7 += result7[2]
        total_time_7 += time7
        print("")

        print("time taken agent 8: " + str(time8))
        print("moves made agent 8: " + str(result8[1]))
        total_moves_8 += result8[1]
        print("cells examined agent 8: " + str(result8[2]))
        total_cells_examined_8 += result8[2]
        total_time_8 += time8
        print("")
        """

        time9, result9, start, goal = test_function(dim)

        print("time taken agent 9: " + str(time9))
        print("cells examined agent 9: " + str(result9[1]))
        total_cells_examined_9 += result9[1]
        total_time_9 += time9
        print("")

        start = start
        goal = goal
        dist = astar.manhattan(start[0], goal[0], start[1], goal[1])
        total_dist += dist
        print("distance from start to goal: " + str(dist))
        print("")
    print("")
    """
    average_time_6 = total_time_6/num_tests
    average_moves_6 = total_moves_6/num_tests
    average_cells_examined_6 = total_cells_examined_6/num_tests

    average_time_7 = total_time_7/num_tests
    average_moves_7 = total_moves_7/num_tests
    average_cells_examined_7 = total_cells_examined_7/num_tests

    average_time_8 = total_time_8 / num_tests
    average_moves_8 = total_moves_8 / num_tests
    average_cells_examined_8 = total_cells_examined_8 / num_tests
    """
    average_time_9 = total_time_9 / num_tests
    average_cells_examined_9 = total_cells_examined_9 / num_tests

    average_dist = total_dist/num_tests
    """
    print("average time agent 6:", average_time_6)
    print("average moves agent 6:", average_moves_6)
    print("average cells examined agent 6", average_cells_examined_6)

    print("average time agent 7:", average_time_7)
    print("average moves agent 7:", average_moves_7)
    print("average cells examined agent 7:", average_cells_examined_7)

    print("average time agent 8:", average_time_8)
    print("average moves agent 8:", average_moves_8)
    print("average cells examined agent 8:", average_cells_examined_8)
    """
    print("average time agent 9:", average_time_9)
    print("average cells examined agent 9:", average_cells_examined_9)
    print("average distance:", average_dist)


def main():
    dim = 100
    num_tests = 1
    #file_name = "tests" + str(dim) + "x" + str(dim) + "_agent_six_seven.txt"
    #sys.stdout = open(file_name, "w")
    run_tests(dim, num_tests)
    #sys.stdout.close()




if __name__ == '__main__':
    main()


