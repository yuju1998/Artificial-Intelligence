import sys
import time
import astar
import pprint
import inferenceagents

dim_list = {101}
dim = 101
p_list = {0, 0.05, 0.1, 0.15, 0.2, 0.23, 0.25, 0.28, 0.3, 0.33}
num_test = 100
#heuristic_list = {"manhattan", "chebyshev", "euclidean"}
heuristic = "manhattan"
pp = pprint.PrettyPrinter(indent=4)

def run_test(dim, p_list, test_count, heuristic):
    astar_dictionary = {}
    """
    {
        0: {
            blindfolded_average: 
            blindfolded_solvable: 
            ...
        },
        0.05: {
            blindfolded_average: 
            blindfolded_solvable: 
            ...
        },
        ...
    }
    """

    for p in p_list: 
        blindfolded_solvable = 0
        blindfolded_unsolvable = 0
        four_neighbor_solvable = 0
        four_neighbor_unsolvable = 0
        example_inference_solvable = 0
        example_inference_unsolvable = 0
        inference_solvable = 0
        inference_unsolvable = 0

        blindfolded_total_trajectory = 0
        four_neighbor_total_trajectory = 0
        example_inference_total_trajectory = 0
        inference_total_trajectory = 0
        blindfolded_cells_processed = 0
        four_neighbor_cells_processed = 0
        example_inference_cells_processed = 0
        inference_cells_processed = 0
        for i in range(test_count):        
            #print("test: " + str(i))
            gridworld = astar.generategridworld(dim, p)
            #gw_density = astar.gw_density(dim, gw)
            #total_density += gw_density
            
            blindfolded_path = astar.repeatedforwardastar_restricted(dim, gridworld, heuristic)
            four_neighbor_path = astar.repeatedforwardastar(dim, gridworld, heuristic)
            example_inference_path = inferenceagents.example_inference_agent(dim, gridworld, heuristic)
            inference_path = inferenceagents.inference_agent(dim, gridworld, heuristic)

            # BLINDFOLDED
            if (blindfolded_path[0] == -1 or blindfolded_path[0] == False): 
                blindfolded_unsolvable += 1
            else: 
                blindfolded_solvable += 1 
                blindfolded_total_trajectory += blindfolded_path[0]
                blindfolded_cells_processed += blindfolded_path[1]

            # FOUR NEIGHBOR  
            if (four_neighbor_path[0] == -1 or four_neighbor_path[0] == False):
                four_neighbor_unsolvable += 1
            else: 
                four_neighbor_solvable += 1
                four_neighbor_total_trajectory += four_neighbor_path[0]
                four_neighbor_cells_processed += four_neighbor_path[1]

            # EXAMPLE INFERENCE
            if(example_inference_path[0] == -1 or example_inference_path[0] == False):
                example_inference_unsolvable += 1
            else: 
                example_inference_solvable += 1
                example_inference_total_trajectory += example_inference_path[0]
                example_inference_cells_processed += example_inference_path[1]

            # INFERENCE
            if(inference_path[0] == -1 or inference_path[0] == False):
                inference_unsolvable += 1
            else: 
                inference_solvable += 1
                inference_total_trajectory += inference_path[0]
                inference_cells_processed += inference_path[1]

        blindfolded_avg_trajectory = blindfolded_total_trajectory / blindfolded_solvable
        four_neighbor_avg_trajectory = four_neighbor_total_trajectory / four_neighbor_solvable
        example_inference_avg_trajectory = example_inference_total_trajectory / example_inference_solvable
        inference_avg_trajectory = inference_total_trajectory / inference_solvable

        blindfolded_avg_cp = blindfolded_cells_processed / blindfolded_solvable
        four_neighbor_avg_cp = four_neighbor_cells_processed / four_neighbor_solvable
        example_inference_avg_cp = example_inference_cells_processed / example_inference_solvable
        inference_avg_cp = inference_cells_processed / inference_solvable


        astar_dictionary[p] = {
            "0_solvable": inference_solvable,
            "1_blindfolded avg trajectory": blindfolded_avg_trajectory,
            "2_four neighbor avg trajectory": four_neighbor_avg_trajectory,
            "3_example inference avg trajectory": example_inference_avg_trajectory,
            "4_inference avg trajectory": inference_avg_trajectory,
            "5_blindfolded avg cells processed": blindfolded_avg_cp,
            "6_four neighber avg cells processed": four_neighbor_avg_cp,
            "7_example inference avg cells processed": example_inference_avg_cp,
            "8_inference avg cells processed": inference_avg_cp

        }

    return astar_dictionary


astar_dictionary = run_test(dim,p_list,num_test,"manhattan")
pp.pprint(astar_dictionary)