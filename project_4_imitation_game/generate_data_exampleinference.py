import csv
import exampleinference
import pandas as pd

dims = [101]
trials = 30
for t in range(trials):
	for dim in dims: 
		filename = 'testing_data/example_inference/exampleinference_trial' + str(t) + '.csv'
		#pkl_filename = './exampleinference_data_' + str(dim) + '_trial' + str(t) + '.pkl'
		with open(filename, 'w') as f: 
			writer = csv.writer(f)
			writer.writerow(["local_grid", "next_move", "at_goal"])
			num_runs = 100
			i = 0
			while i < num_runs: 
				i += 1
				gridworld = exampleinference.generate_solvable_gridworld(dim, 0.25)
				# only run on solvable gridworlds
				exampleinference.exampleinferenceagent(dim, gridworld, "manhattan", writer)
	f.close()
	#df = pd.read_csv(filename)
	#df.to_pickle(pkl_filename)

