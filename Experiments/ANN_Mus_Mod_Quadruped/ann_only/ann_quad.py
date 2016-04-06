"""
    Evolve a quadruped robot that contains both an ANN and a Muscle Model Controller.  This one has one signal from
    the ANN to each Muscle Group.
"""

import argparse
import sys, os, random, time
import itertools
import math
import multiprocessing as mpc

from quad_simulation import Simulation

import MultiNEAT as NEAT
from Controllers import MultiNEAT_logging as mnlog

def evaluate_individual(individual):
    """ Wrapper to call Simulation which will evaluate an individual.  

    Args:
        individual: arguments to pass to the simulation

    Returns:
        fitness of an individual
    """

    simulation = Simulation(log_frames=args.log_frames, run_num=args.run_num, eval_time=args.eval_time, dt=.02, n=4)
    return simulation.evaluate_individual(individual)

def evolutionary_run(**kwargs):
    """ Conduct an evolutionary run.  
    
    Args:
        gens: generations of evolution
        pop_size: population size
        mut_prob: mutation probability
    """
    global args, current_network, run_num, output_path, population_size, simulation

    params = NEAT.Parameters()  
    params.CompatTreshold = 5.0
    params.CompatTresholdModifier = 0.3
    params.YoungAgeTreshold = 15
    params.SpeciesMaxStagnation = 50
    params.OldAgeTreshold = 35
    params.MinSpecies = 1 
    params.MaxSpecies = 25
    params.RouletteWheelSelection = False
    params.RecurrentProb = 0.25
    params.OverallMutationRate = 0.33
    params.MutateWeightsProb = 0.90
    params.WeightMutationMaxPower = 1.0
    params.WeightReplacementMaxPower = 5.0
    params.MutateWeightsSevereProb = 0.5
    params.WeightMutationRate = 0.75
    params.MaxWeight = 20

    params.MutateAddNeuronProb = 0.04
    params.MutateAddLinkProb = 0.1
    params.MutateRemSimpleNeuronProb = 0.04
    params.MutateRemLinkProb = 0.1

    # Phased Searching
    # params.PhasedSearching = True;
    # params.SimplifyingPhaseMPCTreshold = 20;
    # params.SimplifyingPhaseStagnationTreshold = 20;
    # params.ComplexityFloorGenerations = 20;

    params.PopulationSize = kwargs['pop_size'] 

    params.Save(output_path+str(run_num)+"_NEAT_params.cfg")
   
    # Initialize the populations
    genome = NEAT.Genome(0, 22, 0, 16, False, NEAT.ActivationFunction.SIGNED_SIGMOID, NEAT.ActivationFunction.SIGNED_SIGMOID, 0, params)
    # If not including a periodic input.
    if args.no_periodic:
        genome = NEAT.Genome(0, 21, 0, 16, False, NEAT.ActivationFunction.SIGNED_SIGMOID, NEAT.ActivationFunction.SIGNED_SIGMOID, 0, params)
    population = NEAT.Population(genome, params, True, 1.0)
    genome_list = NEAT.GetGenomeList(population)

    mnlog.write_population_statistics_headers(output_path+str(run_num)+"_fitnesses.dat",optional_additions="Num_Neurons,Num_Connections")

    # Setup multiprocessing
    cores = mpc.cpu_count()
    pool = mpc.Pool(processes=cores-2)

    for gen in xrange(kwargs['gens']):
        ind_descriptor = pool.map(evaluate_individual,genome_list) # 0 - fit, 1 - # neurons, 2 - # connections
        fitnesses = []
        num_conns = []
        num_neurons = []
        for g,ind in zip(genome_list,ind_descriptor):
            g.SetFitness(ind[0])
            fitnesses.append(ind[0])
            num_neurons.append(ind[1])
            num_conns.append(ind[2])
        print("Generation "+str(gen)+"\t: "+str(max(fitnesses)))

        # Write the best performing individual to a file.
        mnlog.write_best_individual(output_path+"best_individuals/Evo_NEAT_Mus_run_"+str(run_num)+"_best_gen_"+str(gen)+".dat", 
                genome_list[fitnesses.index(max(fitnesses))])
    
        # Log the progress of the entire population.
        mnlog.write_population_statistics(output_path+str(run_num)+"_fitnesses.dat", genome_list, fitnesses, gen, optional_additions=zip(num_neurons,num_conns))

        # Log the final population for later evaluation.
        if gen == kwargs['gens'] - 1:
            population.Save(output_path+"run_"+str(run_num)+"_population_generation_"+str(gen)+".dat")

        # Create the next generation
        population.Epoch()

        # Get the genomes
        genome_list = NEAT.GetGenomeList(population)

######################################################################

# Process inputs.
parser = argparse.ArgumentParser()
parser.add_argument("--validator", action="store_true", help="Validate current results.")
parser.add_argument("--gens", type=int, default=100, help="Number of generations to run evolution for.")
parser.add_argument("--pop_size", type=int, default=100, help="Population size for evolution.")
parser.add_argument("--mut_prob", type=float, default=0.05, help="Mutation probability for evolution.")
parser.add_argument("--eval_time", type=float, default=10., help="Simulation time for an individual.")
parser.add_argument("--run_num", type=int, default=0, help="Run Number")
parser.add_argument("--output_path", type=str, default="./", help="Output path")
parser.add_argument("--log_run",action="store_true", help="Load data from log file and run.")
parser.add_argument("--log_frames",action="store_true",help="Save the frames to a folder.")
parser.add_argument("--val_num",type=int,default=-1,help="Number to identify the validator run.")
parser.add_argument("--debug_runtime",action="store_true",help="Evaluate the run time of a simulation.")
parser.add_argument("--config_file",type=str,default="../../config_files/default.cfg")
parser.add_argument("--config_string",type=str,default="",help="Use a string instead of file for configuration.")
parser.add_argument("--no_periodic",action="store_true",help="Whether we're including a periodic signal or not.")
args = parser.parse_args()

running = True
eval_time = args.eval_time 

output_path = args.output_path
run_num = args.run_num

# Seed only the evolutionary runs.
random.seed(run_num)

if args.debug_runtime:
    # Initialize the Simulation component
    simulation = Simulation(log_frames=args.log_frames, run_num=args.run_num, eval_time=args.eval_time, dt=.02, n=4)

    simulation.debug_validator()
elif args.validator:
    # Initialize the Simulation component
    simulation = Simulation(log_frames=args.log_frames, run_num=args.run_num, eval_time=args.eval_time, dt=.02, n=4)

    # Ensure that the file exists before trying to open to avoid hanging.
    NEAT_file = output_path+"best_individuals/Evo_NEAT_Mus_run_"+str(args.run_num)+"_best_gen_"+str(args.gens)+".dat"
    if not os.path.isfile(NEAT_file):
        print("NEAT Genome file doesn't exist! "+NEAT_file)
        exit()

    simulation.validator(NEAT_file)
else:
    # Ensure that the necessary folders are created for recording data.
    if not os.path.exists(output_path+"best_individuals"):
        os.makedirs(output_path+"best_individuals")
    evolutionary_run(gens=args.gens,pop_size=args.pop_size,mut_prob=args.mut_prob)