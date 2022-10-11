import json
import os
from timeit import default_timer as timer

# import numpy as np
# import oapackage

from solver.random.random import random_solution
from utils.utils import build_output, optimization_request_cleaner, build_solution_file_for_qgis_using_the_response
from solver.heuristic.ORTools.ORTools import ORTools_run
from solver.heuristic.simulated_annealing.anneal import SimAnneal
from solver.heuristic.two_opt.two_opt import TwoOpt
from solver.exacts.tsp_model import create_and_solve_the_model
from output.output_format import output_file

if __name__ == '__main__':

    # build_solution_file_for_qgis_using_the_response()


    path = "instances/jadlog_ORs/"
    requests = os.listdir(path)

    requests_aux = [requests[len(requests) - 1]]

    for request in requests_aux:
        with open(path + request, 'r') as wor_file:
            instance = json.load(wor_file)

        data = optimization_request_cleaner(instance)

        name = request.split('_')[0]

        for seed in range(0, 1):
            start = timer()
            r_solution = random_solution(data['noNodes'], seed)
            sa_alg = SimAnneal(data, r_solution, seed, -1, -1, -1, stopping_iter=50*len(r_solution))
            sa_alg.perform()
            sa_alg.visualize_routes()
            end = timer()
            build_output(output_file(seed, sa_alg.best_solution, data), name)
            print(" Running time: " + str(end - start))

            # # two_opt_alg = TwoOpt(data, r_solution, seed, name, stopping_iter=2)
            # # two_opt_alg.visualize_routes(r_solution)
            # two_opt_alg.visualize_routes(sa_alg.best_solution)
            # # two_opt_alg.perform()
            # two_opt_alg.visualize_routes()
            # # end = timer()
            # # build_output(output_file(seed, two_opt_alg.best_solution, data), name)
            # # print(" Running time: " + str(end - start))
