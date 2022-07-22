import json
import os

import numpy as np
import matplotlib.pyplot as plt

from solver.random.random import random_solution
from utils.utils import build_output, build_solution_file_for_qgis, euclidean_distance_matrix, request_cleaner
from metrics.visual_attractiveness.route_complexity import bending_energy_metric, crossings_metric, \
    long_distance_index_metric_using_road_network, long_distance_index_metric_using_euclidean_distance
from metrics.traditional.driving_time import driving_time_using_road_network, \
    driving_time_using_euclidean_distance
from solver.heuristic.ORTools.ORTools import ORTools_run
from solver.heuristic.simulated_annealing.anneal import SimAnneal
from solver.heuristic.two_opt.two_opt import TwoOpt
from solver.exacts.tsp_model import create_and_solve_the_model

if __name__ == '__main__':
    path = "instances/jadlog_OR/"
    requests = os.listdir(path)

    for request in requests:
        with open(path + request, 'r') as wor_file:
            optimization_request = json.load(wor_file)

        data = request_cleaner(optimization_request)

        output = []
        name = request.split('_')[0]

        for seed in range(0, 10):

            print("\n")

            if seed < 7:
                r_solution = random_solution(data['noNodes'], seed)
                if seed < 5:
                    print(" >> Simulated Annealing")
                    alg = SimAnneal(data, r_solution, stopping_iter=5000)
                else:
                    print(" >> Two Opt")
                    alg = TwoOpt(data, r_solution, stopping_iter=2)
            elif 7 <= seed < 9:
                r_solution = ORTools_run(data)
                if seed == 7:
                    print(" >> Simulated Annealing")
                    alg = SimAnneal(data, r_solution, stopping_iter=5000)
                else:
                    print(" >> Two Opt")
                    alg = TwoOpt(data, r_solution, stopping_iter=2)
            else:
                print(" >> Two Opt")
                r_solution = create_and_solve_the_model(data)
                alg = TwoOpt(data, r_solution, stopping_iter=2)

            alg.anneal()
            alg.visualize_routes()
            # alg.plot_learning()

            solution = alg.best_solution

            build_solution_file_for_qgis(solution, data, name)

            output.append(seed)

            for point in solution:
                output.append(point)

            # Traditional metrics
            dt_using_rn = driving_time_using_road_network(solution, data)
            print("Driving time using road network: " + str(dt_using_rn))
            output.append(dt_using_rn)

            dt_using_ed = driving_time_using_euclidean_distance(solution, data)
            print("Driving time using euclidean distance: " + str(dt_using_ed))
            output.append(dt_using_ed)

            # Route simplicity metrics
            bending_energy = bending_energy_metric(solution, data)
            print("Total bending energy: " + str(bending_energy))
            output.append(bending_energy)

            number_of_crossings = crossings_metric(solution, data)
            print("Total crossings: " + str(number_of_crossings))
            output.append(number_of_crossings)

            total_ldi_using_road_network = long_distance_index_metric_using_road_network(solution, data)
            print("Total long distance index using road network: " + str(total_ldi_using_road_network))
            output.append(total_ldi_using_road_network)

            total_ldi_using_euclidean_distance = long_distance_index_metric_using_euclidean_distance(solution, data)
            print("Total long distance index using euclidean distance: " + str(total_ldi_using_euclidean_distance))
            output.append(total_ldi_using_euclidean_distance)

            build_output(output, name)

            output.clear()
