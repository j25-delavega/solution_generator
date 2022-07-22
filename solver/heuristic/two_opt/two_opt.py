import math
import random

from metrics.traditional.driving_time import driving_time
from .visualize_tsp.visualize_tsp import plotTSP
import matplotlib.pyplot as plt
from metrics.visual_attractiveness.crossings import crossings
from metrics.visual_attractiveness.bending_energy import bending_energy
from metrics.visual_attractiveness.long_distance_index import long_distance_index


class TwoOpt(object):
    def __init__(self, data, initial_solution, T=-1, alpha=-1, stopping_T=-1, stopping_iter=-1):
        self.current_fitness = None
        self.current_solution = None
        self.data = data
        self.N = len(self.data['coords'])
        self.T = math.sqrt(self.N) if T == -1 else T
        self.T_save = self.T  # save inital T to reset if batch annealing is used
        self.alpha = 0.995 if alpha == -1 else alpha
        self.stopping_temperature = 1e-8 if stopping_T == -1 else stopping_T
        self.stopping_iter = 100000 if stopping_iter == -1 else stopping_iter
        self.iteration = 1
        self.nodes = [i for i in range(self.N)]
        self.best_solution = None
        self.best_fitness = float("Inf")
        self.fitness_list = []
        self.warm_solution = initial_solution

    def initial_solution(self):
        """
        Greedy algorithm to get an initial solution (closest-neighbour).
        """
        # cur_node = random.choice(self.nodes)  # start from a random node
        cur_node = 0  # start from depot node
        solution = [cur_node]

        free_nodes = set(self.nodes)
        free_nodes.remove(cur_node)
        while free_nodes:
            next_node = min(free_nodes, key=lambda x: self.dist(cur_node, x))  # nearest neighbour
            free_nodes.remove(next_node)
            solution.append(next_node)
            cur_node = next_node

        cur_fit = self.fitness(solution)
        if cur_fit < self.best_fitness:  # If best found so far, update best fitness
            self.best_fitness = cur_fit
            self.best_solution = solution
        self.fitness_list.append(cur_fit)
        return solution, cur_fit

    def dist(self, node_0, node_1):
        """
        Euclidean distance between two nodes.
        """
        coord_0, coord_1 = self.data['coords'][node_0], self.data['coords'][node_1]
        return math.sqrt((coord_0[0] - coord_1[0]) ** 2 + (coord_0[1] - coord_1[1]) ** 2)

    def fitness(self, solution):
        """
        Total distance of the current solution path.
        """
        distances = self.data['euclidean_distances']
        return driving_time(solution, distances)

    def p_accept(self, candidate_fitness):
        """
        Probability of accepting if the candidate is worse than current.
        Depends on the current temperature and difference between candidate and current.
        """
        return math.exp(-abs(candidate_fitness - self.current_fitness) / self.T)

    def accept(self, candidate):
        """
        Accept with probability 1 if candidate is better than current.
        Accept with probabilty p_accept(..) if candidate is worse.
        """
        candidate_fitness = self.fitness(candidate)
        if candidate_fitness < self.current_fitness:
            self.current_fitness, self.current_solution = candidate_fitness, candidate
            if candidate_fitness < self.best_fitness:
                self.best_fitness, self.best_solution = candidate_fitness, candidate
        else:
            if random.random() < self.p_accept(candidate_fitness):
                self.current_fitness, self.current_solution = candidate_fitness, candidate

    def anneal(self):
        """
        Execute simulated annealing algorithm.
        """
        # Initialize with the greedy solution.
        # print(self.warm_solution)
        # self.cur_solution, self.cur_fitness = self.initial_solution()

        self.current_solution = self.warm_solution
        self.current_fitness = self.fitness(self.current_solution)
        self.fitness_list.append(self.current_fitness)
        self.best_solution = self.current_solution
        self.best_fitness = self.current_fitness
        print(self.current_solution)
        print("Initial fitness value: ", self.current_fitness)

        '''self.cur_solution, self.cur_fitness = self.initial_solution()
        print(self.cur_solution)
        print("Initial fitness value: ", self.cur_fitness)'''

        print("Starting annealing.")
        while self.T >= self.stopping_temperature and self.iteration < self.stopping_iter:
            # print("iteration: " + str(self.iteration))
            candidate = list(self.current_solution)
            new_candidate = self.two_opt(candidate)
            self.accept(new_candidate)
            self.T *= self.alpha
            self.iteration += 1

            self.fitness_list.append(self.current_fitness)

        print("Best fitness obtained: ", self.best_fitness)
        improvement = 100 * (self.fitness_list[0] - self.best_fitness) / (self.fitness_list[0])
        print(f"Improvement over greedy heuristic: {improvement : .2f}%")
        print(self.best_solution)

    def batch_anneal(self, times=10):
        """
        Execute simulated annealing algorithm `times` times, with random initial solutions.
        """
        for i in range(1, times + 1):
            print(f"Iteration {i}/{times} -------------------------------")
            self.T = self.T_save
            self.iteration = 1
            self.current_solution, self.current_fitness = self.initial_solution()
            self.anneal()

    def visualize_routes(self):
        """
        Visualize the TSP route with matplotlib.
        """
        plotTSP([self.best_solution], self.data['coords'])

    def plot_learning(self):
        """
        Plot the fitness through iterations.
        """
        plt.plot([i for i in range(len(self.fitness_list))], self.fitness_list)
        plt.ylabel("Fitness")
        plt.xlabel("Iteration")
        plt.show()

    def two_opt(self, route):
        best = route
        improved = True
        while improved:
            improved = False
            for i in range(1, len(route) - 2):
                for j in range(i + 1, len(route)):
                    if j - i == 1: continue  # changes nothing, skip then
                    new_route = route[:]
                    new_route[i:j] = route[j - 1:i - 1:-1]  # this is the 2woptSwap
                    if self.fitness(new_route) < self.fitness(best):
                        best = new_route
                        # print("best fitness: " + str(self.fitness(best)))
                        improved = True
            route = best
        return best

    def two_opt_modified(self, route):
        best = route
        best_fitness = self.fitness(best)
        for i in range(1, len(route) - 2):
            for j in range(i + 1, len(route)):
                if j - i == 1: continue  # changes nothing, skip then
                new_route = route[:]
                new_route[i:j] = route[j - 1:i - 1:-1]  # this is the 2woptSwap
                new_route_fitness = self.fitness(new_route)
                if new_route_fitness < best_fitness:
                    best = new_route
                    best_fitness = new_route_fitness
                    print("best fitness: " + str(best_fitness))
        return best
