import math
import random

from metrics.traditional.driving_time import driving_time
from .visualize_tsp.visualize_tsp import plot_TSP
import matplotlib.pyplot as plt
from metrics.visual_attractiveness.crossings import crossings
from metrics.visual_attractiveness.bending_energy import bending_energy
from metrics.visual_attractiveness.long_distance_index import long_distance_index

random.seed(1)


class SimAnneal(object):
    def __init__(self, data, initial_solution, seed, T=-1, alpha=-1, stopping_T=-1, stopping_iter=-1):
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
        self.seed = seed
        self.number_of_intra_local_searches = 3
        self.weights = [1] * self.number_of_intra_local_searches
        self.success = [0.0] * self.number_of_intra_local_searches
        self.iterations_to_update = 100
        self.ro = 0.35
        self.first_delta = 135
        self.second_delta = 70
        self.third_delta = 25
        self.n_used = [0] * self.number_of_intra_local_searches

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
        return driving_time(solution, distances, False)

    def p_accept(self, candidate_fitness):
        """
        Probability of accepting if the candidate is worse than current.
        Depends on the current temperature and difference between candidate and current.
        """
        return math.exp(-abs(candidate_fitness - self.current_fitness) / self.T)

    def accept(self, candidate, ls_id):
        """
        Accept with probability 1 if candidate is better than current.
        Accept with probabilty p_accept(..) if candidate is worse.
        """
        candidate_fitness = self.fitness(candidate)
        if candidate_fitness < self.current_fitness:
            self.current_fitness, self.current_solution = candidate_fitness, candidate
            self.success[ls_id] += self.second_delta
            if candidate_fitness < self.best_fitness:
                self.best_fitness, self.best_solution = candidate_fitness, candidate
                self.success[ls_id] += (self.first_delta - self.second_delta)
                print(" Iteration: " + str(self.iteration) + " Cost: " + str(self.best_fitness))
        else:
            if random.random() < self.p_accept(candidate_fitness):
                self.current_fitness, self.current_solution = candidate_fitness, candidate
                self.success[ls_id] += self.third_delta

    def local_search_selector(self):
        total_weights = 0
        for weigh in self.weights:
            total_weights += weigh

        accumulated_probability = [0.0] * self.number_of_intra_local_searches
        accumulated_probability[0] = self.weights[0] / total_weights

        for i in range(1, self.number_of_intra_local_searches):
            accumulated_probability[i] = accumulated_probability[i - 1] + (self.weights[i] / total_weights)

        random_number = random.uniform(0, 1)
        count = 0
        while random_number > accumulated_probability[count]:
            count += 1
        return count

    def weight_updater(self):
        for ls in range(0, self.number_of_intra_local_searches):
            self.weights[ls] *= (1 - self.ro)
            if self.n_used[ls] > 0:
                self.weights[ls] += self.ro * (self.success[ls] / self.n_used[ls])

    def perform(self):
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
            for ls in range(0, self.number_of_intra_local_searches):
                self.success[ls] = 0.0
                self.n_used[ls] = 0

            for iter_aux in range(0, self.iterations_to_update):
                current_solution = list(self.current_solution)

                ls_id = self.local_search_selector()
                self.n_used[ls_id] += 1

                i = random.randint(1, self.N - 2)
                if ls_id == 0:
                    # print(" >> Please, apply the Two Opt local search >>")
                    candidate = self.two_opt_perform(current_solution, i)
                elif ls_id == 1:
                    # print(" >> Please, apply the Exchange local search >>")
                    candidate = self.exchange_perform(current_solution, i)
                else:
                    # print(" >> Please, apply the Reallocate local search >>")
                    candidate = self.reallocate_perform(current_solution, i)
                self.accept(candidate, ls_id)
                self.T *= self.alpha
                self.iteration += 1
                self.fitness_list.append(self.current_fitness)
            self.weight_updater()
        print("Best fitness obtained: ", self.best_fitness)
        improvement = 100 * (self.fitness_list[0] - self.best_fitness) / (self.fitness_list[0])
        print(f"Improvement over greedy heuristic: {improvement : .2f}%")

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
        plot_TSP(self.best_solution, self.data, self.seed)

    def plot_learning(self):
        """
        Plot the fitness through iterations.
        """
        plt.plot([i for i in range(len(self.fitness_list))], self.fitness_list)
        plt.ylabel("Fitness")
        plt.xlabel("Iteration")
        plt.show()

    def two_opt_perform(self, current_solution, i):
        best_solution = current_solution
        for j in range(i + 1, len(current_solution)):
            if j - i == 1: continue  # changes nothing, skip then
            new_route = current_solution[:]
            new_route[i:j] = current_solution[j - 1:i - 1:-1]
            if self.fitness(new_route) < self.fitness(best_solution):
                best_solution = new_route
        return best_solution

    def exchange_perform(self, current_solution, i):
        best_solution = current_solution
        for j in range(1, len(current_solution)):
            if j == i: continue  # changes nothing, skip then
            new_route = current_solution[:]
            new_route[i] = current_solution[j]
            new_route[j] = current_solution[i]
            if self.fitness(new_route) < self.fitness(best_solution):
                best_solution = new_route
        return best_solution

    def new_route_in_reallocate_when_i_less_than_j(self, current_solution, i, j):
        new_route = current_solution[:]
        new_route[i:j] = current_solution[i + 1:j + 1]
        new_route[j] = current_solution[i]
        return new_route

    def new_route_in_reallocate_when_j_less_than_i(self, current_solution, i, j):
        new_route = current_solution[:]
        new_route[j] = current_solution[i]
        new_route[j + 1:i + 1] = current_solution[j:i]
        return new_route

    def reallocate_perform(self, current_solution, i):
        best_solution = current_solution
        for j in range(i + 1, len(current_solution)):
            if j == i: continue  # changes nothing, skip then

            if i < j:
                new_route = self.new_route_in_reallocate_when_i_less_than_j(current_solution, i, j)
            else:
                new_route = self.new_route_in_reallocate_when_j_less_than_i(current_solution, i, j)

            if self.fitness(new_route) < self.fitness(best_solution):
                best_solution = new_route
        return best_solution

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
