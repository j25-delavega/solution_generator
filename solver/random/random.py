import math
import random


def solution(random_number_vector):
    n = len(random_number_vector)
    random_solution_to_TSP = [0]
    for i in range(1, n + 1):
        random_solution_to_TSP.append(-1)

    for order in range(1, n + 1):
        minimum = math.inf
        minimum_position = -2
        for pos in range(0, n):
            if -1 == random_solution_to_TSP[pos + 1]:
                if random_number_vector[pos] < minimum:
                    minimum = random_number_vector[pos]
                    random_solution_to_TSP[pos + 1] = order
                    if minimum_position != -2:
                        random_solution_to_TSP[minimum_position] = -1
                    minimum_position = pos + 1

    return random_solution_to_TSP


def random_solution(n, seed):
    random.seed(seed)
    random_number_vector = []
    for r in range(0, n):
        random_number_vector.append(random.random())
    return solution(random_number_vector)


if __name__ == '__main__':
    random_number_vector = []
    for r in range(0, 11):
        random_number_vector.append(random.random())
    runs = solution(random_number_vector)
    print(runs)
