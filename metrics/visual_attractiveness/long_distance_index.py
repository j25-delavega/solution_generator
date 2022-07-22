import numpy as np
from constants.constants import ZERO_TOLERANCE

'''
    The range function is open, meaning it doesn't take the last value.
'''


def long_distance_index(solution, distance_matrix):
    maximum_value = np.max(distance_matrix)
    total_long_distance_index = 0.0
    for pos_i in range(1, len(solution) - 2):
        total_long_distance_index_at_pos_i = 0.0
        for pos_j in range(pos_i + 1, len(solution)):
            direct_distance_between_pos_i_and_j = distance_matrix[solution[pos_i]][solution[pos_j]]
            route_distance_between_pos_i_and_j = 0.0
            for pos_k in range(pos_i, pos_j):
                route_distance_between_pos_i_and_j += distance_matrix[solution[pos_k]][solution[pos_k + 1]]

            if direct_distance_between_pos_i_and_j > ZERO_TOLERANCE:
                total_long_distance_index_at_pos_i += route_distance_between_pos_i_and_j \
                                                      / direct_distance_between_pos_i_and_j
            else:
                if route_distance_between_pos_i_and_j > ZERO_TOLERANCE:
                    total_long_distance_index_at_pos_i += maximum_value

        total_long_distance_index += total_long_distance_index_at_pos_i
    return total_long_distance_index
