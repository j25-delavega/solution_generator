from utils.utils import angle_between_two_vectors


def bending_energy(solution, points):
    bending_energy_sum = 0.0
    for position in range(3, len(solution)):
        point_i_minus_2 = points[solution[position - 2] - 1]
        point_i_minus_1 = points[solution[position - 1] - 1]
        point_i_minus_0 = points[solution[position - 0] - 1]
        first_vector = [point_i_minus_2['lon'] - point_i_minus_1['lon'],
                        point_i_minus_2['lat'] - point_i_minus_1['lat']]
        second_vector = [point_i_minus_0['lon'] - point_i_minus_1['lon'],
                         point_i_minus_0['lat'] - point_i_minus_1['lat']]
        bending_energy_sum += angle_between_two_vectors(first_vector, second_vector)

    return bending_energy_sum
