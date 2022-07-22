import math

import numpy as np
from scipy.spatial import distance
from constants.constants import FACTOR, ZERO_TOLERANCE
import json


def build_output(output, request_name):
    output_files = "output/"
    file_name = request_name + '_output_summary'
    f = open(output_files + file_name + ".txt", "a+")
    output_string = ""
    for value in output:
        output_string += str(value) + "\t"
    f.write(output_string + "\n")
    f.close()


def build_solution_file_for_qgis(solution, request, request_name):
    output_files = "qgis/"

    points = request['points']
    depot = request['depot']

    file_name = request_name + '_solution'
    f = open(output_files + file_name + ".txt", "w+")
    f.write(str("ident" + "\t" + "lon" + "\t" + "lat" + "\n"))
    f.write(str(0) + "\t" + str(depot['lon']) + "\t" + str(depot['lat']) + "\n")

    for position in range(1, len(solution)):
        point = points[solution[position] - 1]
        f.write(str(position) + "\t" + str(point['lon']) + "\t" + str(point['lat']) + "\n")

    f.close()


def build_or_solution_file_for_qgis(request, request_name):
    output_files = "qgis/"

    vehicles = request['vehicles']
    points = vehicles[0]['tours'][0]['nodes']
    depot = points[0]

    file_name = request_name + '_solution'
    f = open(output_files + file_name + ".txt", "w+")
    f.write(str("ident" + "\t" + "lon" + "\t" + "lat" + "\n"))
    f.write(str(0) + "\t" + str(depot['lon']) + "\t" + str(depot['lat']) + "\n")

    for position in range(1, len(points) - 1):
        point = points[position]
        f.write(str(position) + "\t" + str(point['lon']) + "\t" + str(point['lat']) + "\n")

    f.close()


def build_files_for_qgis(request_list, path):
    output_files = "qgis/"
    for req in request_list:
        with open(path + req, 'r') as wor_file:
            optimization_request = json.load(wor_file)
            file_name = req.split(".")[0]
            f = open(output_files + file_name + ".txt", "w+")
            depot_lon = optimization_request["depotInfo"]["depotList"][0]["lon"]
            depot_lat = optimization_request["depotInfo"]["depotList"][0]["lat"]
            f.write(str("ident" + "\t" + "lon" + "\t" + "lat" + "\n"))
            f.write("0" + "\t" + str(depot_lon) + "\t" + str(depot_lat) + "\n")
            for point in optimization_request['routingInfo']['routeList']:
                f.write(str(point['counter'] + 1) + "\t" + str(point['lon']) + "\t" + str(point['lat']) + "\n")
            f.close()


def euclidean_distance_between_two_points(first_point, second_point):
    coordinate_first_point = (first_point['lon'], first_point['lat'])
    coordinate_second_point = (second_point['lon'], second_point['lat'])
    return FACTOR * distance.euclidean(coordinate_first_point, coordinate_second_point)


def angle_between_two_vectors(first_vector, second_vector):
    if is_zero_vector(first_vector):
        return 0.0

    if is_zero_vector(second_vector):
        return 0.0

    first_unit_vector = first_vector / np.linalg.norm(first_vector)
    second_unit_vector = second_vector / np.linalg.norm(second_vector)

    dot_product = np.dot(first_unit_vector, second_unit_vector)

    if dot_product > 1 - ZERO_TOLERANCE:
        return 0.0

    if math.isnan(np.arccos(dot_product)):
        print('\n')
        print(first_vector)
        print(second_vector)
        print(dot_product)
    return np.arccos(dot_product)


def is_zero_vector(vector):
    for value in vector:
        if abs(value) > ZERO_TOLERANCE:
            return False

    return True


def euclidean_distance_array(first_point, depot, point_list):
    distance_array = [euclidean_distance_between_two_points(first_point, depot)]

    for second_point in point_list:
        distance_array.append(euclidean_distance_between_two_points(first_point, second_point))
    return distance_array


def euclidean_distance_matrix(depot, points):
    distance_matrix = [euclidean_distance_array(depot, depot, points)]
    for first_point in points:
        distance_matrix.append(euclidean_distance_array(first_point, depot, points))

    return distance_matrix


def distances_matrix_remover(distances_matrix, positions):
    aux_positions = []
    for position in positions:
        aux_positions.append(position + 1)
    initial_cleaned_distances_matrix = np.delete(distances_matrix, aux_positions, axis=0)
    final_cleaned_distances_matrix = np.delete(initial_cleaned_distances_matrix, aux_positions, axis=1)
    return final_cleaned_distances_matrix.tolist()


def positions_remover(points):
    positions_to_remove = []
    for pos_i in range(0, len(points) - 1):
        for pos_j in range(pos_i + 1, len(points)):
            if pos_j not in positions_to_remove:
                first_point = points[pos_i]
                second_point = points[pos_j]
                if abs(first_point['lon'] - second_point['lon']) < ZERO_TOLERANCE \
                        and abs(first_point['lat'] - second_point['lat']) < ZERO_TOLERANCE:
                    positions_to_remove.append(pos_j)
    positions_to_remove.sort(reverse=True)
    return positions_to_remove


def points_remover(points, positions):
    for position in positions:
        del points[position]


def request_cleaner(o_request):
    points = o_request['routingInfo']['routeList']
    distances_matrix = o_request['distances'][0]['distance_table']
    depot = o_request['depotInfo']['depotList'][0]

    positions_to_remove = positions_remover(points)
    points_remover(points, positions_to_remove)
    distances = distances_matrix_remover(distances_matrix, positions_to_remove)

    depot_coordinate = [depot['lon'], depot['lat']]
    coordinates = [depot_coordinate]
    for point in points:
        point_coordinate = [point['lon'], point['lat']]
        coordinates.append(point_coordinate)

    cleaned_request = {'noNodes': len(points), 'depot': depot, 'points': points, 'real_distances': distances,
                       'euclidean_distances': euclidean_distance_matrix(depot, points), 'coords': coordinates}

    return cleaned_request


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
