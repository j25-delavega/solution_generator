from metrics.route_complexity.bending_energy import bending_energy
from utils.utils import Point, euclidean_distance_matrix
from metrics.route_complexity.crossings import crossings
from metrics.route_complexity.long_distance_index import long_distance_index


def bending_energy_metric(solution, request):
    points = request['points']
    return bending_energy(solution, points)


def crossings_metric(solution, request):
    points = request['points']
    return crossings(solution, points)


def long_distance_index_metric_using_road_network(solution, request):
    distances = request['real_distances']
    return long_distance_index(solution, distances)


def long_distance_index_metric_using_euclidean_distance(solution, request):
    distances = request['euclidean_distances']
    return long_distance_index(solution, distances)
