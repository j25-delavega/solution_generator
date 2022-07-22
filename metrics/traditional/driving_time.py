def driving_time_using_road_network(solution, request):
    distance_matrix = request['real_distances']
    return driving_time(solution, distance_matrix)


def driving_time_using_euclidean_distance(solution, request):
    distance_matrix = request['euclidean_distances']
    return 100 * driving_time(solution, distance_matrix)


def driving_time(solution, distance_matrix):
    driving_time_value = 0.0
    # Driving time
    for position in range(1, len(solution) - 1):
        driving_time_value += distance_matrix[solution[position]][solution[position + 1]]
    ''' # Cost: Depot to the first point of the route
    driving_time_value = distance_matrix[0][solution[1]]
    # Cost: Depot to the last point of the route
    driving_time_value += distance_matrix[0][solution[len(solution) - 1]]'''
    return driving_time_value
