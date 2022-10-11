def driving_time_using_road_network(solution, request):
    distance_matrix = request['real_distances']
    return driving_time(solution, distance_matrix, True)


def driving_time_using_euclidean_distance(solution, request):
    distance_matrix = request['euclidean_distances']
    return driving_time(solution, distance_matrix, True)


def driving_time(solution, distance_matrix, is_to_print):
    driving_time_value = 0.0
    # Driving time
    for position in range(1, len(solution) - 1):
        driving_time_value += distance_matrix[solution[position]][solution[position + 1]]
    # Cost: Depot to the first point of the route
    # driving_time_value += distance_matrix[0][solution[1]]
    # Cost: Depot to the last point of the route
    if is_to_print:
        driving_time_value += distance_matrix[0][solution[1]]
        driving_time_value += distance_matrix[0][solution[len(solution) - 1]]
    else:
        driving_time_value += distance_matrix[0][solution[1]]
        # print(" >> Cost: Depot to the last point of the route: " + str(distance_matrix[0][solution[len(solution) - 1]]))
        driving_time_value -= distance_matrix[0][solution[len(solution) - 1]]
    return 100 * driving_time_value
