from utils.utils import Point
from metrics.route_complexity.crossings import doIntersect
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


def crossings(solution, coords_list):
    number_of_crossings = 0
    for position_p1 in range(1, len(solution) - 2):
        info_p1 = coords_list[solution[position_p1]]
        info_q1 = coords_list[solution[position_p1 + 1]]
        point_p1 = Point(info_p1[0], info_p1[1])
        point_q1 = Point(info_q1[0], info_q1[1])

        for position_p2 in range(position_p1 + 2, len(solution) - 1):
            info_p2 = coords_list[solution[position_p2]]
            info_q2 = coords_list[solution[position_p2 + 1]]
            point_p2 = Point(info_p2[0], info_p2[1])
            point_q2 = Point(info_q2[0], info_q2[1])

            if doIntersect(point_p1, point_q1, point_p2, point_q2):
                number_of_crossings += 1

    return number_of_crossings


"""Simple Travelling Salesperson Problem (TSP) between cities."""


def create_data_model(request):
    data = {'distance_matrix': request['real_distances'], 'num_vehicles': 1, 'depot': 0}
    return data


def print_solution(manager, routing, solution):
    route = []
    """Prints solution on console."""
    # print('Objective: {} miles'.format(solution.ObjectiveValue()))
    index = routing.Start(0)
    plan_output = 'Route for vehicle 0:\n'
    route_distance = 0
    while not routing.IsEnd(index):
        plan_output += ' {} ->'.format(manager.IndexToNode(index))
        route.append(index)
        previous_index = index
        index = solution.Value(routing.NextVar(index))
        route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
    plan_output += ' {}\n'.format(manager.IndexToNode(index))
    # print(plan_output)
    plan_output += 'Route distance: {}miles\n'.format(route_distance)

    return route


def ORTools_run(request):
    route = []

    """Entry point of the program."""
    # Instantiate the data problem.
    data = create_data_model(request)

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_vehicles'], data['depot'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        route = print_solution(manager, routing, solution)

    return route
