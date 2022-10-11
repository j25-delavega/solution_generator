from metrics.traditional.driving_time import driving_time_using_road_network, driving_time_using_euclidean_distance
from metrics.visual_attractiveness.route_complexity import bending_energy_metric, crossings_metric, \
    long_distance_index_metric_using_road_network, long_distance_index_metric_using_euclidean_distance


def output_file(seed, route, data):
    output = [seed]

    for point in route:
        output.append(point)

    # Traditional metrics
    dt_using_rn = driving_time_using_road_network(route, data)
    print("Driving time using road network: " + str(dt_using_rn))
    output.append(dt_using_rn)

    dt_using_ed = driving_time_using_euclidean_distance(route, data)
    print("Driving time using euclidean distance: " + str(dt_using_ed))
    output.append(dt_using_ed)

    # Route simplicity metrics
    bending_energy = bending_energy_metric(route, data)
    print("Total bending energy: " + str(bending_energy))
    output.append(bending_energy)

    number_of_crossings = crossings_metric(route, data)
    print("Total crossings: " + str(number_of_crossings))
    output.append(number_of_crossings)

    total_ldi_using_road_network = long_distance_index_metric_using_road_network(route, data)
    print("Total long distance index using road network: " + str(total_ldi_using_road_network))
    output.append(total_ldi_using_road_network)

    total_ldi_using_euclidean_distance = long_distance_index_metric_using_euclidean_distance(route, data)
    print("Total long distance index using euclidean distance: " + str(total_ldi_using_euclidean_distance))
    output.append(total_ldi_using_euclidean_distance)

    return output
