import matplotlib.pyplot as plt


def plot_TSP(paths, data, seed):
    """
    path: List of lists with the different orders in which the nodes are visited
    points: coordinates for the different nodes
    num_iters: number of paths that are in the path list

    """
    points = data['coords']

    # Unpack the primary TSP path and transform it into a list of ordered
    # coordinates
    x = []
    y = []
    for i in range(1, len(paths)):
        point = paths[i]
        x.append(points[point][0])
        y.append(points[point][1])

    point = paths[0]
    # x.append(points[point][0])
    # y.append(points[point][1])
    # plt.xlabel('Kilómetros recorridos: ' + str(driving_time_using_road_network(paths, data)))
    plt.plot(x, y, 'bo-', linewidth=0.5, markersize=1)
    plt.xticks([])
    plt.yticks([])
    # plt.savefig('fig_' + str(seed) + '.png')
    plt.show()