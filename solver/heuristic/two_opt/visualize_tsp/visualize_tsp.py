import math

from mpl_toolkits.basemap import Basemap
import numpy as np

import matplotlib.pyplot as plt

from metrics.traditional.driving_time import driving_time_using_road_network


def plot_map(lat1, lat2, lon1, lon2, x_map, y_map):
    plt.figure(figsize=(x_map, y_map))
    mapa = Basemap(projection='cyl', resolution='h',
                   llcrnrlat=lat1, urcrnrlat=lat2,
                   llcrnrlon=lon1, urcrnrlon=lon2)
    mapa.drawcoastlines()
    mapa.fillcontinents(color='palegoldenrod', lake_color='lightskyblue')
    mapa.drawmapboundary(fill_color='lightskyblue')
    mapa.drawparallels(np.arange(lat1, lat2 + 0.5, 2), labels=[1, 0, 0, 0])
    mapa.drawmeridians(np.arange(lon1, lon2 + 0.5, 2), labels=[0, 0, 0, 1])
    return mapa


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
