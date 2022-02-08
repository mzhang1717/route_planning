import osmnx as ox
import networkx as nx
from routeplanner import RoutePlanner
import sys


def main():
    # Load map network
    # map_graph = ox.graph_from_place('Berkeley, California', network_type='drive')
    # origin = ox.nearest_nodes(map_graph, 37.8743, -122.277)
    # destination = list(map_graph.nodes())[-1]
    map_graph = ox.graph_from_place('Toronto, Canada', network_type='drive')
    origin = ox.nearest_nodes(map_graph, -79.40219, 43.65852)
    # destination = ox.nearest_nodes(map_graph, -79.39012, 43.66513)
    destination = ox.nearest_nodes(map_graph, -79.35370, 43.67674)

    shortest_path = nx.shortest_path(map_graph, origin, destination, weight='length')
    fig, ax = ox.plot_graph_route(map_graph, shortest_path)
    print(shortest_path)

    router = RoutePlanner(heuristic='astar_euclidean')  # astar_euclidean, dijkstra

    path = router.route_search(origin, destination, map_graph)
    fig, ax = ox.plot_graph_route(map_graph, path)
    print(path)


if __name__ == '__main__':
    try:
        main()
    except:
        print("Whew!", sys.exc_info()[0], "occurred")



