from util import PriorityDict
import math

DIST_DICT = {'astar_euclidean': 'euclidean_heuristic',
             'dijkstra': 'zero_heuristic'}


class RoutePlanner:
    """
    Class that performs Dijkstra or A-star route search
    """
    def __init__(self, heuristic='astar_euclidean'):
        """
        Constructor of Class RoutePlanner
        :param heuristic: the option of distance heuristic, defined in DIST_DICT
        """
        self.dist_heuristic = heuristic

    def set_dist_heuristic(self, heuristic):
        """
        Method to set the option of distance heuristic
        :param heuristic: the option of distance heuristic, defined in DIST_DICT
        :return:
        """
        self.dist_heuristic = heuristic

    def zero_heuristic(self, n1, n2):
        return 0

    def euclidean_heuristic(self, n1, n2):
        # Get the longitude and latitude for each vertex.
        long1 = n1['x'] * math.pi / 180.0
        lat1 = n1['y'] * math.pi / 180.0
        long2 = n2['x'] * math.pi / 180.0
        lat2 = n2['y'] * math.pi / 180.0

        # Use a spherical approximation of the earth for
        # estimating the distance between two points.
        r = 6371000
        x1 = r * math.cos(lat1) * math.cos(long1)
        y1 = r * math.cos(lat1) * math.sin(long1)
        z1 = r * math.sin(lat1)

        x2 = r * math.cos(lat2) * math.cos(long2)
        y2 = r * math.cos(lat2) * math.sin(long2)
        z2 = r * math.sin(lat2)

        d = ((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2) ** 0.5

        return d

    def distance_heuristic(self, state_key, goal_key, node_data):
        """
        Computer the distance heuristic between the current state and the goal
        :param state_key: current vertex key
        :param goal_key: goal vertex key
        :param node_data: nodes of the graph
        :return: distance heuristic between the current state and the goal
        """

        if self.dist_heuristic in DIST_DICT.keys():
            func = getattr(self, DIST_DICT[self.dist_heuristic])
        else:
            print('Warning: no heuristic selected! Default to AStar-Euclidean')
            func = getattr(self, DIST_DICT['astar_euclidean'])

        n1 = node_data[state_key]
        n2 = node_data[goal_key]
        return func(n1, n2)

    def get_path(self, origin_key, goal_key, predecessors):
        """
        Method to retrieve a path from the predecessor back-pointers
        :param origin_key:
        :param goal_key:
        :param predecessors:
        :return: a path as a list of vertex keys.
        """
        key = goal_key
        path = [goal_key]

        while key != origin_key:
            key = predecessors[key]
            path.insert(0, key)

        return path

    # For a given graph, origin vertex key, and goal vertex key,
    #  using A* search.
    # Returns
    def route_search(self, origin_key, goal_key, graph):
        """
        Computes the shortest path in the graph from the origin vertex to the goal vertex.
        Depending on the distance heuristic selected, it performs A-star or Dijkstra search
        :param origin_key: key of the origin vertex in the graph
        :param goal_key: key of the destination vertex in the graph
        :param graph: a graph of the OSMNX format
        :return: the shortest path as a list of vertex keys.
        """
        # The priority queue of open vertices we've reached.
        # Keys are the vertex keys, vals are the accumulated
        # distances plus the heuristic estimates of the distance
        # to go.
        open_queue = PriorityDict({})

        # The dictionary of closed vertices we've processed.
        closed_dict = set()  # {}

        # The dictionary of predecessors for each vertex.
        predecessors = {}

        # The dictionary that stores the best cost to reach each
        # vertex found so far.
        costs = {}

        # Get the spatial data for each vertex as a dictionary.
        node_data = graph.nodes(True)

        # Add the origin to the open queue and the costs dictionary.
        costs[origin_key] = 0.0
        open_queue[origin_key] = self.distance_heuristic(origin_key, goal_key, node_data)

        # Iterate through the open queue, until we find the goal.
        # Each time, perform an A* update on the queue.
        # TODO: Implement the A* update loop.
        goal_found = False
        while open_queue:
            # pass
            (u, _) = open_queue.pop_smallest()
            uCost = costs[u]
            if u == goal_key:
                goal_found = True
                break

            for edge in graph.out_edges(u, data=True):  # v = edge[1], length = edge[2]['length']
                if edge[1] in closed_dict:
                    # print("Edge found in cosed_dict = {}".format(edge[1]))
                    continue

                f_cost = uCost + edge[2]['length'] + self.distance_heuristic(edge[1], goal_key, node_data)
                if edge[1] in open_queue:
                    # print("Edge found in open_queue = {}".format(edge[1]))

                    if f_cost < open_queue[edge[1]]:
                        open_queue[edge[1]] = f_cost
                        costs[edge[1]] = uCost + edge[2]['length']
                        predecessors[edge[1]] = u
                else:
                    open_queue[edge[1]] = f_cost
                    costs[edge[1]] = uCost + edge[2]['length']
                    predecessors[edge[1]] = u

            closed_dict.add(u)

        # If we get through entire priority queue without finding the goal,
        # something is wrong.
        if not goal_found:
            raise ValueError("Goal not found in search.")

        # Construct the path from the predecessors dictionary.
        return self.get_path(origin_key, goal_key, predecessors)
