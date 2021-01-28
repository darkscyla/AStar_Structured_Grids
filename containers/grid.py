# --- Internal Imports ---
from .sorted_nodes import SortedNodes
from .node import Node


class Grid:
    """
    Implements the A-Star algorithm for structured grids

    ...

    Attributes
    ----------
    start : tuple
        The id of the start node. ID comprises of (row, column) indices
    end : tuple
        The id of the end node. ID comprises of (row, column) indices
    closed_nodes : dict
        A dictionary contain the closed nodes ids and their node data.
        A closed node is one that cannot be visited anymore. This may
        happen if it is an obstacle or we already visited it
    sorted_nodes_map : SortedNodes
        A value sorted dict that stores the open nodes. An open node 
        is one that lies on the current search path, that is it is a
        potential candidate that may lead to the shortest path

    Methods
    -------
    astar(start: tuple, end: tuple, obstacle: list)
        Given a start and end node along with obstacles, search the
        domain for the shortest path using A-Star algorithm
    """
    def __init__(self, resolution: tuple):
        self.x, self.y = resolution

        # Start and end goal node
        self.start = None
        self.end = None

        # Stores the dict of closed nodes
        self.closed_nodes = {}

        # Sorted open nodes dictionary
        self.sorted_nodes_map = SortedNodes()

    def astar(self, start: tuple, end: tuple, obstacles: list):
        """
        Computes the A-Star path given a start and end goal with obstacles.
        Currently, we automatically add a parameter obstacles for the grid
        to make sure its remains bounded
        """
        # We reset the internal containers
        self._reset()

        self.start = start
        self.end = end

        # We add the boundary of domain to the closed obstacles
        self._add_boundary()

        # We add the obstacles to the closed nodes
        self.closed_nodes.update({obstacle: None for obstacle in obstacles})

        # We need to compute the heuristic cost only once
        # per node once we know the end goal
        self._add_nodes_to_map((start, end))

        # Update the cost of the start node from the start node
        # which is of course 0
        self.sorted_nodes_map.update_cost(start, 0.0, None)

        # We now loop over the nodes in our sorted map and get the
        # best path until we reach the end node
        curr_vertex = self.sorted_nodes_map.pop_smallest()

        # Just a sanity check
        assert (
            curr_vertex.id == self.start
        ), "Expected the first node to be the start vertex"

        while curr_vertex.id != self.end:

            # We mark the current vertex as closed
            self.closed_nodes[curr_vertex.id] = curr_vertex.node

            # Get the neighbours with their cost
            neighbours = self._get_neighbours_and_costs(curr_vertex.id)

            # We add the nodes if not already in our list
            self._add_nodes_to_map(neighbours.keys())

            # We finally update the cost of the nodes
            for neighbour, cost in neighbours.items():
                self.sorted_nodes_map.update_cost(
                    neighbour, curr_vertex.node.g_cost() + cost, curr_vertex.id
                )

            # We move on to the next best candidate if possible. If no node is
            # inside sorted_nodes_map, no solution exists
            if len(self.sorted_nodes_map) > 0:
                curr_vertex = self.sorted_nodes_map.pop_smallest()
            else:
                print("No solution exists!")
                return []

        # We mark the end node as closed as well
        self.closed_nodes[curr_vertex.id] = curr_vertex.node

        # We now compute the path that led to the solution
        return self._get_astar_path()

    def _add_nodes_to_map(self, nodes_list):
        """
        Helper function to compute the heuristic cost of the nodes
        """
        for node in nodes_list:
            # node here basically is the key and represents
            # the coordinates of the nodes as well
            if node not in self.sorted_nodes_map:
                self.sorted_nodes_map[node] = Node(self._compute_heuristic(node))

    def _compute_heuristic(self, origin):
        """
        Computes the grid distance between the origin and
        the end node. We can move diagonally or straight
        along a grid cell. We can better estimate the actual
        distance using this info
        """
        diff = (abs(origin[0] - self.end[0]), abs(origin[1] - self.end[1]))
        return 1.4142135623730951 * min(diff) + abs(diff[0] - diff[1])

    def _get_neighbours_and_costs(self, vertex: tuple):
        """
        As we have a grid, we can hardcode these neighbours including the cost
        for efficiency reasons. In case of a different data structure, this is
        the only function then would need to be changed
        """
        neighbours = {
            (vertex[0], vertex[1] + 1): 1.0,  # N
            (vertex[0] + 1, vertex[1] + 1): 1.4142135623730951,  # NE
            (vertex[0] + 1, vertex[1]): 1.0,  # E
            (vertex[0] + 1, vertex[1] - 1): 1.4142135623730951,  # SE
            (vertex[0], vertex[1] - 1): 1.0,  # S
            (vertex[0] - 1, vertex[1] - 1): 1.4142135623730951,  # SW
            (vertex[0] - 1, vertex[1]): 1.0,  # W
            (vertex[0] - 1, vertex[1] + 1): 1.4142135623730951,  # NW
        }

        # We remove the closed vertices from the neighbours if any
        return {
            neighbour: cost
            for neighbour, cost in neighbours.items()
            if neighbour not in self.closed_nodes
        }

    def _get_astar_path(self):
        """
        Traceback through the most efficient neighbours until it
        reach's the start node
        """
        path = []
        curr_node = self.end

        while curr_node != self.start:
            path.append(curr_node)

            # We get the previous best vertex of the node
            curr_node = self.closed_nodes[curr_node].previous_vertex()

        # We add the first node to the list as well
        path.append(curr_node)

        return path

    def _reset(self):
        """
        Clears out the internal containers
        """
        self.closed_nodes = {}
        self.sorted_nodes_map = SortedNodes()

    def _add_boundary(self):
        """
        Adds the boundary to the closed vertices list. OpenCV
        indexes oppositely
        """
        self.closed_nodes.update({(-1, x): None for x in range(-1, self.x + 1)})  # S
        self.closed_nodes.update(
            {(y, self.x): None for y in range(-1, self.y + 1)}
        )  # E
        self.closed_nodes.update(
            {(self.y, x): None for x in range(-1, self.x + 1)}
        )  # S
        self.closed_nodes.update({(y, -1): None for y in range(-1, self.y + 1)})  # W
