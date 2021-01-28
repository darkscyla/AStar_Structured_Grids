# --- Standard Imports ---
import math


class Node:
    """
    Class to store an instance of graph node

    ...

    Attributes
    ----------
    distance_heuristic : float
        The heuristic distance of the node from the end node (h cost)
    distance_from_start : float
        The shortest distance from current node to start node (g cost)
    cost : float
        The total cost of the node -> is distance_heuristic + distance_from_start (f cost)
    prev_vertex: tuple
        ID of the immediate node through which we can reach the start with
        shoetest distance

    Methods
    -------
    update_cost(val: float)
        Updates the cost of the node, usually happens when we
        discover a shorter route to the start node
    """

    def __init__(self, heuristic_cost):
        """
        Parameters
        ----------
        heuristic_cost: float
            The heuristic distance of the current node from the end node
        """
        self.distance_heuristic = heuristic_cost

        self.dist_from_start = float("inf")
        self.cost = float("inf")

        # Store the last vertex that connect the current node
        # with the start node
        self.prev_vertex = None

    def g_cost(self):
        """
        Fetches the g cost of the node
        """
        return self.dist_from_start

    def previous_vertex(self):
        """
        Returns the best known neighbour of the current node 
        """
        return self.prev_vertex

    def update_cost(self, vertex: tuple, val: float):
        """
        Updates the cost of the node given a new g cost iff cost is lower
        than the internal cost. We also store the immediate vertex which
        results in the shorter path

        Parameters
        ----------
        vertex: tuple

        val: float
            The new g cost
        """
        self.prev_vertex = vertex
        self.dist_from_start = val

        # The total cost is summation of the two
        self.cost = self.dist_from_start + self.distance_heuristic

    ###### Operator Overloads ######
    def __lt__(self, other):
        return self.cost < other.cost

    def __le__(self, other):
        return self.__lt__(other) or self.__eq__(other)

    def __gt__(self, other):
        return self.cost > other.cost

    def __ge__(self, other):
        return self.__gt__(other) or self.__eq__(other)

    def __eq__(self, other):
        return math.isclose(self.cost, other.cost)

    def __ne__(self, other):
        return not self.__eq__(other)
