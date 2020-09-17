from sortedcollections import ValueSortedDict
from collections import namedtuple


class SortedNodes(ValueSortedDict):
    """
    A convinience extension of value sorted dict
    to allow already added elements to be changed
    """

    NodeWithID = namedtuple("NodeWithID", "id, node")

    def __init__(self, *args, **kwargs):
        # Forward it to the base class constructor
        super().__init__(*args, **kwargs)

    def update_cost(self, node_id, cost: float, neighbour_node_id):
        """
        Changes the already existing entry in the sorted list

        Parameters
        ----------
        node:
            The node that needs to be updated
        cost:
            The new cost of the node

        Raises
        ------
        ValueError:
            If the node is not in the sorted dict
        """
        if cost < self[node_id].g_cost():
            node = self.pop(node_id)
            node.update_cost(neighbour_node_id, cost)
            self[node_id] = node

    def pop_smallest(self):
        return self.NodeWithID(*self.popitem(0))
