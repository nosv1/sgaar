from __future__ import annotations

class Node:
    def __init__(
        self, 
        x: float, 
        y: float,
        start_to_node_cost: float=0,
        heuristic_cost: float=0,
        parent: Node=None
    ) -> None:
        self.x = x
        self.y = y
        self.start_to_node_cost = start_to_node_cost
        self.heuristic_cost = heuristic_cost
        self.parent = parent

    @property
    def id(self) -> str:
        return f"({self.x:.5f}, {self.y:.5f})"

    @property
    def total_cost(self) -> float:
        """
        Returns the total cost of the node
        """
        return self.start_to_node_cost + self.heuristic_cost

    def __eq__(self, other: Node) -> bool:
        """
        Checks if two nodes are equal
        """
        return self.id == other.id

    def from_id(id_str: str) -> Node:
        """
        Creates a node from an id string
        """
        x, y = id_str[1:-1].split(", ")
        return Node(float(x), float(y))

    def distance_to(self, other: Node) -> float:
        """
        Calculates the ecuclidean distance between two nodes
        """
        return ((self.x - other.x) ** 2 + (self.y - other.y) ** 2) ** 0.5

    def find_closest_node(self, nodes: list[Node]) -> Node:
        """
        Finds the closest node to the current node in a list of nodes.
        """
        closest_node: Node = None
        closest_distance: float = float('inf')
        for node in nodes:
            distance: float = self.distance_to(node)
            if distance < closest_distance:
                closest_node = node
                closest_distance = distance
        return closest_node

    def step_towards_node(self, node: Node, step_length: float) -> Node:
        """
        Move step_length towards node from root.
        """
        distance: float = self.distance_to(node)
        return Node(
            x=self.x + step_length * (node.x - self.x) / distance,
            y=self.y + step_length * (node.y - self.y) / distance
        )