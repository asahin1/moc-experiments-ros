from planner_adapter.point_2d import Point2D


class ObjectCagingNode:
    def __init__(self, id: int, coords: Point2D):
        self.id = id
        self.coords = coords


class ObjectCagingGraph:
    """ """

    def __init__(self, nodes, neighbors, edges, e_coords, object_cycles) -> None:
        self.nodes = nodes  # Dict - id: handle
        self.neighbors = neighbors  # Dict - node_id: {Dict - neighbor_id: edge_id}
        self.edges = edges  # Dict - id : {Dict - points: [], cost: float}
        self.e_coords = e_coords  # Dict - node_id: [e1, e2]
        self.object_cycles = object_cycles  # List - [[cycle 1 nodes], [], ...]

    @classmethod
    def from_dict(cls, data):
        """
        Pure logic. Turns a dictionary into an object.
        NO file I/O here.
        """
        # Validate data
        if "nodes" not in data or "edges" not in data or "loops" not in data:
            raise ValueError("Invalid dictionary format")

        # Parse logic
        nodes = {}
        neighbors = {}
        edges = {}
        e_coords = {}
        object_cycles = []
        for node_id, info in data["nodes"].items():
            node_id = int(node_id)
            nodes[node_id] = ObjectCagingNode(node_id, Point2D(info["x"], info["y"]))

            neighbors[node_id] = {
                int(nbh_id): edge_id for nbh_id, edge_id in info["neighbors"].items()
            }
            for edge_id in neighbors[node_id].values():
                if edge_id not in edges:
                    edge_data = data["edges"][str(edge_id)]
                    points = [
                        Point2D(x, y) for x, y in zip(edge_data["x"], edge_data["y"])
                    ]
                    if "cost" in edge_data:
                        cost = edge_data["cost"]
                    else:
                        total = 0
                        for i in range(len(points) - 1):
                            total += points[i].getDistance(points[i + 1])
                        cost = total
                    edges[int(edge_id)] = {"points": points, "cost": cost}

        for node_id, info in data["nodes"].items():
            node_id = int(node_id)
            if "e1" in info.keys() and "e2" in info.keys():
                e_coords[node_id] = (
                    Point2D(info["e1"][0], info["e1"][1]),
                    Point2D(info["e2"][0], info["e2"][1]),
                )

        for loop_id, info in data["loops"].items():
            object_cycles.append(info)

        return cls(nodes, neighbors, edges, e_coords, object_cycles)

    @classmethod
    def from_file(cls, filepath, name):
        """
        I/O only. Delegates logic to from_dict.
        """
        import json

        with open(filepath, "r") as f:
            all_graph_data = json.load(f)
            if name not in all_graph_data:
                raise KeyError(
                    f"Graph '{name}' not found. Available: {list(all_graph_data)}"
                )
            raw_graph_data = all_graph_data[name]

        return cls.from_dict(raw_graph_data)
