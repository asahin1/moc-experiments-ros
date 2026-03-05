from planner_adapter.object_caging_graph import ObjectCagingGraph
from planner_adapter.action import ActionDirection, ActionType, ActionSequences
from planner_adapter.point_2d import Point2D


def act_dir_to_int(act_dir):
    if act_dir == ActionDirection.REAR:
        return 0
    elif act_dir == ActionDirection.FRONT:
        return 1
    elif act_dir == ActionDirection.NA:
        raise ValueError("Should not attemp to convert NA action to int")
    else:
        raise ValueError("Unknown ActionDirection")


class PostProcessor:
    def __init__(self, oc_graph: ObjectCagingGraph, img_height, vertex_size, scale):
        self.oc_graph = oc_graph
        self.img_height = img_height
        self.vertex_size = vertex_size
        self.scale = scale
        self.assigned_e_nodes = {}

    def run(self, act_sequences: ActionSequences, cable_trails, savePath):
        plans = self._post_process(act_sequences, cable_trails)
        return plans

    def save(self, plans, savePath):
        output = {"plans": plans}
        with open(savePath, "w") as f:
            import json

            json.dump(output, f, indent=4)

    def _from_map_to_world(self, point):
        return self.scale * Point2D(point.x, self.img_height - point.y)

    def _post_process(self, act_sequences: ActionSequences, cable_trails):
        init_vertices = [cable[0] for cable in cable_trails]
        final_vertices = [cable[-1] for cable in cable_trails]
        n_cables = act_sequences.n_cables
        print(f"N cables: {n_cables}")
        plans = [[] for i in range(n_cables)]

        # Assign e_nodes to each cable
        all_e_nodes = {i + 1: [0, 1] for i in range(len(self.oc_graph.nodes))}
        for i in range(n_cables):
            v_r = init_vertices[i]
            v_f = final_vertices[i]
            initE = all_e_nodes[v_r][0]
            all_e_nodes[v_r].remove(initE)
            finalE = all_e_nodes[v_f][0]
            all_e_nodes[v_f].remove(finalE)
            self.assigned_e_nodes[i] = [initE, finalE]

        print(f"Assigned e_nodes: {self.assigned_e_nodes}")
        for v_id, coords in self.oc_graph.e_coords.items():
            print(f"e coords at {v_id}: {coords[0]} - {coords[1]}")

        n_steps = act_sequences.n_steps
        for step in range(n_steps):
            joint_act = [act_seq[step] for act_seq in act_sequences.actions]
            for cable_id, act in enumerate(joint_act):
                act_dict = {}
                if act.actionType == ActionType.NOOP:
                    continue
                elif act.actionType == ActionType.WAIT:
                    continue
                elif act.actionType == ActionType.PROGRESS:
                    act_dict["type"] = "m"
                    waypoints = self.get_move_action_waypoints(act)
                    transformed_wps = [
                        self._from_map_to_world(point) for point in waypoints
                    ]
                    waypoints_dict = [
                        {"x": point.x, "y": point.y} for point in transformed_wps
                    ]
                    act_dict["waypoints"] = waypoints_dict
                elif act.actionType == ActionType.INTERLACE:
                    act_dict["type"] = "h"
                    act_dict["end"] = act_dir_to_int(act.actionDirection)
                    act_dict["vertex_id"] = act.currVertexId
                    vertex_coords = self.oc_graph.nodes[act.currVertexId].coords
                    transformed_vertex_coords = self._from_map_to_world(vertex_coords)
                    act_dict["vertex_coords"] = {
                        "x": transformed_vertex_coords.x,
                        "y": transformed_vertex_coords.y,
                    }
                    exit_pt = self.find_next_move_point(
                        step, cable_id, act_sequences.actions[cable_id]
                    )
                    transformed_exit = self._from_map_to_world(exit_pt)
                    act_dict["exit_point"] = {
                        "x": transformed_exit.x,
                        "y": transformed_exit.y,
                    }
                plans[cable_id].append(act_dict)
        return plans

    def get_move_action_waypoints(self, act):
        v1 = act.currVertexId
        v2 = act.nextVertexId
        edge_id = self.oc_graph.neighbors[v1][v2]
        points = self.oc_graph.edges[edge_id]["points"]
        v1_point = self.oc_graph.nodes[v1].coords
        if points[0] == v1_point:
            pass
        elif points[-1] == v1_point:
            points = points[::-1]
        else:
            raise RuntimeError(f"Edge points {points} do not connect to v1: {v1_point}")
        return points

    def find_next_move_point(self, step, tether_id, act_sequence):
        curr_act = act_sequence[step]
        v = curr_act.currVertexId
        if curr_act.actionDirection == ActionDirection.REAR:
            e_coords = self.oc_graph.e_coords[v][self.assigned_e_nodes[tether_id][0]]
            return e_coords
        n_steps = len(act_sequence)
        for i in range(step + 1, n_steps):
            act = act_sequence[i]
            if act.actionType == ActionType.PROGRESS:
                v1 = act.currVertexId
                v2 = act.nextVertexId
                edge_id = self.oc_graph.neighbors[v1][v2]
                points = self.oc_graph.edges[edge_id]["points"]
                v1_point = self.oc_graph.nodes[v1].coords
                if points[0] == v1_point:
                    pass
                elif points[-1] == v1_point:
                    points = points[::-1]
                else:
                    raise RuntimeError(
                        f"Edge points {points} do not connect to v1: {v1_point}"
                    )
                return points[1]
            else:
                continue
        # Return exit
        if curr_act.actionDirection == ActionDirection.FRONT:
            e_coords = self.oc_graph.e_coords[v][self.assigned_e_nodes[tether_id][1]]
            return e_coords
        else:
            raise RuntimeError("Unknown action direction for hitch")
