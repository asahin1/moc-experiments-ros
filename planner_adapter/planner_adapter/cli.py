from pathlib import Path
import json

from dataclasses import dataclass

from planner_adapter.object_caging_graph import ObjectCagingGraph
from planner_adapter.action import ActionSequences
from planner_adapter.post_processor import PostProcessor


@dataclass
class PlannerAdapterConfig:
    img_height: int = 480
    vertex_size: int = 50
    scale: float = 0.01
    resource_dir_name: str = "resources"
    data_dir_name: str = "data"
    graph_dir_name: str = "graphs"
    raw_plan_dir_name: str = "raw_plans"
    processed_plan_dir_name: str = "processed_plans"
    metadata_filename: str = "metadata.json"
    act_seq_filename: str = "actions.jsonl"
    cable_trails_filename: str = "cables.jsonl"


def main():
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("timestamp", help="Which timestamp dir to load from")
    parser.add_argument("out_filename", help="Filename to save the processed plan")
    args = parser.parse_args()

    pa_config = PlannerAdapterConfig()

    data_dir_path = Path(pa_config.resource_dir_name) / pa_config.data_dir_name
    raw_plan_path = data_dir_path / pa_config.raw_plan_dir_name / args.timestamp

    # Load metadata
    metadata_file_path = raw_plan_path / pa_config.metadata_filename
    with open(metadata_file_path, "r") as metadata_file:
        metadata = json.load(metadata_file)
    print(metadata)

    # Load oc_graph
    graph_filename = metadata["graph_filename"]
    graph_name = metadata["graph_name"]
    graph_file_path = data_dir_path / pa_config.graph_dir_name / graph_filename
    oc_graph = ObjectCagingGraph.from_file(graph_file_path, graph_name)

    # Load action sequences
    action_file_path = raw_plan_path / pa_config.act_seq_filename
    act_sequences = ActionSequences.from_file(action_file_path)
    print(act_sequences.actions)

    # Load cable trails
    cable_file_path = raw_plan_path / pa_config.cable_trails_filename
    with open(cable_file_path, "r") as cable_file:
        cable_data = json.load(cable_file)

    post_processor = PostProcessor(
        oc_graph, pa_config.img_height, pa_config.vertex_size, pa_config.scale
    )
    plans = post_processor.run(act_sequences, cable_data, None)
    processed_plan_path = (
        data_dir_path / pa_config.processed_plan_dir_name / args.out_filename
    )
    post_processor.save(plans, processed_plan_path)


if __name__ == "__main__":
    main()
