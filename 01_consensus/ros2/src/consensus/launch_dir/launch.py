from launch import LaunchDescription
from launch_ros.actions import Node
import networkx as nx
import numpy as np


MAX_ITER = 15
NUM_AGENTS = 5
SEED = 42


def generate_launch_description():
    node_list = []
    G = nx.path_graph(NUM_AGENTS)
    x0 = np.random.default_rng(SEED).integers(low=0, high=100, size=NUM_AGENTS)
    print(f"x0={x0}")

    for i in range(NUM_AGENTS):
        node_list.append(
            Node(
                package = "consensus",
                namespace = f"agent_{i}",
                executable = "generic_agent",
                parameters = [
                    {
                        "id": i,
                        "neighbors": list(G.neighbors(i)),
                        "x0": int(x0[i]),
                        "max_iters": MAX_ITER
                    }
                ],
                output = "screen",
                prefix = f"xterm -title 'agent_{i}' -hold -e",
            )
        )

    return LaunchDescription(node_list)
