from launch import LaunchDescription
from launch_ros.actions import Node
import numpy as np
import os


MAX_ITERS = 500
COMMUNICATION_TIME = 5e-2  # communication time period
VARS_DIM = 2

LOGS_DIR = os.path.join(os.path.dirname(__file__), "../../../../run")

SEED = 42    
rng = np.random.default_rng(SEED)


def generate_launch_description():
    num_agents = 6
    edge_dist = 2
    diag_dist = 2 * edge_dist
    distances = np.array([
        [0,         edge_dist,  0,          diag_dist,  0,          edge_dist],
        [edge_dist, 0,          edge_dist,  0,          diag_dist,  0],
        [0,         edge_dist,  0,          edge_dist,  0,          diag_dist],
        [diag_dist, 0,          edge_dist,  0,          edge_dist,  0],
        [0,         diag_dist,  0,          edge_dist,  0,          edge_dist],
        [edge_dist, 0,          diag_dist,  0,          edge_dist,  0],
    ])

    x0 = rng.random((num_agents, VARS_DIM))
    node_list = []
    os.makedirs(LOGS_DIR, exist_ok=True)

    for i in range(num_agents):
        node_list.append(
            Node(
                package = "discrete_formation",
                namespace = f"agent_{i}",
                executable = "generic_agent",
                parameters = [{
                    "id": i,
                    "communication_time": COMMUNICATION_TIME,
                    "neighbors": np.nonzero(distances[i])[0].tolist(),
                    "x0": x0[i].flatten().tolist(),
                    "dist": distances[i].tolist(),
                    "max_iters": MAX_ITERS,
                    "logs_dir": LOGS_DIR
                }],
                output = "screen",
                prefix = f"xterm -title 'agent_{i}' -hold -e",
            )
        )

    return LaunchDescription(node_list)
