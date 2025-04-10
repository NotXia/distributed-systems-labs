import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray as MsgFloat
from time import sleep
import os


def formation_vect_field_step(curr_x, neighbors_state, distances, discretization_time):
    d_xi = np.zeros_like(curr_x)

    for j in neighbors_state:
        x_j = neighbors_state[j]
        d_V_ij = (((curr_x - x_j).T @ (curr_x - x_j)) - distances[j]**2) * (curr_x - x_j)
        d_xi += -d_V_ij

    # Forward Euler discretization
    return curr_x + (discretization_time * d_xi)


class Agent(Node):
    def __init__(self):
        super().__init__(
            "formation_control_agent",
            allow_undeclared_parameters = True,
            automatically_declare_parameters_from_overrides = True,
        )
        self.id = self.get_parameter("id").value
        self.neighbors = self.get_parameter("neighbors").value
        self.curr_x = np.array( self.get_parameter("x0").value )
        self.curr_k = 0
        self.distances = np.array( self.get_parameter("dist").value )
        self.max_iters = self.get_parameter("max_iters").value
        communication_time = self.get_parameter("communication_time").value
        self.discretization_time = communication_time / 10
        self.logs_file = os.path.join(self.get_parameter("logs_dir").value, f"agent{self.id}.csv")
        open(self.logs_file, "w").close()

        print(f"I am agent: {self.id:d}")

        # Topics initialization
        self.received_queue = {j: [] for j in self.neighbors}
        for j in self.neighbors:
            self.create_subscription(MsgFloat, f"/topic_{j}", self.listener_callback, 10)
        self.publisher = self.create_publisher(MsgFloat, f"/topic_{self.id}", 10)

        # Communication polling
        self.timer = self.create_timer(communication_time, self.timer_callback)


    def __log(self):
        print(f"Agent {self.id} x_{self.curr_k} = {self.curr_x}")
        with open(self.logs_file, "a") as f:
            f.write(f"{self.id};{self.curr_k};{';'.join([str(x) for x in self.curr_x.tolist()])}\n")


    def listener_callback(self, msg):
        sender_id = int(msg.data[0])
        timestep = int(msg.data[1])
        state = msg.data[2:]
        self.received_queue[sender_id].append({ "timestep": timestep, "state": state })


    def __send_state(self):
        msg = MsgFloat()
        msg.data = [float(self.id), float(self.curr_k), *self.curr_x]
        self.publisher.publish(msg)


    def timer_callback(self):
        if self.curr_k == 0:
            self.__log()
            self.__send_state()
            self.curr_k += 1
        elif all((len(self.received_queue[j]) > 0) and (self.received_queue[j][0]["timestep"] == self.curr_k-1) for j in self.neighbors):
            self.curr_x = formation_vect_field_step(
                self.curr_x,
                { j: self.received_queue[j].pop(0)["state"] for j in self.neighbors },
                self.distances,
                self.discretization_time
            )

            self.__log()
            self.__send_state()
            self.curr_k += 1

            if self.curr_k > self.max_iters:
                self.destroy_node()


def main():
    rclpy.init()

    agent = Agent()
    sleep(1) # For synchronization (kind of)
    rclpy.spin(agent)
    agent.destroy_node()

    rclpy.shutdown()