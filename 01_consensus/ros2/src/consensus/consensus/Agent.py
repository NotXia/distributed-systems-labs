import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from time import sleep
from queue import Queue
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy, Duration



class Agent(Node):
    def __init__(self):
        super().__init__(
            "parametric_agent",
            allow_undeclared_parameters = True,
            automatically_declare_parameters_from_overrides = True,
        )
        self.agent_id = self.get_parameter("id").value
        self.neighbors = self.get_parameter("neighbors").value
        self.x0 = self.get_parameter("x0").value
        self.max_iters = self.get_parameter("max_iters").value
        self.x_prev = None
        self.x_curr = self.x0
        self.curr_k = 0
        self.received_queue = { j: Queue() for j in self.neighbors }

        print(f"Agent {self.agent_id:d}: neighbors={self.neighbors} x0={self.x0}")

        for j in self.neighbors:
            self.create_subscription(Float32MultiArray, f"/topic{j}", self.receive_cb, qos_profile=10)
        self.pub = self.create_publisher(Float32MultiArray, f"/topic{self.agent_id}", qos_profile=10)

        self.timer = self.create_timer(1.0, self.send_cb)


    def receive_cb(self, msg: Float32MultiArray):
        receiver_idx = int(msg.data[0])
        message = {
            "k": msg.data[1],
            "value": msg.data[2],
        }
        # Drop duplicate messages
        if message["k"] not in [msg["k"] for msg in self.received_queue[receiver_idx].queue]:
            self.received_queue[receiver_idx].put( message )


    def __send_current_value(self):
        msg = Float32MultiArray()
        msg.data = (float(self.agent_id), float(self.curr_k), float(self.x_curr))
        self.pub.publish(msg)

    def __send_previous_value(self):
        msg = Float32MultiArray()
        msg.data = (float(self.agent_id), float(self.curr_k-1), float(self.x_prev))
        self.pub.publish(msg)

    def send_cb(self):
        print(f"Agent {self.agent_id:d}: k={self.curr_k} x_{self.curr_k}={self.x_curr}")

        if self.curr_k == 0:
            self.__send_current_value()
            self.x_prev = self.x_curr
            self.curr_k += 1
        elif all([ (not self.received_queue[j].empty()) and (self.curr_k-1 == self.received_queue[j].queue[0]["k"]) for j in self.neighbors ]): 
            # Received message from all neighbors
            curr_max = self.x_curr
            for j in self.neighbors:
                neigh_message = self.received_queue[j].get()
                curr_max = max(curr_max, neigh_message["value"])

            self.x_prev = self.x_curr
            self.x_curr = curr_max
            self.__send_current_value()
            self.curr_k += 1
        else:
            # Resend previous message in case of late arriving nodes
            self.__send_previous_value()

        if self.curr_k >= self.max_iters:
            self.destroy_node()


def main():
    rclpy.init()

    agent = Agent()
    rclpy.spin(agent)
    agent.destroy_node()

    rclpy.shutdown()
