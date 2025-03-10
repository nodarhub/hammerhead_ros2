import rclpy
from rclpy.node import Node

from hammerhead_msgs.srv import CameraParam


class ClientNode:
    def __init__(self, node_name, topic):
        rclpy.init()
        self.node = Node(node_name)
        self.topic = topic
        self.clients = {}

    def send_request(self, val):
        if self.topic not in self.clients:
            self.clients[self.topic] = self.node.create_client(CameraParam, self.topic)

        client = self.clients[self.topic]
        if not client.wait_for_service(timeout_sec=1.0):
            print(f"There does not appear to be a service for the topic: `{self.topic}`\n"
                  "Please check the spelling, and check that the service is actually up by running\n\n"
                  "    ros2 service list\n")
            return

        request = CameraParam.Request()
        request.val = val
        future = client.call_async(request)
        print("Waiting for a response...")
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is not None:
            print("Client")
            print(f"    request->val      : {request.val}")
            print(f"    response->success : {future.result().success}")
        else:
            print("Failed to call service")


def main(topic):
    print(f"{topic}\n\n--------------------\n"
          "To set a parameter, just input the desired value, and press ENTER.\n"
          "--------------------\n")

    client_node = ClientNode("client_node", topic)
    while True:
        try:
            val = float(input())
            print(f"Requesting {topic} = {val}")
            client_node.send_request(val)
        except ValueError:
            print("Could not parse your last input. Try entering that again.")

    client_node.node.destroy_node()
    rclpy.shutdown()


def main_gain():
    main("nodar/set_gain")


def main_exposure():
    main("nodar/set_exposure")
