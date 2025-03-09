import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class ClientNode:
    def __init__(self, node_name):
        rclpy.init()
        self.node = Node(node_name)
        self.clients = {}

    def send_request(self, topic, val):
        if topic not in self.clients:
            self.clients[topic] = self.node.create_client(SetBool, topic)

        client = self.clients[topic]

        if not client.wait_for_service(timeout_sec=1.0):
            print(f"There does not appear to be a service for the topic: `{topic}`\n"
                  "Please check the spelling, and check that the service is actually up by running\n\n"
                  "    ros2 service list\n")
            return

        request = SetBool.Request()
        request.data = val

        future = client.call_async(request)
        print("Waiting for a response...")

        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is not None:
            print("Client")
            print(f"    request->val      : {request.data}")
            print(f"    response->success : {future.result().success}")
        else:
            print("Failed to call service")


def main():
    TOPIC = "nodar/should_record"
    print(f"{TOPIC}\n\n--------------------\n"
          "To stop or start recording, just enter 0 or 1, and press ENTER.\n"
          "--------------------\n")

    client_node = ClientNode("client_node")

    while rclpy.ok():
        try:
            val = int(input())
            if val in [0, 1]:
                print(f"Requesting {TOPIC} = {val}")
                client_node.send_request(TOPIC, bool(val))
            else:
                print("Could not parse your last input. Try entering 0 or 1.")
        except ValueError:
            print("Could not parse your last input. Try entering 0 or 1.")

    client_node.node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
