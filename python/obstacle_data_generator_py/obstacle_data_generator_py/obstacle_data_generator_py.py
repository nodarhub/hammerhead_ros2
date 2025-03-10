from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy

from hammerhead_msgs.msg import ObstacleData


def write_data(filename, obstacle_data):
    with open(filename, 'w') as out:
        out.write("x1,z1,x2,z2,x3,z3,x4,z4,vx,vz\n")
        for obstacle in obstacle_data.obstacles:
            for p in obstacle.bounding_box:
                out.write(f"{p.x:.6f},{p.z:.6f},")
            out.write(f"{obstacle.velocity.x:.6f},{obstacle.velocity.z:.6f}\n")
        out.write("\n")


class ObstacleDataGeneratorNode(Node):
    def __init__(self, output_dir):
        super().__init__('obstacle_data_generator_node')
        self.logger = self.get_logger()
        self.output_dir = output_dir
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.subscription = self.create_subscription(
            ObstacleData,
            'nodar/obstacle_data',
            self.on_new_message,
            self.qos_profile
        )
        self.frame_index = 0

    def on_new_message(self, msg):
        self.frame_index += 1
        filename = self.output_dir / f"{self.frame_index}.txt"

        self.logger.info(
            f"onNewMessage: Received {len(msg.obstacles)} obstacles at "
            f"{msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9:.9f}. Writing {filename}"
        )

        write_data(filename, msg)


def main():
    print("Starting obstacle_data_generator_node")
    rclpy.init()

    exec = rclpy.executors.MultiThreadedExecutor()

    HERE = Path(".").parent.absolute()
    output_dir = HERE / "obstacle_data"
    output_dir.mkdir(parents=True, exist_ok=True)

    # Put a .gitignore in that directory
    with open(output_dir / ".gitignore", "w") as gitignore:
        gitignore.write("*\n")
        gitignore.write("!.gitignore\n")

    obstacle_data_generator_node = ObstacleDataGeneratorNode(output_dir)
    exec.add_node(obstacle_data_generator_node)

    exec.spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
