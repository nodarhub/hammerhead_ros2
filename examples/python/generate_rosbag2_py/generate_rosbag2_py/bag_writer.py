from rclpy.clock import Clock
from rclpy.serialization import serialize_message
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata


class BagWriter:
    def __init__(self, output_dir, topics):
        self.output_dir = output_dir
        self.topics = topics

        storage_options = StorageOptions(
            uri=str(output_dir),
            storage_id='sqlite3'
        )
        converter_options = ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )

        self.writer = SequentialWriter()
        self.writer.open(storage_options, converter_options)

        for topic in topics:
            topic_metadata = TopicMetadata(
                name=topic['name'],
                type=topic['type'],
                serialization_format='cdr'
            )
            self.writer.create_topic(topic_metadata)

    def write(self, topic_name, msg):
        try:
            sec = msg.header.stamp.sec
            nanosec = msg.header.stamp.nanosec
            stamp_ns = sec * 1000000000 + nanosec
        except AttributeError:
            # If message has no header or header.stamp, fall back to system time
            print("Attribute error!")
            stamp_ns = Clock().now().nanoseconds

        # If the message timestamp is zero (missing), fall back to current system time
        if stamp_ns == 0:
            stamp_ns = Clock().now().nanoseconds
        self.writer.write(topic_name, serialize_message(msg), stamp_ns)
