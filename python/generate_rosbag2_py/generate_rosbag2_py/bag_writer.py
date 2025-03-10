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
        serialized_data = serialize_message(msg)
        self.writer.write(topic_name, serialized_data, Clock().now().to_msg())
