import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Transform, TransformStamped, Vector3
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import MarkerArray


class RTCosmikObstaclePoseAdapter(Node):
    """Bridge RT-COSMIK collision markers to Agimus moving-geometry Pose topics."""

    def __init__(self):
        super().__init__("rtcosmik_obstacle_pose_adapter")

        self.declare_parameter("input_topic", "/rtcosmik/collision_markers")
        self.declare_parameter("output_topics", ["obstacle_0", "obstacle_1", "obstacle_2"])
        self.declare_parameter("marker_ids", [0, 1, 2])
        self.declare_parameter("marker_namespace", "rtcosmik_collision")
        self.declare_parameter("publish_debug_tf", True)
        self.declare_parameter("debug_tf_suffix", "")
        self.declare_parameter("force_frame_id", "")
        self.declare_parameter("fallback_frame_id", "fer_link0")

        self._input_topic = str(self.get_parameter("input_topic").value)
        self._output_topics = list(self.get_parameter("output_topics").value)
        self._marker_ids = [int(x) for x in self.get_parameter("marker_ids").value]
        self._marker_namespace = str(self.get_parameter("marker_namespace").value)
        self._publish_debug_tf = bool(self.get_parameter("publish_debug_tf").value)
        self._debug_tf_suffix = str(self.get_parameter("debug_tf_suffix").value)
        self._force_frame_id = str(self.get_parameter("force_frame_id").value)
        self._fallback_frame_id = str(self.get_parameter("fallback_frame_id").value)

        if len(self._output_topics) != len(self._marker_ids):
            raise ValueError(
                "output_topics and marker_ids must have the same length."
            )

        self._publishers = [
            self.create_publisher(Pose, topic, 10) for topic in self._output_topics
        ]
        self._last_poses = {mid: None for mid in self._marker_ids}
        self._last_frame_ids = {mid: self._fallback_frame_id for mid in self._marker_ids}

        self._tf_broadcaster = TransformBroadcaster(self) if self._publish_debug_tf else None
        self._subscriber = self.create_subscription(
            MarkerArray,
            self._input_topic,
            self._markers_cb,
            10,
        )

        self.get_logger().info(
            f"Adapter started. Reading '{self._input_topic}', publishing "
            f"{list(zip(self._marker_ids, self._output_topics))}."
        )

    def _markers_cb(self, msg: MarkerArray) -> None:
        marker_by_id = {}
        frame_by_id = {}

        for marker in msg.markers:
            if self._marker_namespace and marker.ns != self._marker_namespace:
                continue
            marker_by_id[marker.id] = marker
            if self._force_frame_id:
                frame_id = self._force_frame_id
            else:
                frame_id = marker.header.frame_id if marker.header.frame_id else self._fallback_frame_id
            frame_by_id[marker.id] = frame_id

        for marker_id, topic, publisher in zip(
            self._marker_ids, self._output_topics, self._publishers
        ):
            marker = marker_by_id.get(marker_id)
            if marker is not None:
                pose = marker.pose
                self._last_poses[marker_id] = pose
                self._last_frame_ids[marker_id] = frame_by_id[marker_id]
            else:
                pose = self._last_poses.get(marker_id)
                if pose is None:
                    continue

            publisher.publish(pose)

            if self._tf_broadcaster is not None:
                tf_child = topic[1:] if topic.startswith("/") else topic
                transform = TransformStamped()
                transform.header.stamp = self.get_clock().now().to_msg()
                transform.header.frame_id = self._last_frame_ids.get(
                    marker_id, self._fallback_frame_id
                )
                transform.child_frame_id = f"{tf_child}{self._debug_tf_suffix}"
                transform.transform = Transform(
                    translation=Vector3(
                        x=pose.position.x, y=pose.position.y, z=pose.position.z
                    ),
                    rotation=pose.orientation,
                )
                self._tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RTCosmikObstaclePoseAdapter()
        rclpy.spin(node)
    except (KeyboardInterrupt, ValueError):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
