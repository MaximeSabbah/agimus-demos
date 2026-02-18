import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Transform, TransformStamped, Vector3
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


class RTCosmikPoseDebugMarkers(Node):
    def __init__(self):
        super().__init__("rtcosmik_pose_debug_markers")
        self._pub = self.create_publisher(
            MarkerArray, "/rtcosmik/obstacle_debug_markers", 10
        )
        self._tf_pub = TransformBroadcaster(self)
        self._poses = {
            "right_upperarm": None,
            "right_lowerarm": None,
            "right_hand": None,
        }
        self._frame_id = "fer_link0"

        self.create_subscription(
            Pose,
            "/rtcosmik/collision_pose/right_upperarm",
            lambda msg: self._cb("right_upperarm", msg),
            10,
        )
        self.create_subscription(
            Pose,
            "/rtcosmik/collision_pose/right_lowerarm",
            lambda msg: self._cb("right_lowerarm", msg),
            10,
        )
        self.create_subscription(
            Pose,
            "/rtcosmik/collision_pose/right_hand",
            lambda msg: self._cb("right_hand", msg),
            10,
        )

        self.create_timer(0.05, self._publish_markers)

    def _cb(self, key: str, pose: Pose) -> None:
        self._poses[key] = pose

    def _publish_markers(self) -> None:
        scales = {
            "right_upperarm": (0.09, 0.09, 0.22),
            "right_lowerarm": (0.07, 0.07, 0.20),
            "right_hand": (0.06, 0.06, 0.12),
        }
        colors = {
            "right_upperarm": (1.0, 0.2, 0.2, 0.8),
            "right_lowerarm": (0.2, 1.0, 0.2, 0.8),
            "right_hand": (0.2, 0.4, 1.0, 0.8),
        }
        keys = ["right_upperarm", "right_lowerarm", "right_hand"]
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()
        tf_links = {
            "right_upperarm": "obstacle_0",
            "right_lowerarm": "obstacle_1",
            "right_hand": "obstacle_2",
        }
        tf_msgs = []
        for i, key in enumerate(keys):
            pose = self._poses[key]
            if pose is None:
                continue
            sx, sy, sz = scales[key]
            r, g, b, a = colors[key]
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = self._frame_id
            m.ns = "agimus_rtcosmik_pose_debug"
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose = pose
            m.scale.x = sx
            m.scale.y = sy
            m.scale.z = sz
            m.color.r = r
            m.color.g = g
            m.color.b = b
            m.color.a = a
            marker_array.markers.append(m)

            # Drive environment link frames so RobotModel(/environment_description)
            # can follow the same obstacle poses.
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self._frame_id
            t.child_frame_id = tf_links[key]
            t.transform = Transform(
                translation=Vector3(
                    x=pose.position.x, y=pose.position.y, z=pose.position.z
                ),
                rotation=pose.orientation,
            )
            tf_msgs.append(t)

        self._pub.publish(marker_array)
        if tf_msgs:
            self._tf_pub.sendTransform(tf_msgs)


def main(args=None):
    rclpy.init(args=args)
    node = RTCosmikPoseDebugMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
