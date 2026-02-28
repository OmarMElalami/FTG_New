"""Substitutes every LiDAR measurement with a circle obstacle.

Subscribes to /scan (LaserScan) and publishes /obstacles (ObstaclesStamped).
"""

import sys
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from obstacle_msgs.msg import CircleObstacle, Obstacles, ObstaclesStamped
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point


def polar_to_point(distance, angle):
    """Converts polar coordinates of a 2D point into cartesian coordinates.

    Arguments:
    distance -- distance to the point [m]
    angle -- angle between x_axis and point [rad]

    Returns:
    point -- point in cartesian coordinates, geometry_msgs.msg/Point
    """
    point = Point()

    point.x = math.cos(angle) * distance
    point.y = math.sin(angle) * distance
    point.z = 0.0

    return point


class ObstacleSubstitutionNode(Node):

    def __init__(self):
        super().__init__(node_name='obstacle_substitution')

        # QoS profile for sensor data (BestEffort matches real LiDAR hardware)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # publishers
        self.pub = self.create_publisher(msg_type=ObstaclesStamped, topic='/obstacles', qos_profile=1)

        # subscriptions
        self.scan_subscription = self.create_subscription(
            msg_type=LaserScan,
            topic='/scan',
            callback=self.callback_scan,
            qos_profile=sensor_qos,
        )

        self.get_logger().info('ObstacleSubstitutionNode ready — subscribing to /scan, publishing /obstacles')

    def callback_scan(self, data: LaserScan):
        """Convert each LiDAR measurement into a CircleObstacle and publish."""
        msg = ObstaclesStamped()

        msg.header = data.header

        msg.obstacles = Obstacles()

        msg.obstacles.circles = []

        angle = data.angle_min - data.angle_increment

        for m in data.ranges:
            angle += data.angle_increment

            if math.isnan(m) or math.isinf(m) or m > data.range_max or m < data.range_min:
                continue

            obs = CircleObstacle()

            obs.center = polar_to_point(m, angle)
            obs.radius = 0.01

            msg.obstacles.circles.append(obs)

        self.pub.publish(msg)


# NOTE: See more info the the `main()` in the project README.md (the top-level one)
#       (section [Python entrypoints `main()` inconsistencies])
def main(args=None):
    if args is None:
        args = sys.argv

    rclpy.init(args=args)

    # print(f'main args = {args}')

    node = None

    try:
        node = ObstacleSubstitutionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    if node is not None:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        #  when the garbage collector destroys the node object)
        node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
