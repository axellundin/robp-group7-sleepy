from rclpy.node import Node
from geometry_msgs.msg import Polygon, Point32
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
def main():
    rclpy.init()
    node = Node("geofence_publisher")

    latching_qos = QoSProfile(depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publisher = node.create_publisher(Polygon, "geofence", latching_qos)

    polygon = Polygon()
    for p in [(0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0)]:
        x_value, y_value, z_value = p
        point = Point32()
        point.x = float(x_value)  
        point.y = float(y_value)
        point.z = float(z_value)
        polygon.points.append(point)
    
    publisher.publish(polygon)
    node.get_logger().info('Geofence published')
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
