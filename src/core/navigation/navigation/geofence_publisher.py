from rclpy.node import Node
from geometry_msgs.msg import PolygonStamped, Polygon, Point32
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
import rclpy.time
import os
import numpy as np

def get_fence(workspace_file)-> Polygon:
    read_start_row = 0 
    file_path = os.path.expanduser(workspace_file)
    while True:
        try:
            data = np.loadtxt(file_path, delimiter="\t", skiprows=read_start_row)
            break
        except ValueError:
            read_start_row += 1

    return data

def main():
    rclpy.init()
    node = Node("geofence_publisher")
    workspace_file = "~/robp-group7-sleepy/src/core/navigation/navigation/maps/workspace_2.tsv"


    latching_qos = QoSProfile(depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
    
    publisher = node.create_publisher(PolygonStamped, 
                                      "geofence", 
                                      latching_qos)
    
    polygon = PolygonStamped()
    polygon.header.stamp = node.get_clock().now().to_msg()
    polygon.header.frame_id = 'map'
    data = get_fence(workspace_file)
    data = [
        [10,10], 
        [-10,10],
        [-10,-10],
        [10,-10]
        ]
    for p in data:
        x_value, y_value,z_value = p[0]/100, p[1]/100, 0
        point = Point32()
        point.x = float(x_value)  
        point.y = float(y_value)
        point.z = float(z_value)
        polygon.polygon.points.append(point)
    
    publisher.publish(polygon)
    node.get_logger().info('Geofence published')
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
