from core_interfaces.srv import GeofenceCompliance
from std_msgs.msg import Header
from navigation.polygon import LocalPolygon
from geometry_msgs.msg import Polygon, Point32
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
import numpy as np

class GeofenceComplianceService(Node):

    def __init__(self):
        print("Initializing GeofenceComplianceService")
        super().__init__('geofence_compliance')
        self.srv = self.create_service(GeofenceCompliance, 'geofence_compliance', self.geofence_compliance_callback)
        
        # Create QoS profile matching the publisher
        latching_qos = QoSProfile(depth=1,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        
        self.robot_convex_hull = [ 
            np.array([0.1, 0.16]),
            np.array([-0.3, 0.16]),
            np.array([-0.3, -0.16]),
            np.array([0.1, -0.16]),
        ]

        self.geofence_subscriber = self.create_subscription(
            Polygon, 
            'geofence', 
            self.geofence_callback,
            latching_qos
            )  # Use matching QoS profile
        
        self.geofence = None

    def geofence_callback(self, polygon):
        self.get_logger().info('Geofence received: %s' % polygon)
        self.geofence = polygon
        verticies = [np.array([point.x, point.y]) for point in self.geofence.points]
        self.polygon = LocalPolygon(verticies)

    def check_geofence_compliance(self, pose):
        if self.geofence is None:
            self.get_logger().info('Geofence not received yet')
            return True
        
        # Check if the pose is inside the geofence
        position = np.array([pose.position.x, pose.position.y])
        if not self.polygon.is_internal(position):
            return False
        # rotate robot convex hull by pose orientation 
        rotation_matrix = np.array([[np.cos(pose.orientation.z), -np.sin(pose.orientation.z)],
                                    [np.sin(pose.orientation.z), np.cos(pose.orientation.z)]])
        rotatex_convex_hull = [rotation_matrix @ vertex for vertex in self.robot_convex_hull]
        for i in range(len(rotatex_convex_hull)):
            start = position + rotatex_convex_hull[i]
            end = position + rotatex_convex_hull[(i + 1) % len(rotatex_convex_hull)]
            if self.polygon.segment_intersects_polygon(start, end):
                return False
        return True

    def geofence_compliance_callback(self, request, response):
        response.success = self.check_geofence_compliance(request.pose.pose)
        return response

def main(args=None):
    rclpy.init(args=args)

    geofence_compliance_service = GeofenceComplianceService()

    rclpy.spin(geofence_compliance_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()