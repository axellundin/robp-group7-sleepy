from core_interfaces.srv import GeofenceCompliance
from geometry_msgs.msg import Polygon, Point32
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

class GeofenceComplianceService(Node):

    def __init__(self):
        super().__init__('geofence_compliance')
        self.srv = self.create_service(GeofenceCompliance, 'geofence_compliance', self.geofence_compliance_callback)
        
        # Create QoS profile matching the publisher
        latching_qos = QoSProfile(depth=1,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        
        self.geofence_subscriber = self.create_subscription(
            Polygon, 
            'geofence', 
            self.geofence_callback, 
            latching_qos)  # Use matching QoS profile
        self.geofence = None
    
    def geofence_callback(self, polygon):
        self.get_logger().info('Geofence received: %s' % polygon)
        self.geofence = polygon
    
    def cross(self, a: Point32, b: Point32):
        return a.x * b.y - a.y * b.x
    
    def is_to_the_left(self, p1: Point32, p2: Point32, x: Point32):
        x_p1 = Point32()
        x_p1.x = x.x - p1.x
        x_p1.y = x.y - p1.y
        p2_p1 = Point32()
        p2_p1.x = p2.x - p1.x
        p2_p1.y = p2.y - p1.y
        return self.cross(x_p1, p2_p1) < 0

    def check_geofence_compliance(self, pose):
        if self.geofence is None:
            self.get_logger().info('Geofence not received yet')
            return True
        
        # Check if the pose is inside the geofence
        for i in range(len(self.geofence.points)):
            p1 = self.geofence.points[i]
            p2 = self.geofence.points[(i + 1) % len(self.geofence.points)]
            if not self.is_to_the_left(p1, p2, pose.position):
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