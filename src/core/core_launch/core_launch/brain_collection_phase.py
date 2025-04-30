import rclpy
from rclpy.node import Node
from core_interfaces.srv import MoveToObject, Pickup, MoveTo, Place
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time
from geometry_msgs.msg import PoseStamped

class BrainCollectionPhase(Node):
    def __init__(self):
        super().__init__('brain_collection_phase')
        
        # Create callback group for the clients
        group1 = ReentrantCallbackGroup()
        group2 = ReentrantCallbackGroup()
        
        # Create client for MoveToObject service
        self.move_client = self.create_client(
            MoveToObject,
            'move_to_object',
            callback_group=group1
        )
        while not self.move_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('MoveToObject service not available, waiting...')
            
        # Create client for Pickup service
        self.pickup_client = self.create_client(
            Pickup,
            'pickup',
            callback_group=group2
        )
        while not self.pickup_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Pickup service not available, waiting...')
        
        self.place_client = self.create_client(
            Place,
            'place',
            callback_group=group2
        )
        while not self.place_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Place service not available, waiting...')
            
        self.get_logger().info('Brain Collection Phase node initialized')
        self.get_logger().info('Running MS2 phase')
        self.run_MS2_phase()


    def move_to_object(self):
        # Call the MoveToObject service to move to next object
        self.get_logger().info('Moving to object')
        request = MoveToObject.Request()
        request.target = "OBJECT"
        future = self.move_client.call_async(request)
        future.add_done_callback(self.pickup_object)

    def pickup_object(self, future):

        if not future.result().success:
            self.get_logger().error('Failed to move to object')
            return 
        
        # Call the Pickup service
        request = Pickup.Request()
        future = self.pickup_client.call_async(request)
        future.add_done_callback(self.move_back_from_pickup)

    def move_back(self, future, cb):
        
        # Call the MoveToObject service to move back to workspace
        request = MoveToObject.Request()
        request.target = "Go BACK"
        future = self.move_client.call_async(request)
        future.add_done_callback(cb)

    def move_back_from_pickup(self, future):
        if not future.result().success:
            self.get_logger().error('Failed to pickup object')
            self.move_to_object()
            return
        
        self.move_back(future, self.move_to_box)

    def move_back_from_place(self, future):

        self.move_back(future, self.move_back_callback)

    def move_back_callback(self, future):
        if not future.result().success:
            self.get_logger().error('Failed to move back to workspace')
            return
        
        if not future.result().finished:
            self.move_to_object()
        else:
            self.print_done(future)

    def place_object(self, future): 
        if not future.result().success:
            self.get_logger().error('Failed to move to box')
            return 
        
        # Call the Place service
        request = Place.Request()
        future = self.place_client.call_async(request)
        future.add_done_callback(self.move_back_from_place)
        
        self.get_logger().info('Placed object')

    def move_to_box(self, future):
        if not future.result().success:
            self.get_logger().error('Failed to move back to workspace')
            return
        
        # Call the MoveToObject service to move to box
        request = MoveToObject.Request()
        request.target = "BOX"
        future = self.move_client.call_async(request)
        future.add_done_callback(self.place_object)

    def print_done(self, future):
        if future.result().success:
            self.get_logger().info('Done with MS2 phase')
        else:
            self.get_logger().error('Failed to move to box')

    def run_MS2_phase(self):
        self.get_logger().info('Want to move to object')

        self.move_to_object()
        
def main(args=None):
    rclpy.init(args=args)
    
    node = BrainCollectionPhase()
    
    try:
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
