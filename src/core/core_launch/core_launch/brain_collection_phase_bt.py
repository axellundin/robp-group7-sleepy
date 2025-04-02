import py_trees, py_trees_ros
from core_interfaces.srv import MoveToObject, Pickup
import rclpy
import sys

class MoveToObject(py_trees.behaviour.Behaviour):
    def __init__(self, name, target_type="OBJECT"):
        super().__init__(name)
        self.node = None
        self.move_to_object_client = None
        self.target_type = target_type
        self.future = None
        self.logger = None

    def setup(self, **kwargs):
        try:
            self.node = kwargs['node']  # Get the ROS node from kwargs
            self.move_to_object_client = self.node.create_client(MoveToObject,"move_to_object")

            self.logger = self.node.get_logger()
            self.logger.info(f"MoveToObject setup completed")  # Use py_trees logger
            return True
        except KeyError as e:
            self.logger.error(f"Missing required kwargs: {e}")
            return False
        
    def update(self):
        """
        Called every tick while this behavior is running
        Returns: SUCCESS, FAILURE, or RUNNING
        """
        # Check if we're waiting for a response
        if self.future is not None:
            if not self.future.done():
                # Still waiting for service response
                return py_trees.Status.RUNNING
                
            try:
                response = self.future.result()
                self.future = None  # Reset for next time
                
                if response.success:
                    self.logger.info(f"Move to {self.target_type} succeeded: {response.message}")
                    return py_trees.Status.SUCCESS
                else:
                    self.logger.error(f"Move to {self.target_type} failed: {response.message}")
                    return py_trees.Status.FAILURE
                    
            except Exception as e:
                self.logger.error(f"Service call failed: {str(e)}")
                self.future = None
                return py_trees.Status.FAILURE

        # No active future, send new request
        if not self.move_to_object_client.wait_for_service(timeout_sec=1.0):
            self.logger.error("Move to object service not available")
            return py_trees.Status.FAILURE

        try:
            request = MoveToObject.Request()
            request.target = self.target_type
            self.future = self.move_to_object_client.call_async(request)
            return py_trees.Status.RUNNING
            
        except Exception as e:
            self.logger.error(f"Failed to send service request: {str(e)}")
            return py_trees.Status.FAILURE

    def terminate(self, new_status):
        """
        Cleanup when the behavior finishes
        """
        self.logger.debug(f"{self.name} terminated with status {new_status}")
        self.future = None  # Clear any pending future

class PickupObject(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)  
        self.node = None
        self.pickup_object_client = None
        self.future = None
        self.logger = None

    def setup(self, **kwargs):
        try: 
            self.node = kwargs['node']
            self.pickup_object_client = self.node.create_client(Pickup,"pickup")
            self.logger = self.node.get_logger()
        except KeyError as e:
            self.logger.error(f"Missing required kwargs: {e}")
            return False
        
    def update(self):
        """
        Called every tick while this behavior is running
        Returns: SUCCESS, FAILURE, or RUNNING
        """
        if self.future is not None:
            if not self.future.done():
                return py_trees.Status.RUNNING
            
            try:
                response = self.future.result()
                self.future = None
                if response.success:
                    self.logger.info(f"Pickup object succeeded: {response.message}")
                    return py_trees.Status.SUCCESS
                else:
                    self.logger.error(f"Pickup object failed: {response.message}")
                    return py_trees.Status.FAILURE
                    
            except Exception as e:
                self.logger.error(f"Service call failed: {str(e)}")
                self.future = None
                return py_trees.Status.FAILURE

        if not self.pickup_object_client.wait_for_service(timeout_sec=1.0):
            self.logger.error("Pickup object service not available")
            return py_trees.Status.FAILURE
        
        request = Pickup.Request()  
        self.future = self.pickup_object_client.call_async(request)
        return py_trees.Status.RUNNING
    
    def terminate(self, new_status):
        """
        Cleanup when the behavior finishes
        """
        self.logger.debug(f"{self.name} terminated with status {new_status}")
        self.future = None 
    


def get_root():
    # Create the root node 
    root = py_trees.composites.Parallel(
        name="Collection",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(
            synchronise=False
        )
    )
    priorities = py_trees.composites.Selector(name="Tasks", memory=False)

    # Pick up object 
    collect_object = py_trees.composites.Sequence(
        name="Collect object",
        memory=False,
    ) 
    # Create the node to move to an object
    move_to_object = MoveToObject("Move to object", "OBJECT")
    collect_object.add_child(move_to_object) 
    pickup_object = PickupObject("Pickup object") 
    collect_object.add_child(pickup_object) 
    priorities.add_child(collect_object)
    root.add_child(priorities)

    return root 

def create_bt():
    tree = py_trees_ros.trees.BehaviourTree(root=get_root(), unicode_tree_debug=True) 
    try:
        tree.setup(timeout=15)
    except py_trees_ros.exceptions.TimedOutError as e:
        tree.node.get_logger().error(f"failed to setup the tree, aborting [{str(e)}]")
        tree.shutdown()
        rclpy.shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        # not a warning, nor error, usually a user-initiated shutdown
        tree.node.get_logger().error("tree setup interrupted")
        tree.shutdown()
        rclpy.shutdown()
        sys.exit(1)

    tree.tick_tock(period_ms=1000.0)


def main(args=None):
    rclpy.init(args=args)
    tree = create_bt()
    rclpy.spin(tree.node)
    tree.node.destroy_node()
    rclpy.shutdown()