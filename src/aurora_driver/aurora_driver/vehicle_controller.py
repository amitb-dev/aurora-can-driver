import rclpy
from rclpy.node import Node

class VehicleController(Node):
    def __init__(self):
        super().__init__('vehicle_controller')
        self.get_logger().info('Vehicle Controller Node has been started')

def main(args=None):
    rclpy.init(args=args)
    node = VehicleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()