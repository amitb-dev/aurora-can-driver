import rclpy
from rclpy.node import Node
import can
# Importing the constants we created earlier
from canbridge.aurora_message import AuroraCANIDs

class VehicleController(Node):
    def __init__(self):
        super().__init__('vehicle_controller')
        
        # 1. Initialize the CAN Bus connection
        try:
            self.bus = can.interface.Bus(channel='vcan0', interface='socketcan')
            self.get_logger().info('Connected to CAN bus: vcan0')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to CAN: {str(e)}')
            return

        # 2. Create a timer that runs every 0.01 seconds (100Hz) to check for messages
        self.create_timer(0.01, self.can_receive_callback)
        self.get_logger().info('Vehicle Controller Node is now listening...')

    def can_receive_callback(self):
        """
        Check for new CAN messages without blocking the node.
        """
        msg = self.bus.recv(timeout=0) # timeout=0 means "don't wait, just check and return"
        
        if msg is not None:
            # Check if this is the Battery Status message (BMS)
            if msg.arbitration_id == AuroraCANIDs.BMS_BATTERY_STATUS:
                # Based on protocol: Byte 6-7 (last 2 bytes) are SOC * 10
                # Let's start simple: just log that we got it
                soc_raw = int.from_bytes(msg.data[6:8], byteorder="big")
                soc_percent = soc_raw / 10.0
                self.get_logger().info(f'BMS Update: Battery is at {soc_percent}%')

def main(args=None):
    rclpy.init(args=args)
    node = VehicleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt received. Shutting down...')
    finally:
        if hasattr(node, 'bus'):
            node.bus.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()