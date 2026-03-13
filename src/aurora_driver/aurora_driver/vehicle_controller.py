import rclpy
from rclpy.node import Node
import can
# Importing the constants we created earlier
from canbridge.aurora_message import AuroraCANIDs, AuroraCANCommand

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

        # 3. Heartbeat timer: runs at 20Hz (0.05s) to keep the vehicle active
        self.create_timer(0.05, self.publish_heartbeat)

        # Startup Sequence: Step 1 (Handbrake) and Step 2 (Ignition)
        self.release_handbrake()
        self.send_ignition_on()

        # Schedule gear change to Drive after 2 seconds to allow engine startup
        self.create_timer(2.0, self.send_gear_drive)

    def release_handbrake(self):
        # Prepare 8-byte command for HANDBRAKE_DISENGAGE
        data = [0x00] * 8
        data[1] = AuroraCANCommand.HANDBRAKE_DISENGAGE
        
        msg = can.Message(
            arbitration_id=AuroraCANIDs.COMMAND,
            data=data,
            is_extended_id=False
        )
        
        try:
            self.bus.send(msg)
            self.get_logger().info('Handbrake release command sent')
        except can.CanError as e:
            self.get_logger().error(f'Failed to release handbrake: {e}')

    def send_ignition_on(self):
        # Prepare 8-byte command message
        data = [0x00] * 8
        data[1] = AuroraCANCommand.ENGINE_IGNITION_ON # ENGINE_IGNITION_ON code
        
        msg = can.Message(
            arbitration_id=AuroraCANIDs.COMMAND,
            data=data,
            is_extended_id=False
        )
        
        try:
            self.bus.send(msg)
            self.get_logger().info('Ignition ON command sent')
        except can.CanError as e:
            self.get_logger().error(f'Failed to send ignition command: {e}')

    def publish_heartbeat(self):
        # Co 1-2: gas_brake (int16, big-endian)
        # Byte 3-4: steering (int16, big-endian)
        # Byte 7: estop (0x00 for normal operation)nstruct 8-byte heartbeat message
        # Byte
        data = [0x00] * 8
        
        # For now, we send 0 for all controls to keep the vehicle in standby
        # Byte 1-2 (gas_brake) and 3-4 (steering) remain 0x00
        data[7] = 0x00 # Normal operation (no e-stop)

        msg = can.Message(
            arbitration_id=AuroraCANIDs.HEARTBEAT,
            data=data,
            is_extended_id=False
        )

        try:
            self.bus.send(msg)
        except can.CanError:
            self.get_logger().error('Failed to send heartbeat')

    def send_gear_drive(self):
        # Prepare 8-byte command for GEAR_DRIVE
        data = [0x00] * 8
        data[1] = AuroraCANCommand.GEAR_DRIVE
        
        msg = can.Message(
            arbitration_id=AuroraCANIDs.COMMAND,
            data=data,
            is_extended_id=False
        )
        
        try:
            self.bus.send(msg)
            self.get_logger().info('Gear shifted to DRIVE')

        except can.CanError as e:
            self.get_logger().error(f'Failed to send gear command: {e}')

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