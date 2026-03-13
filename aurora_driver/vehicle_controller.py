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

        self.get_logger().info('Vehicle Controller Node is now listening...')

        # 2. Create a timer that runs every 0.01 seconds (100Hz) to check for messages
        self.create_timer(0.01, self.can_receive_callback)

        # 3. Heartbeat timer: runs at 20Hz (0.05s) to keep the vehicle active
        self.create_timer(0.05, self.publish_heartbeat)

        # Startup Sequence: Step 1 (Handbrake) and Step 2 (Ignition)
        self.release_handbrake()
        self.send_ignition_on()

        # Schedule gear change to Drive after 2 seconds to allow engine startup
        self.create_timer(2.0, self.send_gear_drive)

        # Target value for gas/brake (Step 7: -300 for gas)
        self.target_gas_brake = -300
        self.current_speed = 0.0

        # Step 8 & 9: After 7s (2s startup + 5s driving), start braking
        self.create_timer(7.0, self.start_braking)
        # Step 10: After 9s (7s + 2s braking), shutdown
        self.create_timer(9.0, self.stop_and_shutdown)

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
        data = [0x00] * 8
        
        # Use target_gas_brake instead of hardcoded value
        throttle_bytes = self.target_gas_brake.to_bytes(2, byteorder='big', signed=True)
        
        data[1] = throttle_bytes[0]
        data[2] = throttle_bytes[1]
        data[7] = 0x00 

        msg = can.Message(arbitration_id=AuroraCANIDs.HEARTBEAT, data=data, is_extended_id=False)
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

    def start_braking(self):
        # Step 8: Log current speed from feedback
        self.get_logger().info(f'Step 8: 5 seconds of driving reached. Current speed: {self.current_speed} km/h')
        
        # Step 9: Apply brake (+500) for 2 seconds
        self.target_gas_brake = 500
        self.get_logger().info('Step 9: Applying brakes (+500)')

    def stop_and_shutdown(self):
        # Step 10: Engage handbrake and turn engine off
        self.get_logger().info('Step 10: Stopping - Engaging handbrake and ENGINE_OFF')
        
        # Send Handbrake Engage
        hb_msg = can.Message(arbitration_id=AuroraCANIDs.COMMAND, data=[0x00, AuroraCANCommand.HANDBRAKE_ENGAGE] + [0]*6, is_extended_id=False)
        self.bus.send(hb_msg)
        
        # Send Engine Off
        off_msg = can.Message(arbitration_id=AuroraCANIDs.COMMAND, data=[0x00, AuroraCANCommand.ENGINE_OFF] + [0]*6, is_extended_id=False)
        self.bus.send(off_msg)

    def can_receive_callback(self):
        """
        Non-blocking check for incoming CAN messages.
        Processes BMS and Vehicle Status messages based on the protocol.
        """
        msg = self.bus.recv(timeout=0)
        
        if msg is not None:
            # Bonus 1: Process Battery Status (BMS) with Extended ID 0x18904010
            if msg.arbitration_id == 0x18904010:
                # Byte 6-7: State of Charge (SOC) scaled by 10
                soc_raw = int.from_bytes(msg.data[6:8], byteorder="big")
                soc_percent = soc_raw / 10.0
                self.get_logger().info(f'BMS Update: Battery is at {soc_percent}%')

            # Process Vehicle Status for speed feedback (Step 8)
            elif msg.arbitration_id == AuroraCANIDs.VEHICLE_STATUS:
                # Byte 0-1: Vehicle speed (int16, big-endian)
                speed_raw = int.from_bytes(msg.data[0:2], byteorder="big", signed=True)
                self.current_speed = speed_raw / 10.0

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