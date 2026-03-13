import rclpy
from rclpy.node import Node
import can
from canbridge.aurora_message import AuroraCANIDs, AuroraCANCommand

class VehicleController(Node):
    def __init__(self):
        super().__init__('vehicle_controller')
        
        # Initialize SocketCAN interface
        try:
            self.bus = can.interface.Bus(channel='vcan0', interface='socketcan')
            self.get_logger().info('Connected to CAN bus: vcan0')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to CAN: {str(e)}')
            return
        
        # Node parameter configuration
        self.declare_parameter('drive_throttle', -300)
        self.declare_parameter('drive_duration_sec', 5.0)
        
        self.throttle_val = self.get_parameter('drive_throttle').get_parameter_value().integer_value
        self.duration_val = self.get_parameter('drive_duration_sec').get_parameter_value().double_value

        self.get_logger().info('Vehicle Controller Node initialized with parameters.')

        # Internal State
        self.current_speed = 0.0
        self.target_gas_brake = 0.0 # Neutral until engine is ready
        self.engine_started = False
        self.handbrake_released = False

        # 1. Async CAN receiver loop (100Hz)
        self.timer_receive = self.create_timer(0.01, self.can_receive_callback)

        # 2. Heartbeat publisher (20Hz)
        self.create_timer(0.05, self.publish_heartbeat)

        # 3. Startup Sequence Retry (1Hz) - This is the "Brain"
        # It will call startup_sequence_retry every second until the vehicle responds
        self.timer_startup = self.create_timer(1.0, self.startup_sequence_retry)

        # 4. Mission timers (Initialize as None, will be created once vehicle is READY)
        self.timer_drive = None
        self.timer_brake = None
        self.timer_shutdown = None

    def startup_sequence_retry(self):
        """
        Periodically sends Ignition and Handbrake commands until the vehicle is ready.
        """
        if not self.engine_started or not self.handbrake_released:
            self.get_logger().info('Attempting to wake up vehicle...')
            self.release_handbrake()
            self.send_ignition_on()
        else:
            # Vehicle is ready! Stop retrying and start the drive sequence
            self.get_logger().info('Vehicle is READY. Starting mission timers.')
            self.timer_startup.cancel()
            
            # Start the 2s delay before shifting to DRIVE
            self.timer_drive = self.create_timer(2.0, self.send_gear_drive)
            
            # Schedule braking
            braking_start_time = 2.0 + self.duration_val
            self.timer_brake = self.create_timer(braking_start_time, self.start_braking)
            
            # Schedule shutdown
            shutdown_time = braking_start_time + 2.0
            self.timer_shutdown = self.create_timer(shutdown_time, self.stop_and_shutdown)
            
            # Set drive throttle now that engine is starting
            self.target_gas_brake = self.throttle_val

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

    def publish_heartbeat(self, estop_active=False):
        """
        Sends a heartbeat message (ID 0x41) according to section 4.3.
        """
        # Initialize 8 bytes with 0x00
        data = [0x00] * 8
        
        # Byte 0: Reserved (Always 0x00) - Already set by initialization

        # Pack gas/brake command (int16 BE)
        if estop_active:
            self.target_gas_brake = 0 # Force neutral on emergency
            
        throttle_bytes = int(self.target_gas_brake).to_bytes(2, byteorder='big', signed=True)
        data[1] = throttle_bytes[0] # MSB
        data[2] = throttle_bytes[1] # LSB

        # Bytes 3-4: Steering (int16 big-endian) - Set to 0 for now
        data[3] = 0x00
        data[4] = 0x00

        # Bytes 5-6: Reserved (0x00) - Already set

        # Emergency stop status flag
        data[7] = 0x01 if estop_active else 0x00

        msg = can.Message(arbitration_id=0x41, data=data, is_extended_id=False)
        try:
            self.bus.send(msg)
        except can.CanError:
            self.get_logger().error('CAN Bus error: Failed to send heartbeat')

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
            self.timer_drive.cancel() # Stop the timer after first success
        except can.CanError as e:
            self.get_logger().error(f'Failed to send gear command: {e}')

    def start_braking(self):
        self.get_logger().info(f'Target duration reached ({self.duration_val}s). Feedback velocity: {self.current_speed} km/h')        

        # Apply service brake
        self.target_gas_brake = 500
        self.get_logger().info('Applying service brake (+500)')

    def stop_and_shutdown(self):
        """
        Final mission stage. Secures the vehicle and stops all timers.
        """
        self.get_logger().info('Mission sequence finalized. Initiating secure shutdown.')
        
        # Cancel all timers
        self.timer_drive.cancel()
        self.timer_brake.cancel()
        self.timer_shutdown.cancel()
        self.timer_receive.cancel()
        
        # Secure vehicle state
        hb_msg = can.Message(arbitration_id=AuroraCANIDs.COMMAND, 
                             data=[0x00, AuroraCANCommand.HANDBRAKE_ENGAGE] + [0]*6, 
                             is_extended_id=False)
        self.bus.send(hb_msg)
        
        off_msg = can.Message(arbitration_id=AuroraCANIDs.COMMAND, 
                              data=[0x00, AuroraCANCommand.ENGINE_OFF] + [0]*6, 
                              is_extended_id=False)
        self.bus.send(off_msg)

        self.get_logger().info('Vehicle is secured. Shutdown sequence finished.')

    def can_receive_callback(self):
        while True:
            try:
                msg = self.bus.recv(timeout=0)
                if msg is None: break

                # Parse BMS
                if msg.arbitration_id == 0x18904010:
                    soc_percent = int.from_bytes(msg.data[6:8], byteorder="big") / 10.0
                    self.get_logger().info(f'BMS Update: Battery is at {soc_percent}%')

                # Parse Vehicle Status (ID 0x39)
                elif msg.arbitration_id == AuroraCANIDs.VEHICLE_STATUS:
                    # Check if engine is running (Byte 0, bit 7)
                    self.engine_started = bool(msg.data[0] & (1 << 7))
                    # Speed feedback (Byte 2)
                    self.current_speed = float(msg.data[2])
                
                # Parse Handbrake Status (ID 0x38)
                elif msg.arbitration_id == AuroraCANIDs.HANDBRAKE_STATUS:
                    # Byte 1 is 0x01 for engaged, 0x00 for released
                    self.handbrake_released = (msg.data[1] == 0x00)
            
            except Exception as e:
                self.get_logger().error(f'CAN error: {e}')
                break

def main(args=None):
    rclpy.init(args=args)
    node = VehicleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt received.')
    finally:
        # Resource cleanup
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()