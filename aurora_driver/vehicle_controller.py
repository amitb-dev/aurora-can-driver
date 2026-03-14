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
        self.current_rpm = 0
        self.target_gas_brake = 0.0 
        self.engine_started = False
        self.handbrake_released = False
        self.bms_log_counter = 0 # counter to reduce BMS log spam
        self.emergency_triggered = False

        # Async CAN receiver loop
        self.timer_receive = self.create_timer(0.01, self.can_receive_callback)

        # Heartbeat publisher (assigned to variable for cleanup)
        self.timer_heartbeat = self.create_timer(0.05, self.publish_heartbeat)

        # Startup Sequence Retry (1Hz)
        self.timer_startup = self.create_timer(1.0, self.startup_sequence_retry)

        # Mission timers handles (initialized as None)
        self.timer_drive = None
        self.timer_brake = None
        self.timer_shutdown = None

    def startup_sequence_retry(self):
        """
        Periodically sends Ignition and Handbrake commands until the vehicle is ready (RPM > 0).
        """
        # If mission already started, do nothing
        if self.timer_drive is not None:
            return

        if not self.engine_started or not self.handbrake_released:
            self.get_logger().info(f'Vehicle not ready (RPM: {self.current_rpm}, HB: {"Released" if self.handbrake_released else "Engaged"}). Retrying startup...')
            self.release_handbrake()
            self.send_ignition_on()
        else:
            self.get_logger().info('Vehicle is READY (RPM detected). Starting mission timers.')
            self.timer_startup.cancel()
            
            # Mission Sequence:
            # 1. Wait 2s before DRIVE
            self.timer_drive = self.create_timer(2.0, self.send_gear_drive)
            
            # 2. Apply throttle
            self.target_gas_brake = self.throttle_val

            # 3. Schedule braking
            braking_start_time = 2.0 + self.duration_val
            self.timer_brake = self.create_timer(braking_start_time, self.start_braking)
            
            # 4. Schedule final shutdown
            shutdown_time = braking_start_time + 2.0
            self.timer_shutdown = self.create_timer(shutdown_time, self.stop_and_shutdown)

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
        Sends a heartbeat message (ID 0x41). Can also be used with estop_active=True.
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
        except can.CanError as e:
            self.emergency_shutdown(f'Failed to send heartbeat: {e}')

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
        # Log speed and RPM as required by the protocol
        self.get_logger().info(
            f'Target duration reached ({self.duration_val}s). '
            f'Status: Speed={self.current_speed} km/h, RPM={self.current_rpm}'
        )        

        # Apply service brake
        self.target_gas_brake = 500
        self.get_logger().info('Applying service brake (+500)')

    def stop_and_shutdown(self):
        """
        Final mission stage. Secures the vehicle and stops all timers/interfaces.
        """
        if self.bus is None:
            return
        
        self.get_logger().info('Mission sequence finalized. Initiating secure shutdown.')
        
        # Reset control value
        self.target_gas_brake = 0.0

        # Cancel all timers
        timers_to_cancel = [
            self.timer_drive, self.timer_brake, self.timer_shutdown,
            self.timer_receive, self.timer_heartbeat, self.timer_startup
        ]
        
        for t in timers_to_cancel:
            if t is not None:
                try:
                    t.cancel()
                except Exception:
                    pass
        
        # Send final secure commands
        try:
            hb_msg = can.Message(arbitration_id=AuroraCANIDs.COMMAND, 
                                 data=[0x00, AuroraCANCommand.HANDBRAKE_ENGAGE] + [0]*6, 
                                 is_extended_id=False)
            self.bus.send(hb_msg)
            
            off_msg = can.Message(arbitration_id=AuroraCANIDs.COMMAND, 
                                  data=[0x00, AuroraCANCommand.ENGINE_OFF] + [0]*6, 
                                  is_extended_id=False)
            self.bus.send(off_msg)
            
            # Close the CAN bus interface immediately to prevent resource warnings
            self.bus.shutdown()
            self.bus = None # Mark as closed
        except Exception as e:
            self.get_logger().warn(f'Shutdown CAN communication failed: {e}')

        self.get_logger().info('Vehicle is secured. Shutdown sequence finished.')

    def can_receive_callback(self):
        while True:
            try:
                msg = self.bus.recv(timeout=0)
                if msg is None: break

                # Parse BMS with log throttling
                if msg.arbitration_id == 0x18904010:
                    self.bms_log_counter += 1
                    if self.bms_log_counter % 100 == 0:
                        soc_percent = int.from_bytes(msg.data[6:8], byteorder="big") / 10.0
                        self.get_logger().info(f'BMS Update: Battery at {soc_percent}%')

                # Parse Vehicle Status (ID 0x39)
                elif msg.arbitration_id == AuroraCANIDs.VEHICLE_STATUS:
                    self.current_rpm = int.from_bytes(msg.data[3:5], byteorder="big", signed=False)
                    
                    # Spec requirement: Only consider engine started if RPM > 0
                    self.engine_started = self.current_rpm > 0
                    self.current_speed = float(msg.data[2])
                
                # Parse Handbrake Status (ID 0x38)
                elif msg.arbitration_id == AuroraCANIDs.HANDBRAKE_STATUS:
                    self.handbrake_released = (msg.data[1] == 0x00)
            
            except Exception as e:
                self.emergency_shutdown(f'CAN receive failure: {e}')
                break

    def destroy_node(self):
        """
        Cleanup sequence: stop all timers and shutdown CAN bus.
        """
        # Cancel any remaining timers
        timers = [self.timer_receive, self.timer_heartbeat, self.timer_startup, 
                  self.timer_drive, self.timer_brake, self.timer_shutdown]
        
        for t in timers:
            if t is not None:
                try: t.cancel()
                except Exception: pass

        # Shutdown CAN interface only if it hasn't been shut down yet
        try:
            if self.bus is not None:
                self.bus.shutdown()
                self.bus = None
        except Exception:
            pass

        super().destroy_node()

    def emergency_shutdown(self, reason='Unknown error'):
        """
        Sends one ESTOP heartbeat and performs a controlled emergency shutdown.
        """
        if self.emergency_triggered:
            return

        self.emergency_triggered = True
        self.get_logger().error(f'Emergency shutdown triggered: {reason}')

        # Force neutral command
        self.target_gas_brake = 0.0

        # Try to send one ESTOP heartbeat before shutting down
        try:
            data = [0x00] * 8
            throttle_bytes = int(0).to_bytes(2, byteorder='big', signed=True)
            data[1] = throttle_bytes[0]
            data[2] = throttle_bytes[1]
            data[3] = 0x00
            data[4] = 0x00
            data[7] = 0x01  # ESTOP active

            estop_msg = can.Message(
                arbitration_id=AuroraCANIDs.HEARTBEAT,
                data=data,
                is_extended_id=False
            )
            self.bus.send(estop_msg)
            self.get_logger().warn('ESTOP heartbeat sent')
        except Exception as e:
            self.get_logger().warn(f'Failed to send ESTOP heartbeat: {e}')

        # Reuse normal shutdown flow
        self.stop_and_shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = VehicleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Silently exit on Ctrl+C to avoid ROS 2 context warnings
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()