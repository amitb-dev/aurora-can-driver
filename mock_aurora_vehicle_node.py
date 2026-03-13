#!/usr/bin/env python3
"""
Mock Aurora Vehicle Node
Simulates Aurora vehicle CAN bus responses for development without hardware.

PROVIDED FILE — Do not modify.
"""

import time
from threading import Thread

import can
import rclpy
from canbridge.aurora_message import (
    AuroraCANCommand,
    AuroraCANIDs,
    AuroraGear,
    AuroraMessageHandler,
)
from rclpy.node import Node


class MockAuroraVehicle(Node):
    """
    Mock Aurora vehicle that responds to CAN commands.
    Simulates realistic vehicle behavior without hardware.

    CAN Interface: vcan0 (socketcan) with UDP multicast fallback.

    Status messages sent at ~50 Hz:
      0x31  STEERING_FEEDBACK
      0x37  VEHICLE_LIGHT_STATUS
      0x38  HANDBRAKE_STATUS
      0x39  VEHICLE_STATUS
      0x18904010  BMS_BATTERY_STATUS (29-bit extended)

    Commands accepted (CAN ID 0x49, byte 1):
      0x14  ENGINE_IGNITION_ON  — engine starts after ~1s
      0x15  ENGINE_STARTER      — starter pulse ~0.5s
      0x16  ENGINE_OFF
      0x17  GEAR_DRIVE
      0x18  GEAR_REVERSE
      0x19  GEAR_NEUTRAL
      0x1A  HANDBRAKE_ENGAGE
      0x1D  HANDBRAKE_DISENGAGE

    Control input (CAN ID 0x41, sent by controller at 20-50 Hz):
      bytes 1-2: gas_brake int16 big-endian (negative=accelerate, positive=brake)
      bytes 3-4: steering  int16 big-endian
      byte  7:   estop     0x01 = emergency stop
    """

    def __init__(self):
        super().__init__("mock_aurora_vehicle")

        self.declare_parameter("uuid", "unknown")
        self.platform_uuid = self.get_parameter("uuid").value

        self.declare_parameters(
            namespace="",
            parameters=[
                ("can_channel", "vcan0"),
                ("response_rate", 50.0),
            ],
        )

        can_channel = self.get_parameter("can_channel").value

        try:
            self.bus = can.Bus(channel=can_channel, bustype="socketcan")
            self.get_logger().info(f"Mock vehicle connected to {can_channel} (socketcan)")
        except Exception as socketcan_error:
            if can_channel.startswith("vcan"):
                try:
                    self.bus = can.Bus(channel="239.74.163.2", bustype="udp_multicast", port=43200)
                    self.get_logger().info("Mock vehicle connected (UDP multicast fallback)")
                except Exception as multicast_error:
                    self.get_logger().error(f"CAN init failed: {multicast_error}")
                    raise
            else:
                self.get_logger().error(f"Failed to connect to CAN on {can_channel}: {socketcan_error}")
                raise socketcan_error

        self.msg_handler = AuroraMessageHandler()

        # Vehicle state
        self.steering_angle = 0
        self.steering_current = 0
        self.regular_lights_on = False
        self.high_lights_on = False
        self.handbrake_engaged = True
        self.estop_engaged = False
        self.brake_pressed = False
        self.starter_active = False
        self.high_speed_mode = False
        self.ignition_on = False
        self.gear = AuroraGear.NEUTRAL
        self.speed_kmh = 0
        self.rpm = 0
        self.engine_running = False

        self.mock_battery_soc = 85.0
        self.mock_12v_voltage = 12.8

        self.last_gas_brake = 0
        self.last_steering = 0

        self.running = True
        self.receive_thread = Thread(target=self.receive_commands, daemon=True)
        self.send_thread = Thread(target=self.send_status, daemon=True)
        self.receive_thread.start()
        self.send_thread.start()

        self.create_timer(0.1, self.simulate_physics)

        self.declare_parameter('rocu1_connected', False)
        self.rocu1_connected = self.get_parameter('rocu1_connected').value
        self.create_timer(1.0, self._update_rocu1_param)

        self.get_logger().info("Mock Aurora vehicle ready")

    def receive_commands(self):
        while self.running:
            try:
                msg = self.bus.recv(timeout=0.1)
                if msg:
                    self.process_command(msg)
            except Exception as e:
                if self.running:
                    self.get_logger().error(f"Error receiving CAN: {e}")

    def process_command(self, msg):
        can_id = msg.arbitration_id
        data = msg.data

        if can_id == AuroraCANIDs.HEARTBEAT:
            if len(data) >= 5:
                gas_brake = int.from_bytes(data[1:3], byteorder="big", signed=True)
                steering = int.from_bytes(data[3:5], byteorder="big", signed=True)
                estop = data[7] == 0x01 if len(data) >= 8 else False
                self.last_gas_brake = gas_brake
                self.last_steering = steering
                self.estop_engaged = estop
                self.get_logger().info(
                    f"Received control: gas_brake={gas_brake}, steering={steering}, estop={estop}",
                    throttle_duration_sec=1.0,
                )

        elif can_id == AuroraCANIDs.COMMAND:
            if len(data) >= 2:
                self.process_state_command(data[1])

    def process_state_command(self, command):
        if command == AuroraCANCommand.ENGINE_STARTER:
            self.get_logger().info("🔑 Starter engaged")
            self.starter_active = True
            import threading
            threading.Timer(0.5, self._deactivate_starter).start()

        elif command == AuroraCANCommand.ENGINE_IGNITION_ON:
            self.get_logger().info("🔥 Ignition on - Engine starting")
            self.ignition_on = True
            import threading
            threading.Timer(1.0, self.start_engine).start()

        elif command == AuroraCANCommand.ENGINE_OFF:
            self.get_logger().info("🔴 Engine off")
            self.engine_running = False
            self.ignition_on = False
            self.rpm = 0
            self.speed_kmh = 0

        elif command == AuroraCANCommand.GEAR_DRIVE:
            self.get_logger().info("⚙️  Gear: DRIVE")
            self.gear = AuroraGear.DRIVE

        elif command == AuroraCANCommand.GEAR_REVERSE:
            self.get_logger().info("⚙️  Gear: REVERSE")
            self.gear = AuroraGear.REVERSE

        elif command == AuroraCANCommand.GEAR_NEUTRAL:
            self.get_logger().info("⚙️  Gear: NEUTRAL")
            self.gear = AuroraGear.NEUTRAL

        elif command == AuroraCANCommand.HANDBRAKE_ENGAGE:
            self.get_logger().info("🅿️  Handbrake engaged")
            self.handbrake_engaged = True

        elif command == AuroraCANCommand.HANDBRAKE_DISENGAGE:
            self.get_logger().info("🅿️  Handbrake released")
            self.handbrake_engaged = False

        elif command == AuroraCANCommand.SPEED_MODE_LOW:
            self.get_logger().info("🏎️  High speed mode")
            self.high_speed_mode = True

        elif command == AuroraCANCommand.SPEED_MODE_HIGH:
            self.get_logger().info("🐌 Low speed mode")
            self.high_speed_mode = False

        elif command == AuroraCANCommand.REGULAR_LIGHTS:
            self.regular_lights_on = not self.regular_lights_on
            self.get_logger().info(f"💡 Regular lights {'ON' if self.regular_lights_on else 'OFF'}")

        elif command == AuroraCANCommand.HIGH_LIGHTS:
            self.high_lights_on = not self.high_lights_on
            self.get_logger().info(f"💡 High lights {'ON' if self.high_lights_on else 'OFF'}")

    def _deactivate_starter(self):
        self.starter_active = False

    def start_engine(self):
        if self.ignition_on:
            self.engine_running = True
            self.rpm = 800
            self.get_logger().info("✅ Engine running")

    def simulate_physics(self):
        if not self.engine_running:
            self.rpm = 0
            self.speed_kmh = max(0, self.speed_kmh - 2)
            return

        target_rpm = 800 + (abs(self.last_gas_brake) / 700.0) * 2200 if self.last_gas_brake < 0 else 800
        self.rpm = int(self.rpm + (target_rpm - self.rpm) * 0.1)

        if self.handbrake_engaged:
            self.speed_kmh = max(0, self.speed_kmh - 5)
        elif self.gear == AuroraGear.DRIVE:
            if self.last_gas_brake < 0:
                self.speed_kmh = min(100, self.speed_kmh + (abs(self.last_gas_brake) / 700.0) * 2.0)
            elif self.last_gas_brake > 0:
                self.speed_kmh = max(0, self.speed_kmh - abs(self.last_gas_brake / 700.0) * 5.0)
            else:
                self.speed_kmh = max(0, self.speed_kmh - 0.5)
        elif self.gear == AuroraGear.REVERSE:
            if self.last_gas_brake < 0:
                self.speed_kmh = min(20, self.speed_kmh + (abs(self.last_gas_brake) / 700.0) * 1.0)
            else:
                self.speed_kmh = max(0, self.speed_kmh - 1.0)
        else:
            self.speed_kmh = max(0, self.speed_kmh - 1.0)

        self.steering_angle = int(self.steering_angle + (self.last_steering - self.steering_angle) * 0.2)
        self.steering_current = abs(self.steering_angle) // 100
        self.brake_pressed = self.last_gas_brake > 100

        if self.engine_running:
            self.mock_12v_voltage = min(14.2, self.mock_12v_voltage + 0.002 + (self.rpm / 3000.0) * 0.003)
        else:
            self.mock_12v_voltage = max(10.5, self.mock_12v_voltage - (0.0005 if self.ignition_on else 0.0001))

    def send_status(self):
        period = 1.0 / 50
        while self.running:
            try:
                self.send_steering_feedback()
                self.send_vehicle_light_status()
                self.send_handbrake_status()
                self.send_vehicle_status()
                self.send_bms_battery_status()
                if self.rocu1_connected:
                    self.send_rocu1_status()
                time.sleep(period)
            except Exception as e:
                if self.running:
                    self.get_logger().error(f"Error sending status: {e}")

    def send_steering_feedback(self):
        data = bytearray(8)
        data[0:2] = self.steering_angle.to_bytes(2, byteorder="big", signed=True)
        data[4:6] = self.steering_current.to_bytes(2, byteorder="big", signed=True)
        self.bus.send(can.Message(arbitration_id=AuroraCANIDs.STEERING_FEEDBACK, data=bytes(data), is_extended_id=False))

    def send_vehicle_light_status(self):
        data = bytearray(8)
        data[0] = (1 if self.regular_lights_on else 0) | (2 if self.high_lights_on else 0)
        self.bus.send(can.Message(arbitration_id=AuroraCANIDs.VEHICLE_LIGHT_STATUS, data=bytes(data), is_extended_id=False))

    def send_handbrake_status(self):
        data = bytearray(8)
        data[1] = 0x01 if self.handbrake_engaged else 0x00
        data[6] = 0x02
        data[7] = 0x01 if self.estop_engaged else 0x00
        self.bus.send(can.Message(arbitration_id=AuroraCANIDs.HANDBRAKE_STATUS, data=bytes(data), is_extended_id=False))

    def send_vehicle_status(self):
        data = bytearray(8)
        byte0 = 0
        if self.brake_pressed:    byte0 |= 1 << 4
        if self.starter_active:   byte0 |= 1 << 5
        if not self.high_speed_mode: byte0 |= 1 << 6
        if self.ignition_on:      byte0 |= 1 << 7
        data[0] = byte0
        data[1] = self.gear.value
        data[2] = max(0, min(255, int(round(self.speed_kmh))))
        data[3:5] = self.rpm.to_bytes(2, byteorder="big", signed=False)
        data[5] = max(0, min(255, int(round(self.mock_12v_voltage * 10))))
        self.bus.send(can.Message(arbitration_id=AuroraCANIDs.VEHICLE_STATUS, data=bytes(data), is_extended_id=False))

    def send_bms_battery_status(self):
        data = bytearray(8)
        self.mock_battery_soc = max(0.0, self.mock_battery_soc - 0.0001)
        data[0:2] = (480).to_bytes(2, byteorder="big", signed=False)
        data[4:6] = (30010).to_bytes(2, byteorder="big", signed=False)
        data[6:8] = int(self.mock_battery_soc * 10).to_bytes(2, byteorder="big", signed=False)
        self.bus.send(can.Message(arbitration_id=0x18904010, data=bytes(data), is_extended_id=True))

        if not hasattr(self, "_bms_log_counter"):
            self._bms_log_counter = 0
        self._bms_log_counter += 1
        if self._bms_log_counter % 500 == 0:
            self.get_logger().info(f"🔋 Mock BMS: SOC={self.mock_battery_soc:.1f}% | 12V={self.mock_12v_voltage:.1f}V")

    def send_rocu1_status(self):
        self.bus.send(can.Message(arbitration_id=AuroraCANIDs.ROCU1_STATUS, data=bytes([0x01]), is_extended_id=False))

    def _update_rocu1_param(self):
        new_value = self.get_parameter('rocu1_connected').value
        if new_value != self.rocu1_connected:
            self.rocu1_connected = new_value
            self.get_logger().info(f"🎮 ROCU1 simulation: {'CONNECTED' if new_value else 'DISCONNECTED'}")

    def destroy_node(self):
        self.running = False
        if self.bus:
            self.bus.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    mock_vehicle = None
    try:
        mock_vehicle = MockAuroraVehicle()
        rclpy.spin(mock_vehicle)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if mock_vehicle:
            mock_vehicle.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
