"""
TEMPORARY MOCK FILE 
This module was created manually to satisfy dependencies for the 'mock_aurora_vehicle_node.py' 
as the original 'canbridge' package was missing from the initial assignment files.
Constants and IDs are mapped based on the provided PDF protocol documentation.
To be replaced with the official package upon receipt.
"""

from enum import Enum

class AuroraCANIDs:
    """CAN message IDs from protocol reference & mock code requirements"""
    COMMAND = 0x49
    HEARTBEAT = 0x41
    STEERING_FEEDBACK = 0x31
    VEHICLE_LIGHT_STATUS = 0x37  # זה מה שהיה חסר!
    HANDBRAKE_STATUS = 0x38
    VEHICLE_STATUS = 0x39
    BMS_BATTERY_STATUS = 0x18904010
    ROCU1_STATUS = 0x51 # הוספתי ליתר ביטחון למניעת קריסה הבאה

class AuroraCANCommand:
    """Command codes (Byte 1)"""
    ENGINE_IGNITION_ON = 0x14
    ENGINE_STARTER = 0x15
    ENGINE_OFF = 0x16
    GEAR_DRIVE = 0x17
    GEAR_REVERSE = 0x18
    GEAR_NEUTRAL = 0x19
    HANDBRAKE_ENGAGE = 0x1A
    HANDBRAKE_DISENGAGE = 0x1D
    SPEED_MODE_LOW = 0x1B
    SPEED_MODE_HIGH = 0x1C
    REGULAR_LIGHTS = 0x20 # הערכה לבינתיים
    HIGH_LIGHTS = 0x21    # הערכה לבינתיים

class AuroraGear(Enum):
    DRIVE = 0x01
    REVERSE = 0x02
    NEUTRAL = 0x03

class AuroraMessageHandler:
    def __init__(self):
        pass