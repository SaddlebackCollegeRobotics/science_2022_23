from dataclasses import dataclass
from .roboclaw_3 import Roboclaw

# Windows comport name - "COM8"
# Linux comport name - "/dev/ttyACM1"
# OSX comport name - "/dev/tty.usbmodem1301"

# TODO - Need to find better way to do this.
COMPORT_NAME_1 = "/dev/ttyACM0"
BUFFER_OR_INSTANT = 1  # 0 for buffer, 1 for instant write
SPEED = 3000
ACCELERATION = 2500


class Motor_Controller:

    # Init class
    def __init__(self, rc: Roboclaw, address: int):

        self.rc, self.address = rc, address
        self.rc.Open()


    # Reset encoders of both motors to zero
    def reset_encoders(self):

        self.rc.ResetEncoders(self.address)
        print("Success: Reset Encoders to 0")


    # Print speed and encoder values of both motors to terminal
    def print_encoders(self) -> None:

        enc1 = self.rc.ReadEncM1(self.address)
        print(f"M1: {enc1[1]}" if enc1[0] == 1 else "disconnected", end=' ')

        enc2 = self.rc.ReadEncM2(self.address)
        print(f"M2: {enc2[1]}" if enc2[0] == 1 else "disconnected")




# Set rotation of turret
def set_turret_rotation(mcp: Motor_Controller, encoder_val : float) -> None:

    mcp.rc.SpeedAccelDeccelPositionM2(mcp.address, 
        ACCELERATION, SPEED, ACCELERATION, encoder_val, BUFFER_OR_INSTANT)
    
    print("Set turret rotation to: ", encoder_val)

