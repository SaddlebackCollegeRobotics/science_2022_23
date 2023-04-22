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


class MotorController:
    """
    This class controls the motors connected to the roboclaw motor driver.
    """

    def __init__(self, rc: Roboclaw, address: int):
        """
        Initializes the MotorController class.

        Args:
            rc (Roboclaw): Instance of the roboclaw class used to control the motors.
            address (int): Address of the motor controller.
        """
        self.rc = rc
        self.address = address
        self.rc.Open()

    def reset_encoders(self):
        """
        Resets the encoders of both motors to zero.
        """
        self.rc.ResetEncoders(self.address)
        print("Success: Reset Encoders to 0")

    def print_encoders(self) -> None:
        """
        Prints the speed and encoder values of both motors to the terminal.
        """
        enc1 = self.rc.ReadEncM1(self.address)
        print(f"M1: {enc1[1]}" if enc1[0] == 1 else "disconnected", end=' ')
        enc2 = self.rc.ReadEncM2(self.address)
        print(f"M2: {enc2[1]}" if enc2[0] == 1 else "disconnected")


def set_turret_rotation(mcp: MotorController, encoder_val: float) -> None:
    """
    Sets the rotation of the turret.

    Args:
        mcp (MotorController): Instance of the MotorController class.
        encoder_val (float): Encoder value for the desired rotation of the turret.
    """
    mcp.rc.SpeedAccelDeccelPositionM2(mcp.address, ACCELERATION, SPEED, ACCELERATION, encoder_val, BUFFER_OR_INSTANT)
    print("Set turret rotation to: ", encoder_val)
