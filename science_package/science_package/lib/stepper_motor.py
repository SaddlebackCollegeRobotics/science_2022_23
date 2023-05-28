from ticlib import TicUSB
from time import sleep


class StepperMotor:
    def __init__(self):
        """
        Initializes the stepper motor class and sets default values for parameters.
        """
        self.tic = TicUSB()
        self.resetPosition()

        self.FULL_ROTATION = 200
        self.UNCERTAINTY = 2
        self.FULL_LENGTH = 25

        self.spinCount = 1
        self.positionSet = 0


    def resetPosition(self):
        """
        Resets the position of the stepper motor to 0.
        """
        self.tic.halt_and_set_position(0)
        self.tic.energize()
        self.tic.exit_safe_start()


    def denergize(self):
        """
        Disables the stepper motor.
        """
        self.tic.deenergize()
        self.tic.enter_safe_start()


    def setSpinCount(self, pos):
        """
        Sets the spin count and position for the stepper motor.

        Parameters:
        pos (int): The position to set for the stepper motor.
        """
        if (abs(pos) > self.FULL_ROTATION):
            self.spinCount = pos // self.FULL_ROTATION    # Integer division
            self.positionSet = pos % self.FULL_ROTATION   # Remainder


    def printPositionState(self):
        """
        Prints the current position and target position of the stepper motor.
        """
        print("Current position:", self.tic.get_current_position(), "\tTarget position:", self.tic.get_target_position())
        sleep(0.05)


    def step(self, flag: bool):
        """
        Steps the stepper motor a full rotation in either a clockwise or counterclockwise direction.

        Parameters:
        flag (bool): True if the stepper motor should rotate clockwise, False if counterclockwise.
        """
        if (flag): rotation = 1
        else: rotation = -1

        for i in range(self.FULL_LENGTH + 1):
            self.resetPosition()
            self.tic.set_target_position(rotation * self.FULL_ROTATION)
            self.spinLoop(rotation)

        self.denergize()


    def stepUV(self, flag: bool):
        """
        Steps the stepper motor a half rotation in either a clockwise or counterclockwise direction.

        Parameters:
        flag (bool): True if the stepper motor should rotate clockwise, False if counterclockwise.
        """
        if (flag): rotation = 1
        else: rotation = -1

        # Hardcoded, will change later... (so never unless someone found this comment)
        for i in range(int(self.FULL_LENGTH / 2) + 13):
            self.resetPosition()
            self.tic.set_target_position(rotation * self.FULL_ROTATION)
            self.spinLoop(rotation)

        self.denergize()


    def stepOnce(self, flag: bool):
        """
        Steps the stepper motor one full rotation in either a clockwise or counterclockwise direction.

        Parameters:
        flag (bool): True if the stepper motor should rotate clockwise, False if counterclockwise.
        """
        if (flag): rotation = 1
        else: rotation = -1

        self.resetPosition()
        self.tic.set_target_position(rotation * self.FULL_ROTATION)
        self.spinLoop(rotation)
        self.denergize()

    
    def spinLoop(self, rotation: int):
        """
        Continuously spins the stepper motor until it reaches its target position.

        Parameters:
        rotation (int): The direction the stepper motor should spin in.
        """
        while self.tic.get_current_position() != self.tic.get_target_position():
            if (rotation > 0):
                if (self.tic.get_current_position() >= self.tic.get_target_position() - self.UNCERTAINTY):
                    return
            else:
                if (self.tic.get_current_position() <= self.tic.get_target_position() + self.UNCERTAINTY):
                    return