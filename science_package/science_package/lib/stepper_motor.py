from ticlib import TicUSB
from time import sleep


class StepperMotor:
    def __init__(self):
        self.tic = TicUSB()
        self.resetPosition()

        self.FULL_ROTATION = 200
        self.UNCERTAINTY = 2
        self.FULL_LENGTH = 25

        self.spinCount = 1
        self.positionSet = 0


    def resetPosition(self):
        self.tic.halt_and_set_position(0)
        self.tic.energize()
        self.tic.exit_safe_start()


    def denergize(self):
        self.tic.deenergize()
        self.tic.enter_safe_start()


    def setSpinCount(self, pos):
        if (abs(pos) > self.FULL_ROTATION):
            self.spinCount = pos // self.FULL_ROTATION    # Integer division
            self.positionSet = pos % self.FULL_ROTATION   # Remainder


    def printPositionState(self):
        print("Current position:", self.tic.get_current_position(), "\tTarget position:", self.tic.get_target_position())
        sleep(0.05)


    def step(self, flag: bool):
        if (flag): rotation = 1
        else: rotation = -1

        for i in range(self.FULL_LENGTH + 1):
            self.resetPosition()
            self.tic.set_target_position(rotation * self.FULL_ROTATION)
            self.spinLoop(rotation)

        self.denergize()


    def stepUV(self, flag: bool):
        if (flag): rotation = 1
        else: rotation = -1

        for i in range(int(self.FULL_LENGTH / 2) + 3):
            self.resetPosition()
            self.tic.set_target_position(rotation * self.FULL_ROTATION)
            self.spinLoop(rotation)

        self.denergize()


    def stepOnce(self, flag: bool):
        if (flag): rotation = 1
        else: rotation = -1

        self.resetPosition()
        self.tic.set_target_position(rotation * self.FULL_ROTATION)
        self.spinLoop(rotation)
        self.denergize()

    
    def spinLoop(self, rotation: int):
        while self.tic.get_current_position() != self.tic.get_target_position():
            #self.printPositionState()
            if (rotation > 0):
                if (self.tic.get_current_position() >= self.tic.get_target_position() - self.UNCERTAINTY):
                    return
            else:
                if (self.tic.get_current_position() <= self.tic.get_target_position() + self.UNCERTAINTY):
                    return