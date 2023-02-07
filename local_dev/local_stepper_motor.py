from ticlib import TicUSB
from time import sleep
# from RPi import GPIO


# GPIO.setmode(GPIO.BCM)


class StepperMotor:
    # Private constant variables
    __FULL_ROTATION = 200
    __UNCERTAINTY = 2
    __FULL_LENGTH = 25


    def __init__(self):
        self.tic = TicUSB()
        self.resetPosition()

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
        if (abs(pos) > self.__FULL_ROTATION):
            self.spinCount = pos // self.__FULL_ROTATION    # Integer division
            self.positionSet = pos % self.__FULL_ROTATION   # Remainder

    
    def printPositionState(self):
        print("Current position:", self.tic.get_current_position(), "\tTarget position:", self.tic.get_target_position())
        sleep(0.05)


    # def spinTest(self, pos):
    #     self.setSpinCount(pos)

    #     curSpinRate = self.__FULL_ROTATION

    #     for i in range(self.spinCount + 1):
    #         self.resetPosition()

    #         if (i == self.spinCount): curSpinRate = self.positionSet
            
    #         self.tic.set_target_position(curSpinRate)

    #         while self.tic.get_current_position() != self.tic.get_target_position():
    #             self.printPositionState()
    #             if (self.tic.get_current_position() >= self.tic.get_target_position() - self.__UNCERTAINTY):
    #                 print("Position reached")
    #                 break

    #     self.denergize()


    # def spinForever(self, roation):
    #     while True:
    #         self.resetPosition()
    #         self.tic.set_target_position(roation * self.__FULL_ROTATION)

    #         while True:
    #             self.printPositionState()
    #             if (roation > 0):
    #                 if (self.tic.get_current_position() >= self.tic.get_target_position() - self.__UNCERTAINTY):
    #                     break
    #             else:
    #                 if (self.tic.get_current_position() <= self.tic.get_target_position() + self.__UNCERTAINTY):
    #                     break
    

    def step(self, type):
        if (type.lower() == "down"): rotation = 1
        elif (type.lower() == "up"): rotation = -1

        for i in range(self.__FULL_LENGTH + 1):
            self.resetPosition()
            self.tic.set_target_position(rotation * self.__FULL_ROTATION)

            while self.tic.get_current_position() != self.tic.get_target_position():
                self.printPositionState()
                if (rotation > 0):
                    if (self.tic.get_current_position() >= self.tic.get_target_position() - self.__UNCERTAINTY):
                        break
                else:
                    if (self.tic.get_current_position() <= self.tic.get_target_position() + self.__UNCERTAINTY):
                        break

        self.denergize()



def main():
    stepper = StepperMotor()
    
    while True:
        strInput = input("Input (up/down): ")
        stepper.step(strInput.lower())

if __name__ == "__main__":
    main()

