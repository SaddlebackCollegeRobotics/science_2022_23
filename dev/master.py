from stepper_motor.system_file import *
from stepper_motor.stepper_motor import StepperMotor

def main():
    currentPositon = readPositionFromFile()
    stepper = StepperMotor()

    while True:
        print("(3) Lower for UV Science\n(4) Retract Lowering Platform\n(5) Step down once\n(6) Step up once\n(7) Exit\n")
        inp = input("Command: ")

        if inp == "3" and currentPositon == 0:
            stepper.stepUV(True)
            currentPositon += 2800
            writePositionToFile(currentPositon)

        if inp == "4" and currentPositon == 2800:
            stepper.stepUV(False)
            currentPositon -= 2800
            writePositionToFile(currentPositon)

        if inp == "5" and currentPositon != 5000:
            stepper.stepOnce(True)
            currentPositon += 200
            writePositionToFile(currentPositon)

        if inp == "6" and currentPositon != 0:
            stepper.stepOnce(False)
            currentPositon -= 200
            writePositionToFile(currentPositon)
        
        if inp == "7":
            break

        print("Renee is the goat: ", currentPositon)


if __name__ == "__main__":
    main()