from system_file import *
from stepper_motor import StepperMotor


def main():
    currentPositon = readPositionFromFile()
    stepper = StepperMotor()

    
    while True:
        print("(1) Move all the way down\n(2) Move all the way up\n(3) Step down once\n(4) Step up once\n(5) Exit\n")
        inp = input("Command: ")


        if inp == "1" and currentPositon == 0:
            stepper.step(True)
            currentPositon = 5000
            writePositionToFile(currentPositon)
        
        if inp == "2" and currentPositon == 5000:
            stepper.step(False)
            currentPositon = 0
            writePositionToFile(currentPositon)
        
        if inp == "3" and currentPositon != 5000:
            stepper.stepOnce(True)
            currentPositon += 200
            writePositionToFile(currentPositon)

        if inp == "4" and currentPositon != 0:
            stepper.stepOnce(False)
            currentPositon -= 200
            writePositionToFile(currentPositon)

        if inp == "5":
            break

        print("Renee is the goat: ", currentPositon)


if __name__ == "__main__":
    main()

