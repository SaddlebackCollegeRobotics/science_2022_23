def readPositionFromFile():
    with open("stepper_motor/position.txt", "r") as f:
        return int(f.read())
    
def writePositionToFile(position):
    with open("stepper_motor/position.txt", "w") as f:
        f.write(str(position))