import RPi.GPIO as GPIO
import time


class Mosfet:
    PUMP = 11
    VACUUM = 13

    def __init__(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(Mosfet.PUMP, GPIO.OUT)
import time


class Mosfet:
    PUMP = 11
    VACUUM = 13

    def __init__(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(Mosfet.PUMP, GPIO.OUT)
        GPIO.setup(Mosfet.VACUUM, GPIO.OUT)

    def pump(self, t:float):
        GPIO.output(Mosfet.PUMP, True)
        time.sleep(t)
        GPIO.output(Mosfet.PUMP, False)

    def vacuum(self, t:float):
        GPIO.output(Mosfet.VACUUM, True)
        time.sleep(t)
        GPIO.output(Mosfet.VACUUM, False)

    def __del__(self):
        GPIO.cleanup()


        GPIO.setup(Mosfet.VACUUM, GPIO.OUT)

    def pump(self, t:float):
        GPIO.output(Mosfet.PUMP, True)
        time.sleep(t)
        GPIO.output(Mosfet.PUMP, False)

    def vacuum(self, t:float):
        GPIO.output(Mosfet.VACUUM, True)
        time.sleep(t)
        GPIO.output(Mosfet.VACUUM, False)

    def __del__(self):
        GPIO.cleanup()

