import RPi.GPIO as GPIO
import time

class Mosfet:
    PUMP = 11   # GPIO pin number for pump control
    VACUUM = 13 # GPIO pin number for vacuum control

    def __init__(self):
        """
        Initialize the Mosfet class and set up GPIO pins for pump and vacuum control.
        """
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(Mosfet.PUMP, GPIO.OUT)
        GPIO.setup(Mosfet.VACUUM, GPIO.OUT)

    def pump(self, t: float):
        """
        Turn on the pump for a specified duration of time, then turn it off.
        
        Parameters:
            t (float): The duration of time (in seconds) to run the pump.
        """
        GPIO.output(Mosfet.PUMP, True)
        time.sleep(t)
        GPIO.output(Mosfet.PUMP, False)

    def vacuum(self, t: float):
        """
        Turn on the vacuum for a specified duration of time, then turn it off.
        
        Parameters:
            t (float): The duration of time (in seconds) to run the vacuum.
        """
        GPIO.output(Mosfet.VACUUM, True)
        time.sleep(t)
        GPIO.output(Mosfet.VACUUM, False)

    def __del__(self):
        """
        Clean up GPIO pins when the object is deleted.
        """
        GPIO.cleanup()
