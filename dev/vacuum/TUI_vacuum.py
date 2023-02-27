import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
POWER = 17

GPIO.setup(POWER, GPIO.OUT)

while True:
    GPIO.output(POWER, GPIO.HIGH)
    time.sleep(1)
    GPIO.output(POWER, GPIO.LOW)
    time.sleep(1)
    print ("One cycle completed")

GPIO.cleanup()