#SERVO MOTOR
import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setup(12, GPIO.OUT)

servo1 = GPIO.PWM(12, 50)

servo1.start(0)
print("2 seconds")
time.sleep(2)

print("rotating 180")

duty = 12

time.sleep(2)

print("turn back 90")
servo1.ChangeDutyCycle(7)
time.sleep(2)

print("turn back to 0")
servo1.ChangeDutyCycle(2)
time.sleep(0.5)
servo1.ChangeDutyCycle(0)

servo1.stop
GPIO.cleanup()
print("Done")
