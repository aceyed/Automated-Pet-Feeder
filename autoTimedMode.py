import time
import RPi.GPIO as GPIO

# Set up the GPIO for the servo motor
GPIO.setmode(GPIO.BCM)
GPIO.setup(22, GPIO.OUT)

# Initialize the servo motor
servo1 = GPIO.PWM(22, 50)  # GPIO 22 for PWM with 50Hz
servo1.start(0)

# Function to convert hours to seconds
def hours_to_seconds(hours):
    return hours * 3600

# Automatic mode function to dispense based on user input
def autoHelper(interval_seconds, seconds):
    while True:
        print("Dispensing food automatically.")
        # Rotate to 180 degrees position
        servo1.ChangeDutyCycle(12)
        time.sleep(seconds)  # Wait for the servo to reach the position

        # Reset the servo to 0 degrees
        servo1.ChangeDutyCycle(2)
        time.sleep(1)  # Wait for the servo to return

        servo1.ChangeDutyCycle(0)  # Stop sending the signal
        print(f"Next dispense in {interval_seconds/3600:.1f} hours.")
        
        # Wait for the specified interval before next dispense
        time.sleep(interval_seconds)
def enterAuto():
    try:
        # Ask user for the time interval in hours
        hours = float(input("Enter the time interval for dispensing in hours: "))
        interval_seconds = hours_to_seconds(hours)
        seconds = float(input("Enter the seconds you want the motor to stay open: "))
        print(f"Automatic mode activated. Dispensing will occur every {hours} hours.")
        autoHelper(interval_seconds,seconds)
    except KeyboardInterrupt:
        # Clean up the GPIO on CTRL+C exit
        print("Exiting automatic mode.")
        servo1.stop()
        GPIO.cleanup()
    except ValueError:
        # Handle the error if the user does not enter a valid number
        print("Please enter a valid number for the time interval.")
