# Ali Syed & Manan Pathak

import RPi.GPIO as GPIO
import time
import smbus
# Set up GPIO mode
GPIO.setmode(GPIO.BCM)
GPIO.setup(26, GPIO.OUT)
GPIO.setup(22, GPIO.OUT)

# Set up GPIO pins
# Define keypad GPIO pins for columns (C1, C2, C3) and rows (R1, R2, R3, R4)
columns = [25, 24, 23]  
rows = [21, 20, 16, 12]  

# Define servo motor GPIO pin
servo1 = GPIO.PWM(22, 50)
servo1.start(0)

# Define LED GPIO pin
led_pin = GPIO.PWM(26, 60)

# Define infrared sensor I2C address
address = 0x48
A2 = 0x42

# Initialize I2C bus
bus = smbus.SMBus(1)


# Set columns as outputs
for column in columns:
    GPIO.setup(column, GPIO.OUT)

# Set rows as inputs with pull-down resistors
for row in rows:
    GPIO.setup(row, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# Function to turn on led, checks if led is off first
def turn_on_led():
    led_pin.ChangeDutyCycle(100)

# Function to turn off led, checks if led is on first
def turn_off_led():
    led_pin.ChangeDutyCycle(0)


def get_keypad_input():
    keypad = [
        ['1', '2', '3'],
        ['4', '5', '6'],
        ['7', '8', '9'],
        ['*', '0', '#']
    ]

    input_keys = ""

    while len(input_keys) < 4:
        for col_num, column in enumerate(columns):
            GPIO.output(column, GPIO.HIGH)

            for row_num, row in enumerate(rows):
                if GPIO.input(row) == GPIO.HIGH:
                    key = keypad[row_num][col_num]
                    print(key)
                    input_keys += key
                    time.sleep(0.2)  # Debounce delay


            GPIO.output(column, GPIO.LOW)

    return input_keys


def dim_led(pin, fade_time_seconds):


    led_pin.start(100)  # Start with maximum brightness

    # Calculate the time interval for each step
    step_time = fade_time_seconds / 100  # 100   steps from 100% to 0%


    for duty_cycle in range(100, 0, -1):
        led_pin.ChangeDutyCycle(duty_cycle)
        time.sleep(step_time)


def manual_mode():
    lower_threshhold = 1.75
    upper_threshold = 1.80
    motor_running = False

    try:
        seconds = float(input("Enter the seconds you want the motor to stay open: "))
        degree = int(input("Enter 2 for opening door all the way, or 4 for opening door half way: "))

        while True:
            # Read the value from the infrared sensor
            bus.write_byte(address, A2)
            value = bus.read_byte(address)
            voltage = (value * 3.3 / 255)
            print("AOUT:%1.3f " % voltage)

            if lower_threshhold < voltage < upper_threshold and not motor_running:
                motor_running = True
                print("Infrared sensor triggered! Rotating the servo motor.")

                # Rotate to 90 degrees position
                servo1.ChangeDutyCycle(degree)

                # Turn on the LED with blinking
                dim_led(26, seconds)

                # Reset servo to initial position
                servo1.ChangeDutyCycle(7)
                time.sleep(1)
                servo1.ChangeDutyCycle(0)  # Stop sending the signal to the motor
                print("Waiting for sensor to clear...")
                time.sleep(2)

            elif voltage < lower_threshhold or voltage > upper_threshold:
                motor_running = False  # Reset the flag when the sensor is clear

            # Short delay before the next reading
            time.sleep(0.1)

    except ValueError:
        # Handle the error if the user does not enter a valid number
        print("Input error")
    except KeyboardInterrupt:
        # Clean up the GPIO on CTRL+C exit
        print("Exiting manual mode.")
        turn_off_led()  # Ensure LED is off when exiting


def hours_to_seconds(hours):
    return hours * 3600

# Automatic mode function to dispense based on user input
def autoHelper(interval_seconds, seconds, degree):

    while True:
        print("Dispensing food automatically.")
        # Rotate to 90 degrees position
        servo1.ChangeDutyCycle(degree)
        
        end_time = time.time() + seconds
        while time.time() < end_time:
            dim_led(26, seconds)
        
        # Reset the servo to 0 degrees
        servo1.ChangeDutyCycle(7)
        time.sleep(1)  # Wait for the servo to return
        servo1.ChangeDutyCycle(0)  # Stop sending the signal
        time.sleep(2)
        print(f"Next dispense in {interval_seconds/3600:.1f} hours.")
        
        # Wait for the specified interval before next dispense
        time.sleep(interval_seconds)

def enterAuto():
    try:
        # Ask user for the time interval in hours
        hours = float(input("Enter the time interval for dispensing in hours: "))
        interval_seconds = hours_to_seconds(hours)
        seconds = float(input("Enter the seconds you want the motor to stay open: "))
        degree = int(input("Enter 2 for opening door all the way, or 4 for opening door half way: "))

        print(f"Automatic mode activated. Dispensing will occur every {hours} hours.")
        autoHelper(interval_seconds,seconds, degree)
    except KeyboardInterrupt:
        # Clean up the GPIO on CTRL+C exit
        print("Exiting automatic mode.")
    except ValueError:
        # Handle the error if the user does not enter a valid number
        print("Please enter a valid number for the time interval.")


def main():
    try:
        # Initialize your program here
        led_pin.start(100)
        print("Pet Food Dispenser Program")
        password = "0000"  # Default password
        locked = True  # Initial state is locked
        active_mode = "N/A"  # Initialize the active mode as "N/A"

        while True:
            print("Menu")
            print(f"Locked: {'Yes' if locked else 'No'}")
            print(f"Active Mode: {active_mode}")

            print("1. Lock/Unlock")
            print("2. Manual Mode")
            print("3. Auto-Timed Mode")

            choice = input("Enter your choice: ")

            if choice == '1':
                if locked:
                    # Unlock the dispenser
                    entered_password = get_keypad_input()
                    if entered_password == password:
                        locked = False
                        print("Unlocked")
                        turn_off_led()  # Indicate unlocked state
                    else:
                        print("Incorrect password")
                else:
                    # Lock the dispenser
                    entered_password = input("Enter 'lock' to lock: ")
                    if entered_password.lower() == 'lock':
                        locked = True
                        print("Locked")
                        turn_on_led()  # Indicate locked state
                        active_mode = "N/A"  # Reset active mode when locked
                    else:
                        print("INVALID INPUT")
            elif choice == '2':
                if not locked:
                    # Enter manual mode
                    manual_mode()

            elif choice == '3':
                if not locked:
                    # Enter auto-timed mode
                    enterAuto()
            elif locked:
                print("FEEDER LOCKED")

            else:
                print("INVALID CHOICE")
    finally:
        print("Exiting Program")
        led_pin.stop
        servo1.stop
        GPIO.cleanup()
if __name__ == "__main__":
    main()
