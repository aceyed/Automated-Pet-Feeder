import RPi.GPIO as GPIO
import time
import smbus
# Set up GPIO mode
GPIO.setmode(GPIO.BCM)
GPIO.setup(26, GPIO.OUT)
GPIO.setup(22, GPIO.OUT)

# Set up GPIO pins
# Define keypad GPIO pins for columns (C1, C2, C3) and rows (R1, R2, R3, R4)
columns = [25, 24, 23]  # Update with your wiring
rows = [21, 20, 16, 12]  # Update with your wiring

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

def manual_mode():
    threshold_voltage = 1.85  # Sensor and requirements
    motor_running = False
    blink_interval = 0.5  # LED will blink every 0.5 seconds

    try:
        seconds = float(input("Enter the seconds you want the motor to stay open: "))
        while True:
            # Read the value from the infrared sensor
            bus.write_byte(address, A2)
            value = bus.read_byte(address)
            voltage = (value * 3.3 / 255)
            print("AOUT:%1.3f " % voltage)

            if voltage < threshold_voltage and not motor_running:
                motor_running = True
                print("Infrared sensor triggered! Rotating the servo motor.")

                # Rotate to 180 degrees position
                servo1.ChangeDutyCycle(2)

                # Turn on the LED with blinking
                end_time = time.time() + seconds
                while time.time() < end_time:
                    turn_on_led()  # Turn on the LED
                    time.sleep(blink_interval / 2)  # Wait half the interval
                    turn_off_led()  # Turn off the LED
                    time.sleep(blink_interval / 2)  # Wait half the interval

                # Reset servo to initial position
                servo1.ChangeDutyCycle(7)
                time.sleep(1)
                servo1.ChangeDutyCycle(0)  # Stop sending the signal to the motor
                print("Waiting for sensor to clear...")
                time.sleep(2)

            elif voltage < threshold_voltage:
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
        servo1.stop()
        GPIO.cleanup()


def hours_to_seconds(hours):
    return hours * 3600

# Automatic mode function to dispense based on user input
def autoHelper(interval_seconds, seconds):
    blink_interval = 0.5  # LED will blink every 0.5 seconds
    while True:
        print("Dispensing food automatically.")
        # Rotate to 180 degrees position
        servo1.ChangeDutyCycle(12)
        
        end_time = time.time() + seconds
        while time.time() < end_time:
            turn_on_led()  # Turn on the LED
            time.sleep(blink_interval / 2)  # Wait half the interval
            turn_off_led()  # Turn off the LED
            time.sleep(blink_interval / 2)  # Wait half the interval
        
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


def main():
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
        if not locked:
            # Implement time remaining for the next feeding in auto-timed mode
            print("Time Remaining: X minutes")  # Replace with actual time

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

if __name__ == "__main__":
    main()
