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

# Define LED GPIO pin
led_pin = GPIO.PWM(26, 60)

# Define infrared sensor I2C address
infrared_sensor_address = 0x48
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
    if led_pin.GetDutyCycle() == 0:
        led_pin.ChangeDutyCycle(100)

# Function to turn off led, checks if led is on first
def turn_off_led():
    if led_pin.GetDutyCycle() == 100:
        led_pin.ChangeDutyCycle(0)

# Makes the led blink in 0.5 second intervals throughout the open_time, preserves
# initial led state after blinking is done
def blink_led(open_time):
    initial_duty_cycle = led_pin.GetDutyCycle()
    
    # Calculate the number of blinks (each blink is 0.5 seconds)
    num_blinks = int(2 * open_time)

    for _ in range(num_blinks):
        if led_pin.GetDutyCycle() == 0:
            led_pin.ChangeDutyCycle(100)  # Turn on the LED
        else:
            led_pin.ChangeDutyCycle(0)  # Turn off the LED
        time.sleep(0.5)

    # Restore the LED to its initial state
    led_pin.ChangeDutyCycle(initial_duty_cycle)


# Function to read value of infrared sensor
def read_infrared_sensor():
    bus.write_byte(infrared_sensor_address, A2)  # Use the appropriate channel
    value = bus.read_byte(infrared_sensor_address)
    dist = value * 3.3 / 255  # Convert to voltage or distance as needed
    return dist

# Funciton to open feeder door for a set time, then closes it
def open_feeder_door(open_time):
    blink_led(open_time)  # Indicate feeding
    servo1.start(0)
    # Adjust servo motor to open the door
    servo1.ChangeDutyCycle(12)
    time.sleep(open_time)
    servo1.ChangeDutyCycle(0)
    servo1.stop()


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

food_releases = 0

def manual_mode(infrared_triggered, open_time):
    global food_releases

    if infrared_triggered:
        open_feeder_door(open_time)  # Release food
        food_releases += 1

        # Add logic to track food releases per hour
        # You can use the time module to measure time intervals
        current_time = time.time()
        # Check if an hour has passed and reset the food release count
        if current_time - start_time >= 3600:  # 3600 seconds = 1 hour
            food_releases = 0
            start_time = current_time

start_time = time.time()
interval = 60

def auto_timed_mode(interval, open_time):
    current_time = time.time()
    if current_time - start_time >= interval:
        open_feeder_door(open_time)  # Release food
        start_time = current_time

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
                    turn_off_led()  # Indicate locked state
                    active_mode = "N/A"  # Reset active mode when locked
                else:
                    print("Invalid input")
        elif choice == '2':
            if not locked:
                # Enter manual mode
                open_time = float(input("Enter open time for the servo motor (in seconds): "))
                active_mode = "Manual Mode"
                open_feeder_door(open_time)
        elif choice == '3':
            if not locked:
                # Enter auto-timed mode
                interval = int(input("Enter activation interval (in minutes): "))
                open_time = float(input("Enter open time for the servo motor (in seconds): "))
                active_mode = "Auto-Timed Mode"
                auto_timed_mode(interval, open_time)
        else:
            print("Invalid choice")

if __name__ == "__main__":
    main()



