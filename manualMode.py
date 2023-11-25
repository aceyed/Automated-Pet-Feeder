import smbus
import time
import RPi.GPIO as GPIO

A = 0x42  # infrared
bus = smbus.SMBus(1)

# Set up the GPIO for the servo motor
GPIO.setmode(GPIO.BCM)
GPIO.setup(22, GPIO.OUT)
address = 0x48
servo1 = GPIO.PWM(22, 50)
servo1.start(0)

def manual_mode():
    threshold_voltage = 1.85  # Sensor and requirements
    motor_running = False
    try:
        seconds = float(input("Enter the seconds you want the motor to stay open: "))
        while True:
            # Read the value from the infrared sensor
            bus.write_byte(address, A)
            value = bus.read_byte(address)
            voltage = (value * 3.3 / 255)
            print("AOUT:%1.3f " % voltage)
            
            if voltage > threshold_voltage and not motor_running:
                motor_running = True
                print("Infrared sensor triggered! Rotating the servo motor.")

                # Rotate to 180 degrees position
                servo1.ChangeDutyCycle(2)
                time.sleep(seconds)  # Wait for the servo to reach the position

                # Reset servo to initial position
                servo1.ChangeDutyCycle(7)
                time.sleep(1)
                servo1.ChangeDutyCycle(0)  # Stop sending the signal to the motor
                print("Waiting for sensor to clear...")
                time.sleep(2)

            elif voltage <= threshold_voltage:
                motor_running = False  # Reset the flag when the sensor is clear

            # Short delay before the next reading
            time.sleep(0.1)

    except ValueError:
        # Handle the error if the user does not enter a valid number
        print("Input error")
    except KeyboardInterrupt:
        # Clean up the GPIO on CTRL+C exit
        print("Exiting manual mode.")
        servo1.stop()
        GPIO.cleanup()

manual_mode()
