import Jetson.GPIO as GPIO
import time

# Set up the GPIO mode
GPIO.setmode(GPIO.BOARD)

# Choose a pin (example: pin 13)
output_pin_1 = 13
output_pin_2 = 15

# Set up the pin as output
GPIO.setup(output_pin_1, GPIO.OUT)

try:
	print("Reset Hori")
	# Set HIGH (3.3V)
	GPIO.output(output_pin_1, GPIO.HIGH)
	time.sleep(2)
	# Set LOW (0V)
	GPIO.output(output_pin_1, GPIO.LOW)
	time.sleep(2)
		
	print("Reset Vert")
	GPIO.setup(output_pin_2, GPIO.OUT)
	# Set HIGH (3.3V)
	GPIO.output(output_pin_2, GPIO.HIGH)
	time.sleep(2)
	# Set LOW (0V)
	GPIO.output(output_pin_2, GPIO.LOW)
	time.sleep(2)
    
finally:
    # Clean up GPIO on program exit
    GPIO.cleanup()