import serial
import time
import cv2

# Configure the serial connection to communicate with horizontal teensy
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
time.sleep(2)  # Allow time for the connection to establish

# Initialize OpenCV video capture
cap = cv2.VideoCapture(0)

# Toggle commands for horizontal
commands = {
    'c': [1550, 1550, 1450, 1450], # Yaw Right
    'a': [1400, 1600, 1400, 1600], # Strafe Left
    'z': [1450, 1450, 1550, 1550], # Yaw Left
    'd': [1600, 1400, 1600, 1400], # Strafe Right
    'w': [1650, 1350, 1350, 1650], # Forward
    's': [1350, 1650, 1650, 1350], # Backwards
    'default': [1500, 1500, 1500, 1500] # Neutral
}

# Track current command and binary kill switch value
current_command = 'default'
kill_switch_value = '000'

# Timer for controlling USB serial send interval
last_send_time = time.time()

# Initialize variables for video recording
is_recording = False
out = None
recording_count = 1  # To keep track of the number of recordings

try:
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        # Display the resulting frame
        cv2.imshow('Camera', frame)

        # Check for keyboard input
        key = cv2.waitKey(1) & 0xFF

        if key == ord('w'):
            current_command = 'w'

        elif key == ord('a'):
            current_command = 'a'
            
        elif key == ord('s'):
            current_command = 's'

        elif key == ord('d'):
            current_command = 'd'

        elif key == ord('z'):
            current_command = 'z'
            
        elif key == ord('c'):
            current_command = 'c'
            
        elif key == ord(' '):
            current_command = 'default'
                        
        elif key == ord('p'):
            # Toggle recording functionality
            if is_recording:
                print(f"Stopping recording {recording_count}...")
                is_recording = False
                out.release()
                out = None
                recording_count += 1
            else:
                print(f"Starting recording {recording_count}...")
                is_recording = True
                # Define the codec and create a VideoWriter object
                fourcc = cv2.VideoWriter_fourcc(*'XVID')
                filename = f'2425S2_{recording_count}.avi'
                out = cv2.VideoWriter(filename, fourcc, 20.0, (640, 480))

        elif key == ord('r'):
            kill_switch_value = '000' # No vertical thrust
            
        #elif key == ord('t'):
            #kill_switch_value = '001' # Set depth to 1.0m

        elif key == ord('f'):
            kill_switch_value = '010' # Set depth to 1.5m

        #elif key == ord('u'):
            #kill_switch_value = '011' # Set depth to 2m

        #elif key == ord('i'):
            #kill_switch_value = '100' # Does nothing

        elif key == ord('q'):
            break

        # Write the frame to the video file if recording
        if is_recording:
            out.write(frame)

        # Check if it's time to send new values over USB serial
        current_time = time.time()
        if current_time - last_send_time >= 0.5:
            data_to_send = commands.get(current_command, commands['default']) + [kill_switch_value]
            ser.write(','.join(map(str, data_to_send)).encode() + b'\n')
            print(f"Sent: {data_to_send}")

            ser.flush()
            last_send_time = current_time

except KeyboardInterrupt:
    print("Interrupted by user")

finally:
    # Release the camera and close serial port and video writer if open
    cap.release()
    if out:
        out.release()
    cv2.destroyAllWindows()
    ser.close()
    print("Serial port and camera closed")