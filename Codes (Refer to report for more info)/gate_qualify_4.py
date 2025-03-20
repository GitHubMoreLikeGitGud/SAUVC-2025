import cv2 as cv
from ultralytics import YOLO
import numpy as np
import serial
import time

# ========== GLOBAL TIMER CONFIGURATION ==========
INITIAL_DELAY = 30      # Seconds before main loop starts
TO_DEPTH = 6            # Time to allow vertical run before horizontal
CALIBRATION_TIME = 16   # PID operation duration (seconds)
FORWARD_DURATION = 21    # Forward motion after calibration
UTURN_DURATION = 10      # U-turn phase duration before cutting thrusters

# ========== SERIAL CONFIGURATION ==========
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
time.sleep(2)
last_send_time = time.time()

pwm = (1500, 1500, 1500, 1500) # PWM values to send to teensy

# ========== PID CONTROLLER ==========
class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0

pid = PID(Kp=1.5, Ki=0.01, Kd=1) # Change PID values to calibrate PID
PWM_NEUTRAL = 1500
THRESHOLD = 8
ADJUSTMENT = 100 # How much PWM change relative from neutral

# ========== VISION COMPONENTS ==========
model = YOLO("./runs/detect/train/weights/best.pt")
cap = cv.VideoCapture(0)
frame_width = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))

# ========== STATE MANAGEMENT ==========
STATE_TO_DEPTH = 0
STATE_CALIBRATION = 1
STATE_FORWARD = 2
STATE_UTURN = 3
STATE_POST_UTURN = 4
current_state = STATE_TO_DEPTH
phase_start_time = 0

def compute_pwm_from_pid(m): # Thrusters going left or right
    left_front = PWM_NEUTRAL + (m/100)*ADJUSTMENT
    left_back = PWM_NEUTRAL + (m/100)*ADJUSTMENT
    right_front = PWM_NEUTRAL - (m/100)*ADJUSTMENT
    right_back = PWM_NEUTRAL - (m/100)*ADJUSTMENT
    return left_front, right_front, left_back, right_back

def get_box_center(x_min, y_min, x_max, y_max):
    return (x_min + x_max)//2, (y_min + y_max)//2

# Initial system delay
print(f"Initial calibration delay: {INITIAL_DELAY}s")
time.sleep(INITIAL_DELAY)

loop_start_time = time.time()
phase_start_time = time.time()

while True:
    isTrue, frame = cap.read()
    if not isTrue:
        break

    current_time = time.time()
    annotated_frame = frame.copy()
    
    # Process vision pipeline
    results = model(frame, conf=0.7)
    boxes = results[0].boxes.xyxy
    annotated_frame = results[0].plot()
    
    frame_center_x = frame.shape[1]//2
    frame_center_y = frame.shape[0]//2
    cv.circle(annotated_frame, (frame_center_x, frame_center_y), radius=5, color=(0, 255, 0), thickness=-1)

    # State transitions
    if current_state == STATE_TO_DEPTH:
        if current_time - phase_start_time > TO_DEPTH:
            current_state = STATE_CALIBRATION
            phase_start_time = current_time

    if current_state == STATE_CALIBRATION:
        if current_time - phase_start_time > CALIBRATION_TIME:
            current_state = STATE_FORWARD
            phase_start_time = current_time
            
    elif current_state == STATE_FORWARD:
        if current_time - phase_start_time > FORWARD_DURATION:
            current_state = STATE_UTURN
            phase_start_time = current_time
            
    elif current_state == STATE_UTURN:
        if len(boxes) > 0:
            current_state = STATE_POST_UTURN
            phase_start_time = current_time
            
    # Control logic
    if current_state == TO_DEPTH:
        pwm = (1500, 1500, 1500, 1500) # Neutral
        cv.putText(annotated_frame, "TO DEPTH", (50,50),
            cv.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)

    if current_state == STATE_CALIBRATION:
        if len(boxes) == 0:
            pwm = (1650, 1350, 1349, 1650) # Forward
            cv.putText(annotated_frame, "No Gate: Forward", (50,50), 
                      cv.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
            
        for box in boxes:
            x_min, y_min, x_max, y_max = map(int, box)  # Convert to integers
            center_x, center_y = get_box_center(x_min, y_min, x_max, y_max)

            # Draw a red circle at the detected box center
            cv.circle(annotated_frame, (center_x, center_y), radius=5, color=(0, 0, 255), thickness=-1)

            # Calculate error
            error = center_x - frame_center_x  # Horizontal distance from frame center
            error_percentage = (error / (frame_width // 2)) * 100

            if -THRESHOLD <= error_percentage <= THRESHOLD:
                scenario = "Gate is in the Middle"
                pid.integral = 0  # Reset integral term
                pwm = (1650, 1350, 1349, 1650) # Forward
            else: # Compute PID terms
                proportional = pid.Kp * error_percentage
                pid.integral += error_percentage  # Accumulate error in integral
                integral = pid.Ki * pid.integral
                derivative = pid.Kd * (error_percentage - pid.prev_error)
                pid.prev_error = error_percentage
                m = proportional + integral + derivative
                m = np.clip(m, -100, 100)
                pwm = compute_pwm_from_pid(m)    
                
    elif current_state == STATE_FORWARD:
        pwm = (1650, 1350, 1349, 1650) # Forward
        cv.putText(annotated_frame, "FORWARD PHASE", (50,50),
                  cv.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
        
    elif current_state == STATE_UTURN:
        pwm = (1540, 1540, 1460, 1460)  # Clockwise turn
        cv.putText(annotated_frame, "U-TURN SEARCH", (50,50),
                  cv.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)
        
    elif current_state == STATE_POST_UTURN:
        pwm = (1650, 1348, 1346, 1650) # Forward
        cv.putText(annotated_frame, "FINAL APPROACH", (50,50),
                  cv.FONT_HERSHEY_SIMPLEX, 1, (0,200,200), 2)

    # Display PWM values
    cv.putText(annotated_frame, f"LF: {pwm[0]:.0f}, RF: {pwm[1]:.0f}",
              (50,100), cv.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
    cv.putText(annotated_frame, f"LB: {pwm[2]:.0f}, RB: {pwm[3]:.0f}",
              (50,150), cv.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)

    # Serial communication
    if current_time - last_send_time >= 0.5:
        if (current_time - phase_start_time > UTURN_DURATION) & (current_state == STATE_POST_UTURN):
            pwm = (1500, 1500, 1500, 1500)
            data = [str(int(p)) for p in [*pwm, '000']]
        else:
            data = [str(int(p)) for p in [*pwm, '010']]
        ser.write(','.join(data).encode() + b'\n')
        last_send_time = current_time

    cv.imshow('Control Display', annotated_frame)
    if cv.waitKey(20) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()
