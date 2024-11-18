from collections import defaultdict
import cv2
import numpy as np
from ultralytics import YOLO
import serial
import time

model_path = r'runs\detect\train3(200epochs)\weights\best.pt'
serial_port = 'COM5'
baud_rate = 9600
serial_timeout = 1
esp32_camera_url = "http://192.168.118.121:81/stream"

# Buffer zone parameters
BUFFER_WIDTH = 50  # Width of the buffer zone in pixels
BUFFER_HEIGHT = 50  # Height of the buffer zone in pixels

# Function to adjust servo angle
def adjust_servo_angle(current_angle, step, direction):
    if direction == "increase":
        return min(170, current_angle + step)
    elif direction == "decrease":
        return max(10, current_angle - step)
    return current_angle

# Initialize serial connection
try:
    ser = serial.Serial(serial_port, baud_rate, timeout=serial_timeout)
    time.sleep(2)
    print(f"Serial connection established on {serial_port} at {baud_rate} baud.")
except serial.SerialException as e:
    print(f"Error: Could not open serial port {serial_port}: {e}")
    ser = None

# Load YOLO model
model = YOLO(model_path)
print(f"Loaded YOLOv8 model from {model_path}")

# Open video stream
cap = cv2.VideoCapture(esp32_camera_url)
if not cap.isOpened():
    print(f"Error: Cannot open video stream at {esp32_camera_url}")
    exit()

print("Video stream opened successfully.")
track_history = defaultdict(lambda: [])

# Initial servo angles
servo_x_angle = 90
servo_y_angle = 90
step_size = 2  # Step size for each servo movement

try:
    while cap.isOpened():
        success, frame = cap.read()
        if not success:
            print("Failed to grab frame.")
            break

        results = model.track(frame,persist=True)

        if results and results[0].boxes.id is not None:
            # Filter for class 0 (person)
            persons = [
                (box, track_id)
                for box, cls, track_id in zip(results[0].boxes.xywh.cpu().numpy(),
                                              results[0].boxes.cls.cpu().numpy(),
                                              results[0].boxes.id.int().cpu().tolist())
                if cls == 0
            ]
            print(persons)


            if persons:
                # Track only the first detected person in class 0
                
                box, track_id = persons[0]
                print('box ',box)

                frame_height, frame_width, _ = frame.shape
                center_x = frame_width // 2
                center_y = frame_height // 2

                x, y, w, h = box
                x_center = x
                y_center=y
                # if(h>w):
                #     y_center = y-y/5
                # else:
                #     y_center=y


                # Check if the person is outside the buffer zone
                if x_center < center_x - BUFFER_WIDTH / 2:
                    # Move servo to the left
                    servo_x_angle = adjust_servo_angle(servo_x_angle, step_size, "decrease")
                elif x_center > center_x + BUFFER_WIDTH / 2:
                    # Move servo to the right
                    servo_x_angle = adjust_servo_angle(servo_x_angle, step_size, "increase")

                if y_center < center_y - BUFFER_HEIGHT / 2:
                    # Move servo up
                    servo_y_angle = adjust_servo_angle(servo_y_angle, step_size, "increase")
                elif y_center > center_y + BUFFER_HEIGHT / 2:
                    # Move servo down
                    servo_y_angle = adjust_servo_angle(servo_y_angle, step_size, "decrease")

                print(f"Track ID: {track_id} -> Servo X: {servo_x_angle}, Servo Y: {servo_y_angle}")

                # Send command to servo if connected
                if ser and ser.is_open:
                    try:
                        command = f"{servo_x_angle},{servo_y_angle}\n"
                        ser.write(command.encode('utf-8'))
                    except serial.SerialException as e:
                        print(f"Serial write error: {e}")
                else:
                    print("Person within buffer zone; no servo movement.")

                track = track_history[track_id]
                track.append((float(x_center), float(y_center)))
                if len(track) > 30:
                    track.pop(0)

                cv2.rectangle(frame, (int(x-w/2), int(y-h/2)), (int(x + w/2), int(y + h/2)), (0, 255, 0), 2)
                cv2.circle(frame, (int(x_center), int(y_center)), 5, (0, 0, 255), -1)
                cv2.putText(frame, f"ID: {track_id}", (int(x), int(y) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        else:
            print("No person detected.")

        cv2.imshow('YOLOv8 Tracking', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Quitting...")
            break

except KeyboardInterrupt:
    print("Interrupted by user.")

cap.release()
cv2.destroyAllWindows()

if ser and ser.is_open:
    ser.close()
    print("Serial connection closed.")
