import cv2
from ultralytics import YOLO

# Load the YOLO model
model = YOLO(r'runs\detect\train3(200epochs)\weights\best.pt')

# Replace with your ESP32 camera stream URL
esp32_camera_url = "http://192.168.116.121:81/stream"

# Initialize the tracker (show=False to manually handle display)
# tracker = model.track(source='0', show=True, device=0)
tracker = model.track(source=esp32_camera_url, show=True, device=0)

# Run the tracker and display the video stream manually
for frame, result in tracker:
    # Display the frame
    cv2.imshow('Tracking', frame)

    # Print the results
    print(result)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
