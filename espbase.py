import cv2

esp32_camera_url = "http://192.168.59.1/"
cap = cv2.VideoCapture(esp32_camera_url)

if not cap.isOpened():
    print("Error: Could not open video stream")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to read frame")
        break

    cv2.imshow('ESP32 Stream', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
