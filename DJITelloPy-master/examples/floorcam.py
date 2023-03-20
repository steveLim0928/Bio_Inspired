import cv2
from djitellopy import Tello

tello = Tello()
tello.connect()
#tello.set_video_encoder_rate(2)
tello.streamon()

# Set video direction to downwards
tello.set_video_direction(1)

# Initialize OpenCV video stream
cv2.namedWindow("Drone Feed")
drone_feed = cv2.VideoCapture("udp://0.0.0.0:11111")

# Loop to continuously receive and display video frames
while True:
    ret, frame = drone_feed.read()
    if not ret:
        break

    cv2.imshow("Drone Feed", frame)

    # Press 'q' to exit the video stream
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
drone_feed.release()
cv2.destroyAllWindows()
tello.streamoff()
tello.land()