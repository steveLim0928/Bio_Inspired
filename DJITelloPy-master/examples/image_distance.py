import cv2
import numpy as np
# Camera intrinsics
#fx = 220.915
#fy = 220.915
#cx = 157.073
#cy = 119.555

fx = 3.45328364e+09
fy = 8.42721069e+02
cx = 157.073
cy = 224.555
camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
distortion_coefficients = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

# Size of Aruco marker in millimeters
marker_size = 15.24
board_size = (4, 4)

# Load image from file
img = cv2.imread("bottom_camera2.jpg")
print(img)
# Detect the marker in the image
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(img)

# Extract the position and orientation of the marker
print(markerCorners)
if len(markerCorners) > 0:
    object_points = np.array([[0, 0, 0], [marker_size, 0, 0], [marker_size, marker_size, 0], [0, marker_size, 0]], dtype=np.float32)

    print(img.shape[1::-1])
    print(object_points.shape)
    print(np.shape(markerCorners))

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objectPoints=[object_points],
                                                    imagePoints=markerCorners,
                                                    imageSize=img.shape[1::-1],
                                                    cameraMatrix=None,
                                                    distCoeffs=None)
    print(mtx)
    print(dist)

    print(tvecs)
    #rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorners, marker_size, camera_matrix, distortion_coefficients)
    booleanVal, rvec, tvecs = cv2.solvePnP(object_points, markerCorners[0], camera_matrix, distortion_coefficients)

    print(tvecs)
    x = tvecs[0][0]
    y = tvecs[1][0]
    z = tvecs[2][0]
    print("Marker position: ({:.2f}, {:.2f}, {:.2f})".format(x, y, z))

    # Calculate the x and y distances
    drone_height = 84    # height of drone in cm
    x_distance = x * (drone_height / z)
    y_distance = y * (drone_height / z)
    #print("Distance to marker: {:.2f} cm (x), {:.2f} cm (y)".format(x_distance, y_distance))
    print("Distance to marker: {:.2f} cm (x), {:.2f} cm (y), {:.2f} cm (z)".format(x, y, drone_height))
