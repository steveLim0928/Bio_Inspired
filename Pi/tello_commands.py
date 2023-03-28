from djitellopy import Tello
import time
import math
import cv2
import numpy as np
from path_computation import find_path


# Takes photo using bottom camera, saves as image parameter name
def take_photo(tello, image):
        time.sleep(1)


        # get distances before photo
        pad = tello.get_mission_pad_id()
        dist_x = tello.get_mission_pad_distance_x()
        dist_y = tello.get_mission_pad_distance_y()
        dist_z = tello.get_mission_pad_distance_z()

        print("Distance` pad in X: ",  dist_x, "Y: ", dist_y, "Z: ", dist_z)
        print("PAD DETECTED: ", pad)

        # take a picture with the bottom camera
        frame = tello.get_frame_read().frame

        # save the picture to a file
        cv2.imwrite(image, frame)


        # get distances after photo
        pad = tello.get_mission_pad_id()
        dist_x2 = tello.get_mission_pad_distance_x()
        dist_y2 = tello.get_mission_pad_distance_y()
        dist_z2 = tello.get_mission_pad_distance_z()
        print("Distance` pad in X: ",  dist_x2, "Y: ", dist_y2, "Z: ", dist_z2)
        print("PAD DETECTED: ", pad)
        
        time.sleep(1)


        # return average distance away from floor
        return (dist_z + dist_z2) / 2


def land(tello):

        # update current distance
        pad = tello.get_mission_pad_id()
        dist_x = tello.get_mission_pad_distance_x()
        dist_y = tello.get_mission_pad_distance_y()
        dist_z = tello.get_mission_pad_distance_z()
        print("Distance` pad in X: ",  dist_x, "Y: ", dist_y, "Z: ", dist_z)
        print("PAD DETECTED: ", pad)

        # move drone until over center of rover and at specified height
        while dist_z > 50 or (dist_y > 3 or dist_y < -3) or (dist_x > 3 or dist_x < -3) :

                # detect rover
                pad = tello.get_mission_pad_id()

                # if no rover detected move higher
                if pad == -1:
                    tello.move_up(20)

                # if  rover detected, move over the top of it
                else:  
                    # if drone is too high lower it.    
                    if dist_z >= 50:
                        tello.go_xyz_speed_mid(0, 0, dist_z - 10, 10, pad)
                    # once height reaches specified parameter, stop lowering drone
                    else:
                        tello.go_xyz_speed_mid(0, 0, dist_z, 10, pad)

                time.sleep(1)


                # update distances
                pad = tello.get_mission_pad_id()
                dist_x = tello.get_mission_pad_distance_x()
                dist_y = tello.get_mission_pad_distance_y()
                dist_z = tello.get_mission_pad_distance_z()
                print("Distance` pad in X: ",  dist_x, "Y: ", dist_y, "Z: ", dist_z)
                print("PAD DETECTED: ", pad)
            
        tello.land()


# this function creates a list of movement commands for use with the drone mover_right move_forwards commands
# to implement, still require if statements to deal with negative or positive (e.g. move right or left)
def move_drone(x, y):
        
        #movement commands have to be greater than 20, so to bring drone to exact center, if drone is within 20, it moves 20 away, to then move the exact amount to the center
        movements = []
        while x >= 4 or x <= -4 and y > 4 or y <= -4:
            if x > 0:
                if x < 20:
                    movements.append((20, 0))
                    x += 20
                    movements.append((-(x), 0))
                    x = 0
                else:
                    movements.append((-20, 0))
                    x -= 20
            if x < 0:
                if x > -20:
                    movements.append((20, 0))
                    x -= 20
                    movements.append(((x), 0))
                    x = 0
                else:
                    movements.append((-20, 0))
                    x += 20
            
            if y > 0:
                if y < 20:
                    movements.append((0, 20))
                    y += 20
                    movements.append((0, -(y)))
                    y = 0
                else:
                    movements.append((0, -20))
                    y -= 20
            if y < 0:
                if y > -20:
                    movements.append((0, 20))
                    y += 20
                    movements.append((0, (y)))
                    y = 0
                else:
                    movements.append((0, -20))
                    y += 20            
        
        return movements



# runs all required drone code
def run_drone():
    fx = 433.44868
    fy = 939.895
    cx = 107
    cy = 318.9
    camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)
    dist_coeffs = np.array([0.8333, 0.699, 0.455, 0.00159, -0.94509282], dtype=np.float32)


    # Initialise drone
    tello = Tello()
    tello.connect()

    # configure drone
    tello.enable_mission_pads()
    tello.streamon()
    tello.set_video_direction(1)
    tello.set_mission_pad_detection_direction(2)  # 

    # takeoff
    tello.takeoff()

    image = "bottom_camera.jpg"
    height = take_photo(tello, image)
    #height = 80
    print("HEIGHT: ", height)
    
    #lands drone
    land(tello)


    #computes path
    #parameters: image path, height of image, rover aruco id, destination aruco id, wall aruco id, camera matrix, distortion coefficients, size of grid to segment image into for path computation
    path = find_path(image, height, 0, 1, 2, camera_matrix, dist_coeffs, compress_factor=30)


    print("PATH: ", path)

            


    #tello.disable_mission_pads()
    #tello.land()
    #tello.streamoff()
    #tello.end()


try:
   
   run_drone()
except KeyboardInterrupt:
    tello.land()





