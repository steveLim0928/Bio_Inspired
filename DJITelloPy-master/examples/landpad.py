from djitellopy import Tello
import time
import math
import cv2
# create and connect
tello = Tello()
tello.connect()

try:
    # Set the IMU calibration
    #tello()

    # Wait for the calibration to complete


    # configure drone
    tello.enable_mission_pads()

    tello.streamon()
    tello.set_video_direction(1)

    # takeoff
    tello.takeoff()


    tello.set_mission_pad_detection_direction(2)  # 





    def round_to_nearest_5(x):
        return round(x / 5) * 5


    def move_drone(x, y):
        movements = []
        while x > 4 or x < -4 and y > 4 or y < -4:
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
    pad = tello.get_mission_pad_id()
    print(pad)

    dist_x = tello.get_mission_pad_distance_x()
    dist_y = tello.get_mission_pad_distance_y()
    dist_z = tello.get_mission_pad_distance_z()


    print("Distance` pad in X: ",  dist_x, "Y: ", dist_y, "Z: ", dist_z)
    print("PAD DETECTED: ", pad)

    # take a picture with the bottom camera
    frame = tello.get_frame_read().frame

    # save the picture to a file
    cv2.imwrite("bottom_camera.jpg", frame)


    pad = tello.get_mission_pad_id()



    dist_x = tello.get_mission_pad_distance_x()
    dist_y = tello.get_mission_pad_distance_y()
    dist_z = tello.get_mission_pad_distance_z()
    print("Distance` pad in X: ",  dist_x, "Y: ", dist_y, "Z: ", dist_z)
    print("PAD DETECTED: ", pad)
    time.sleep(1)


    last_z = 100
    while dist_z > 40 or (dist_y > 4 or dist_y < -4) or (dist_x > 4 or dist_x < -4):
        #if detected_pads == [3]:
        #   tello.move_back(30)
        #    tello.rotate_clockwise(90)

        #if detected_pads == [4]:
        #    tello.move_up(30)
        #    tello.flip_forward()

        # Get distance from pad


            pad = tello.get_mission_pad_id()

        
            if pad == -1:
                tello.move_up(20)
            else: 

                # Move towards pad
                #if dist_z > 70  and dist_y < 5 and dist_x < 5 and dist_x > -5 and dist_y > -5:
                #   print(dist_z)
                #    tello.move_down(20)

            
                
                if dist_z > 25:

            
                    tello.go_xyz_speed_mid(0, 0, dist_z - 10, 10, pad)
                else:
                    tello.go_xyz_speed_mid(0, 0, dist_z, 10, pad)

                #tello.go_xyz_speed_mid(0, dist_y, dist_z, 10, pad)
                #tello.go_xyz_speed_mid(dist_x, dist_y, dist_z, 10, pad)
                #if dist_z < 80:
                #    tello.go_xyz_speed(dist_x, dist_y, dist_z, 10)        
                #else:
                #    tello.go_xyz_speed(dist_x, dist_y, dist_z, 10)        
                    

                time.sleep(1)


            
            pad = tello.get_mission_pad_id()

            
            dist_x = tello.get_mission_pad_distance_x()
            dist_y = tello.get_mission_pad_distance_y()
            dist_z = tello.get_mission_pad_distance_z()
            print("Distance` pad in X: ",  dist_x, "Y: ", dist_y, "Z: ", dist_z)
            print("PAD DETECTED: ", pad)
            


            
            '''else:




                movements = move_drone(dist_y, dist_x)

                for movement in movements:
                    if movement[1] != 0:
                        if movement[1] > 0:
                            tello.move_left(movement[1])
                        else:
                            tello.move_right(-movement[1])

                    else:
                        if movement[0] > 0:
                            tello.move_forward(movement[0])
                        else:
                            tello.move_back(-movement[0])

                

            pad = tello.get_mission_pad_id()

            dist_x = tello.get_mission_pad_distance_x()
            dist_y = tello.get_mission_pad_distance_y()
            dist_z = tello.get_mission_pad_distance_z()
            print("Distance` pad in X: ",  dist_x, "Y: ", dist_y, "Z: ", dist_z)
            print("PAD DETECTED: ", pad)
        '''
            
        

    
        



    
        

            


    # land on the detected pad
    #tello.land_on_pad(1)

    # graceful termination
    print("STOP")
    tello.disable_mission_pads()
    tello.land()
    tello.streamoff()
    tello.end()
except KeyboardInterrupt:
    tello.land()
    print("NOT WORKING2")