from djitellopy import Tello
import time

# create and connect
tello = Tello()
tello.connect()

# configure drone
tello.enable_mission_pads()

# takeoff
tello.takeoff()
time.sleep(5)

# get detected mission pads
detected_pads = tello.get_mission_pad_id()
dist_x = tello.get_mission_pad_distance_x()
dist_y = tello.get_mission_pad_distance_y()
dist_z = tello.get_mission_pad_distance_z()
# detect and react to pads until we see pad #1
dist_z = 100
tello.set_mission_pad_detection_direction(2)  # 


print(dist_z)
print(dist_x)
print(dist_y)


while dist_z != 0 or (dist_y > 20 or dist_y < -20) or (dist_x > 20 or dist_x < -20):
    #if detected_pads == [3]:
     #   tello.move_back(30)
    #    tello.rotate_clockwise(90)

    #if detected_pads == [4]:
    #    tello.move_up(30)
    #    tello.flip_forward()

    # Get distance from pad
    try:
        pad = tello.get_mission_pad_id()

        dist_x = tello.get_mission_pad_distance_x()
        dist_y = tello.get_mission_pad_distance_y()
        dist_z = tello.get_mission_pad_distance_z()
        (f"Distance        ` pad in X: {dist_x} cm, Y: {dist_y} cm, Z: {dist_z} cm")

        print("PAD DETECTED: ", pad)
        if pad == -1:
            tello.move_up(20)
        else: 
            tello.go_xyz_speed_mid(dist_x, dist_y, dist_z, 10, pad)
            #tello.go_xyz_speed(dist_x, dist_y, dist_z, 10)




        '''

        elif dist_z > 30:
            tello.move_down(20)
        elif dist_z < 30:
            tello.land()
        '''
    except KeyboardInterrupt:
        tello.land()
        print("NOT WORKING2")

    # Move towards pad
    '''if dist_y > 20:
        tello.move_forward(20)
    elif dist_y < -20:
        tello.move_back(20)

    if dist_x > 20:
        tello.move_right(20)
    elif dist_x < -20:
        tello.move_left(20)
    '''
    
    

        


# land on the detected pad
#tello.land_on_pad(1)

# graceful termination
print("STOP")
tello.disable_mission_pads()
tello.land()
tello.end()
