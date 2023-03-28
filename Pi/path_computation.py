from PIL import Image
import heapq
from queue import PriorityQueue
import numpy as np
import cv2
# Load image
import cv2.aruco as aruco



#Computes real world distance of given pixels
def compute_distance(pixel_x, pixel_y, camera_matrix, dist_coeffs, z):

    # Define the world coordinates
    world_coords = np.array([[pixel_x, pixel_y]], dtype=np.float32)

    # Undistort the image coordinates   
    undistorted_coords = cv2.undistortPoints(world_coords, camera_matrix, dist_coeffs)

    #undistorted_coords = undistorted_coords.reshape((2, 1))
    undistorted_coords = undistorted_coords.reshape((2, 1))
    undistorted_coords = undistorted_coords.T
    undistorted_coords = np.append(undistorted_coords, [[1]], axis=1)

    # Calculate the perspective transformation matrix
    perspective_matrix = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 0]], dtype=np.float32)
    perspective_matrix[2][2] = 1.0 / z
    print(perspective_matrix)

    # Apply the perspective transformation matrix to the undistorted coordinates
    transformed_coords = np.dot(perspective_matrix, undistorted_coords.T)
    x = transformed_coords[0][0]  * z
    y = transformed_coords[1][0]  * z


    return x,y


# assigns rover, wall or destination or neither to blocks of the segmented grid of the picture
def pixel_assignments(image_path, rover_marker_ID, wall_marker_ID, destination_marker_ID, camera_matrix, distortion_coefficients):
    
    # Load the image
    image = cv2.imread(image_path)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Define the aruco dictionary and parameters
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
    aruco_params = cv2.aruco.DetectorParameters()

    # Detect the aruco markers in the image
    corners, ids, rejected = cv2.aruco.detectMarkers(gray,aruco_dict, camera_matrix, distortion_coefficients)

    # Get the corners of the rover, wall, and destination markers
    if rover_marker_ID in ids:
        print("Corners: ", corners)
        rover_corners = corners[np.where(ids == rover_marker_ID)[0][0]]
    else:
        print("Rover marker ID not found")
        rover_corners = []

    if wall_marker_ID in ids:
        wall_corners = corners[np.where(ids == int(wall_marker_ID))[0][0]]
    else:
        print("wall marker ID not found")
        wall_corners = []
 
    if destination_marker_ID in ids:
        destination_corners = corners[np.where(ids == int(destination_marker_ID))[0][0]]
    else:
        print("wall marker ID not found")
        destination_corners = []
    

    # Convert the corners to pixel coordinates
    wall_pixels = [np.int32(corner.squeeze()) for corner in wall_corners]
    rover_pixels = [np.int32(corner.squeeze()) for corner in rover_corners]
    destination_pixels = [np.int32(corner.squeeze()) for corner in destination_corners]

    # Return the pixel coordinates for each marker type
    return wall_pixels, rover_pixels, destination_pixels    




#reduces image to grid with size n x n
def compress_image(image, image_path, n, rover_marker_ID, wall_marker_ID, destination_marker_ID, camera_matrix, distortion_coefficients):
    
    # divide the image into n x n blocks and compute the colour of each block
    block_width = image.width // n
    block_height = image.height // n

    # create 2D array of pixels
    pixel_matrix = np.asarray(image)

    blocks = []
    destination_block_pixels = 0
    rover_block_pixels = 0

    #if wall is seen in block, entire block is defined as wall and exists loop of block
    break_out_flag = False
    
    #assigns pixels as walls, rover or destination
    wall_pixels, rover_pixels, destination_pixels = pixel_assignments(image_path, rover_marker_ID, wall_marker_ID, destination_marker_ID, camera_matrix, distortion_coefficients)
    
    # traverse each block in the grid, and determine which sections are walls, rover or destination
    walls = []
    for i in range(n):
        for j in range(n):
            block = pixel_matrix[j*block_height:(j+1)*block_height,i*block_width:(i+1)*block_width]

            # Counts number of pixels in the block
            rover_pixels_counter = 0
            destination_pixels_counter = 0
            
            # For each pixel in the block:
            for y in range(j*block_height,(j+1)*block_height):
                for x in range(i*block_width, (i+1)*block_width):
                    
                    pixel = (x, y)

                    # counts number of rover pixels
                    if pixel in map(tuple, rover_pixels[0]):
                        rover_pixels_counter += 1

                    # counts number of destination pixels
                    elif pixel in map(tuple, destination_pixels[0]):
                        destination_pixels_counter += 1

                    # if wall pixel, block is avoided
                    elif len(wall_pixels) > 0:
                        if pixel in map(tuple, wall_pixels[0]):
                            walls.append((j, i))
                            break_out_flag = True
                            break
                
                # breaks when wall is detected
                if break_out_flag:
                    break_out_flag = False
                    break
  
            #determine block that is most covered by destination
            if destination_pixels_counter > destination_block_pixels:

                destination = (j, i)
                destination_block_pixels = destination_pixels_counter

            #determine block most covered by rover
            if rover_pixels_counter > rover_block_pixels:
                rover = (j, i)
                rover_block_pixels = rover_pixels_counter

    compressed_image = np.zeros((n,n))

    #draw objects on compressed image
    for wall in walls:
        compressed_image[wall[0]][wall[1]] = 1
    
    compressed_image[rover[0]][rover[1]] = 2

    compressed_image[destination[0]][destination[1]] = 3
 
    #returns image as a 2D array of size n x n, with 1 representing walls, 2 representing rover, 3 representing destination, and 0 representing space.
    return compressed_image, rover, destination


# calculate the Manhattan distance between two points
def heuristic(a, b):
    return abs(b[0] - a[0]) + abs(b[1] - a[1])

#path finding algorithm given 2D array of 
def astar(array, start, dest):
    # initialize the priority queue
    pq = PriorityQueue()
    pq.put((0, start))

    # initialize the distance and visited arrays
    dist = {start: 0}
    visited = {start: None}

    while not pq.empty():
        # get the cell with the lowest f-score
        curr_cost, curr = pq.get()

        # if we've reached the destination, reconstruct the path and return it
        if curr == dest:
            path = []
            while curr is not None:
                path.append(curr)
                curr = visited[curr]
            path.reverse()
            return path

        # check the neighbors of the current cell
        for next in [(0, 1), (0, -1), (1, 0), (-1, 0), (-1,-1), (1, 1)]:
            neighbor = (curr[0] + next[0], curr[1] + next[1])
            # check if the neighbor is inside the array bounds
            if neighbor[0] < 0 or neighbor[0] >= array.shape[0] or neighbor[1] < 0 or neighbor[1] >= array.shape[1]:
                continue
            # check if the neighbor is a wall cell
            if array[neighbor] == 1:
                continue
            # calculate the tentative distance from the start to the neighbor through the current cell
            tentative_dist = dist[curr] + 1
            # update the neighbor's distance if it's lower than the current distance
            if neighbor not in dist or tentative_dist < dist[neighbor]:
                dist[neighbor] = tentative_dist
                priority = tentative_dist + heuristic(dest, neighbor)
                pq.put((priority, neighbor))
                visited[neighbor] = curr

    # if we haven't found the destination, return None
    return None



# finds real world distance of series of movements from rover to destination avoiding walls
def find_path(image_path, height, r_ID, d_ID, w_ID, camera_matrix, dist_coefficients,compress_factor=20):

    # open image 
    image = Image.open(image_path)
    if image.mode == 'CMYK':
        image = image.convert('RGB')
    pixels = image.load()

    walls = []
    rover = None
    destination = None

    # compress image
    compressed_image, start, dest = compress_image(image, image_path, compress_factor, r_ID, d_ID, w_ID, camera_matrix, dist_coefficients)
    
    # find path  
    path = astar(compressed_image, start, dest)

    # draw computed path on original image
    block_width = image.width // compress_factor
    block_height = image.height // compress_factor
    for j, i in path:
        for y in range(j*block_height,(j+1)*block_height):
            for x in range(i*block_width, (i+1)*block_width):
                pixels[x,y] = (255, 255, 0)
    
    
    # find distance away from camera of starting block - set to initial distance
    block_centre_y = (path[0][0]*block_height + (path[0][0]+1) * block_height  - 1) // 2
    block_centre_x = (path[0][1]*block_width + (path[0][1]+1) * block_width - 1) // 2
    dist_from = compute_distance(block_centre_x, block_centre_y, camera_matrix, dist_coefficients, height)

    path_distances = []

    # finds difference in real world distance of center of next block in path from previous block in path.
    for j, i in path:
        block_centre_y = (j*block_height + (j+1) * block_height  - 1) // 2
        block_centre_x = (i*block_width + (i+1) * block_width - 1) // 2

        # computes real world distance of center of block - real world distance of center of last block
        # appends to path_distances, containing all distances to travel
        path_distances.append(np.array(compute_distance(block_centre_x, block_centre_y, camera_matrix, dist_coefficients, height)) - np.array(dist_from))
        dist_from = compute_distance(block_centre_x, block_centre_y, camera_matrix, dist_coefficients, height)

    image.save('output.png')

    return path_distances

