from PIL import Image
import heapq
from queue import PriorityQueue
import numpy as np
import cv2

# Load image
image = Image.open('C:/Users/peter/Documents/Bio inspired robotics/telloside/Pi/input.png')
if image.mode == 'CMYK':
    image = image.convert('RGB')
pixels = image.load()

# Define colours of interest
WALL_colour = (255, 165, 0)  # Orange
ROVER_colour = (255, 0, 0)  # Red
DESTINATION_colour = (0, 255, 0)  # Green
# calculate the colour difference between the pixel and the colours
red_colour = (255, 0, 0)
green_colour = (0, 255, 0)
orange_colour = (255, 165, 0)
# Find walls, rover and destination
walls = []
rover = None
destination = None



def compress_image(image, n):
    # divide the image into n x n blocks and compute the colour of each block
    block_width = image.width // n
    block_height = image.height // n

    pixel_matrix = np.asarray(image)

    blocks = []
    green_block_pixels = 0
    red_block_pixels = 0
    break_out_flag = False

    #traverse each block in the grid, and determine which sections are walls, rover or destination
    for i in range(n):
        for j in range(n):
            block = pixel_matrix[j*block_height:(j+1)*block_height,i*block_width:(i+1)*block_width]

            green_pixel_counter = 0
            red_pixel_counter = 0

            for row in block:             
                for pixel in row:
                    
                    #calculate if pixel is similar colour to bold red green and orange
                    red_difference = abs(pixel[0] - red_colour[0]) + abs(pixel[1] - red_colour[1]) + abs(pixel[2] - red_colour[2])
                    green_difference = abs(pixel[0] - green_colour[0]) + abs(pixel[1] - green_colour[1]) + abs(pixel[2] - green_colour[2])
                    orange_difference = abs(pixel[0] - orange_colour[0]) + abs(pixel[1] -orange_colour[1]) + abs(pixel[2] - orange_colour[2])

                    #handle each colour appropriately
                    if red_difference < 200:
                        rover = (j, i)
                        red_pixel_counter += 1
                    elif green_difference < 200:
                        green_pixel_counter += 1
                    elif orange_difference < 163:
                        walls.append((j, i))
                        break_out_flag = True
                        break

                if break_out_flag:
                    break_out_flag = False
                    break
  
            #determine block that is most covered by green
            if green_pixel_counter >= green_block_pixels:
                destination = (j, i)
                green_block_pixels = green_pixel_counter

            #determine block most covered by red
            if red_pixel_counter >= red_block_pixels:
                rover = (j, i)
                red_block_pixels = red_pixel_counter



    


    compressed_image = np.zeros((n,n))
    #draw objects on compressed image
    for wall in walls:
        compressed_image[wall[0]][wall[1]] = 1
    
    compressed_image[rover[0]][rover[1]] = 2

    compressed_image[destination[0]][destination[1]] = 3
 
    return compressed_image, rover, destination

def heuristic(a, b):
    # calculate the Manhattan distance between two points
    return abs(b[0] - a[0]) + abs(b[1] - a[1])

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

def find_path(image, compress_factor=10):
    #compress image
    compressed_image, start, dest = compress_image(image, 20)
    print(compressed_image)

    #find path
    path = astar(compressed_image, start, dest)

    #draw computed path on original image
    block_width = image.width // 20
    block_height = image.height // 20
    for j, i in path:
        for y in range(j*block_height,(j+1)*block_height):
            for x in range(i*block_width, (i+1)*block_width):
                pixels[x,y] = (255, 255, 0)

    image.save('output.png')

find_path(image)