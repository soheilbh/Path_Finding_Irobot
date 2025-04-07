import heapq
import cv2 as cv

import numpy as np
import matplotlib.pyplot as plt
import scipy as sp
import matplotlib.colors as mcolors
aruco = cv.aruco
import math


'''
1. transform_and_convert_image_to_matrix(image_path, transforming=True, transformed_file_name='maze',
                                          maze_height=maze_height, maze_width=maze_width)                                         
2. imshow_maze_matrix(maze)
3. maze_astar_solver(maze, start, goal, buffer_size=1)
4. summarize_path(directions)
5. clean_up_path(moves, start=start,  move_size_limit=15)
6. visualize_paths(path, directions, 
                    start_point, maze_grid, output_image_filename, fig_size=10, show_maze=True,
                    print_path=True, print_directions=True, 
                    show_summarized_path=True, show_cleaned_path=True, 
                    print_summarized_path_info=True, print_cleaned_path_info=True)
7. fit_calibration_curve(sensor_values, actual_distances, sensor_id, calibration_coefficients=None, polynomial_degree=1)
8. calculate_calibrated_distance(sensor_reading_tuple, calibration_coefficients)                    
9. isualize_sensor_data(sensor_list, actual_distances, selected_sensors)
10. plot_wall_conditions(maze, wall_statuses)
11. class VirtualRobot:
    def __init__(self, maze, start_x, start_y, end_x, end_y, 
                 astar_path, astar_path_bandwidth, 
                 sensor_range=1):
12. is_within_astar_boundaries(robot, path, distance, angle)
13. follow_left_wall(robot, path, desired_left_distance=12, rotation_angle=15,
                        move_distance=1, robot_sensor_angle=65)
14. angle_between_vectors(A, B)
15. is_within_distance_to_goal(x1, y1, x2, y2, threshold)
16. move_robot_to_goal(my_robot, goal)
17. robot_run(robot, path, goal, move_count=2000)
18. visualize_robot_path(maze,robot,fig_size)

'''


######      transform and converts a maze image to a matrix 
def transform_and_convert_image_to_matrix(image_path, transforming=True,
                                          transformed_file_name='maze',
                                          maze_height=390, maze_width=450):

    image = cv.imread(image_path)   # Read image
    transformed_image_filename = f"transformed-{transformed_file_name}.png"

    if transforming :
        # make ArUco dictionary and detection parameters
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters()
        corners, ids, rejected = aruco.detectMarkers(image, aruco_dict, parameters=parameters) # Detect markers in the image
    
        if ids is not None and len(ids) == 4: # check marker are detcted or not, if yes:
        
            src_points = np.zeros((4, 2), dtype=np.float32) # map IDs to the corresponding center points
            scale_factor = 1  # Adjust this scale if you want to change the output resolution
            width = int(maze_width * scale_factor)
            height = int(maze_height * scale_factor)
            destionation_points = np.array([[0, 0], [width, 0], [width, height], [0, height]], dtype=np.float32)
            
            
            
            for i, marker_id in enumerate(ids.flatten()): # calculate the center of each marker for each marker
                c0, c1, c2, c3 = corners[i][0]  # Get the four corner points of the marker
                center_x = (c0[0] + c1[0] + c2[0] + c3[0]) / 4
                center_y = (c0[1] + c1[1] + c2[1] + c3[1]) / 4
                
                # assign to src_points based on the marker ID
                # marker sequation is bottom left , top left, top right, bottom right (out put of detected marker started form top left)
                if marker_id == 0:  # Bottom Left
                    src_points[3] = [center_x, center_y]
                elif marker_id == 1:  # Top Left
                    src_points[0] = [center_x, center_y]
                elif marker_id == 2:  # Top Right
                    src_points[1] = [center_x, center_y]
                elif marker_id == 3:  # Bottom Right
                    src_points[2] = [center_x, center_y]
    
            perspective_matrix = cv.getPerspectiveTransform(src_points, destionation_points) # compute perspective transformation matrix
            transformed_image = cv.warpPerspective(image, perspective_matrix, (width, height))  # apply it
            cv.imwrite(transformed_image_filename, transformed_image) #save_transformed image
        
        else:
            print("No marker detected")
    else:
        transformed_image = image
    
    
    grayscale_image = cv.cvtColor(transformed_image, cv.COLOR_BGR2GRAY) # Convert image to grayscale
    blurred_image = cv.GaussianBlur(grayscale_image,(3,3),0)  # Apply Gaussian blur to reduce noise
    resized_image = cv.resize(blurred_image,(maze_width, maze_height))  # Resize image with our desigre height and with
    edges_detected = cv.Canny(image=resized_image, threshold1=45, threshold2=200, apertureSize=3)  # Edge detection by Canny
    
    blurred_image= cv.GaussianBlur(edges_detected, (5,5),0)
    blurred_image[blurred_image > 50] = 255
    edged_detected = cv.Canny(image=blurred_image, threshold1=45, threshold2=200, apertureSize=3)
    
    edges_detected[edges_detected > 0] = 1  # Convert edge pixels  morethen 0 to 1
    
    return np.array(np.flip(edges_detected, axis=0))  # Return the matrix ( make it based on cartesian)


######      function to show matrix (invert in y axis , because we want to see everything based on cartesion)
def imshow_maze_matrix(maze):
    plt.imshow(maze)
    plt.gca().invert_yaxis()



######      solve the maze using the a-star algorithm and returns the path and directions.
def maze_astar_solver(maze, start, goal, buffer_size=1):

    #
    # maze: binary matrix of maze (0 = open path, 1 = wall).
    # buffer_size: integer value to give how far away path from wall
    #
    
    def manhattan_distance(a, b): #calculate Manhatan distance
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    # vcheck current position is valide or not
    def is_valid(maze, pos, buffer_size):

        rows, cols = len(maze), len(maze[0])

        #--------------------------------------------------------------------------------#
        #-----------------------------IMPORTANT------------------------------------------#
        #--------------------------------------------------------------------------------#
        col, row = pos # pos is cartesian: always keep this one 
        #--------------------------------------------------------------------------------#
        #--------------------------------------------------------------------------------#
        #--------------------------------------------------------------------------------#

        # checkk if the currecnt position is in the matrix
        if not (0 <= row < rows and 0 <= col < cols) or maze[row][col] != 0:
            return False
        
        # Check scurround  position is out of buffer size from wall
        for r_offset in range(-buffer_size, buffer_size + 1):
            for c_offset in range(-buffer_size, buffer_size + 1):
                check_row, check_col = row + r_offset, col + c_offset
                if 0 <= check_row < rows and 0 <= check_col < cols:
                    if maze[check_row][check_col] == 1:
                        return False
        return True

    # reversing path (from goal to star with information of directions form came_from)
    def reconstruct_path(came_from, current):
        
        path, directions = [], []
        while current in came_from:
            previous, direction = came_from[current]
            path.append(current)
            directions.append(direction)
            current = previous
        path.reverse()
        directions.reverse()
        return path, directions

    # check validity of start and end (mostly used for making random star and goal point)
    if not is_valid(maze, start, buffer_size):
        return "Start point is invalid ."
    if not is_valid(maze, goal, buffer_size):
        return "Goal point is invalid ."

    # direction map ( our point of view, not exactly x and y change from matrix)
    directions_map = {"right": (1, 0), "down": (0, -1), "left": (-1, 0), "up": (0, 1)}
    all_directions = ["right", "down", "left", "up"]

    open_list = [(0, start)] #open list of a-star which stores all possible nodes for exploration with their f score value
    g_score = {start: 0} #dictionary fo g-scores, it also can be used for finding neigbours node's g score
    came_from = {} # dictionary to now wich node came from and firection to it

    # a-star main search loop
    while open_list:

        _, current = heapq.heappop(open_list) # Pop node with the lowest f_score from the open_list
        
        if current == goal: # check the current node is goal or not , revere our list to with reconstruct_path
            return reconstruct_path(came_from, current)
        
        for direction in all_directions: #check all possible direction

            move = directions_map[direction]
            neighbor = (current[0] + move[0], current[1] + move[1]) # find the neighbor position
            
            # If the neighbor is valid (within bounds, not a wall, and with a buffer around walls)
            if is_valid(maze, neighbor, buffer_size): # If the neighbor is valid calculate g-score and h-score and finde f-score
                
                new_g_score = g_score[current] + 1 #each g-score of neighbors of one node is 
                # If this path to the neighbor is better, or it's the first time we visit it
                if neighbor not in g_score or new_g_score < g_score[neighbor]: #if g-score is less than g-score of neighbor , updtae g-score, calculate h-score and find new f-score

                    g_score[neighbor] = new_g_score
                    f_score = new_g_score + manhattan_distance(neighbor, goal)
                    heapq.heappush(open_list, (f_score, neighbor)) #push node with new f-score to the open_list
                    came_from[neighbor] = (current, direction) #sotre direction in came_from

    return None



######      summarizes consecutive movements in the same direction and counts the number of consecutive moves
def summarize_path(directions):

    if not directions:
        return []

    summarized_movements = [] #list to store summerized path
    current_index = 0

    #initialize first movement
    current_movement = {
        "direction": directions[0],
        "cm": 1 
    }
    summarized_movements.append(current_movement)

    # loop through list and find consecutive movements and list it
    for i in range(1, len(directions)):
        
        if directions[i] == current_movement["direction"]:
            summarized_movements[current_index]["cm"] += 1

        else:
            current_movement = { # if direction changes, create a new movement summary and restart count
                "direction": directions[i],
                "cm": 1
            }
            
            # move to the next entry in the summarized list and append new movement to the list
            current_index += 1 
            summarized_movements.append(current_movement)

    return summarized_movements


######      clean up summerized path by merging small moves into prev or next large move
def clean_up_path(moves, start,  move_size_limit=15):

    grouped_chunks = {}    # dictionary to store grouped movement chunks
    current_chunk_key = 0  # key to identify each movement chunk
    
    # loop through each move in the list
    for move in moves:

        if move["cm"] > move_size_limit: #if movement is big, creat new chunk and increase by one
            current_chunk_key += 1 
            grouped_chunks[current_chunk_key] = {move["direction"]: move["cm"]}
        else:
            # For smaller moves, add itto the current chunk
            if move["direction"] in grouped_chunks.get(current_chunk_key, {}):
                grouped_chunks[current_chunk_key][move["direction"]] += move["cm"]
            else:
                grouped_chunks.setdefault(current_chunk_key, {})[move["direction"]] = move["cm"]

    for chunk_key, current_chunk in grouped_chunks.items(): # merge small moves from current chunks into the next large chunk with same direction

        next_chunk = {} if chunk_key >= len(grouped_chunks) else grouped_chunks.get(chunk_key + 1, {})
        
        for direction in ["left", "right", "down", "up"]:
            if (direction in current_chunk) and (current_chunk[direction] < move_size_limit) and (direction in next_chunk) and (next_chunk[direction] >= move_size_limit):
                next_chunk[direction] += current_chunk[direction]
                current_chunk[direction] = 0  # set the current chunk's move to 0 after merging ro delete it later

    # pop empty move directions 
    for chunk_key, current_chunk in grouped_chunks.items():
        for direction in ["left", "right", "down", "up"]:
            if current_chunk.get(direction) == 0:
                current_chunk.pop(direction)
    
    # another merge small moves into the next chunk if a direction exists in both the current and next chunk
    for chunk_key, current_chunk in grouped_chunks.items():
        next_chunk = {} if chunk_key >= len(grouped_chunks) else grouped_chunks.get(chunk_key + 1, {})
        directions = list(current_chunk.keys()) 
        

        for direction in directions:
            if direction in next_chunk:
                next_chunk[direction] += current_chunk[direction]
                current_chunk.pop(direction)  # pop the direction from the current chunk after merging
    
    # remove empty chunks
    grouped_chunks = {key: value for key, value in grouped_chunks.items() if value}
    
    clean_full_paths = [start]
    for k,move in grouped_chunks.items() :
        if "left" in move:
            cm = move["left"]
            last_x, last_y = clean_full_paths[-1]
            new_xs = list(np.arange(last_x -1, last_x - cm -1, -1))
            clean_full_paths += [(x, last_y) for x in new_xs]
    
        if "right" in move:
            cm = move["right"]
            last_x, last_y = clean_full_paths[-1]
            new_xs = list(np.arange(last_x +1, last_x + cm +1, +1))
            clean_full_paths += [(x, last_y) for x in new_xs]
    
        if "down" in move:
            cm = move["down"]
            last_x, last_y = clean_full_paths[-1]
            new_ys = list(np.arange(last_y -1, last_y - cm -1, -1))
            clean_full_paths += [(last_x, y) for y in new_ys]
    
        if "up" in move:
            cm = move["up"]
            last_x, last_y = clean_full_paths[-1]
            new_ys = list(np.arange(last_y +1, last_y + cm +1, +1))
            clean_full_paths += [(last_x, y) for y in new_ys]

    clean_directions = []
    for k,move in grouped_chunks.items():
        dir = list(move.keys())[0]
        clean_directions += [dir] * move[dir]
    clean_directions = [clean_directions[0]] + clean_directions

    return grouped_chunks, clean_full_paths, clean_directions


######      visualizes path and direction output from-star on the maze, summarized path and cleaned path.
def visualize_paths(path, directions, 
                    start_point, goal_point, maze_grid, output_image_filename, fig_size=10, show_maze=True,
                    print_path=True, print_directions=True, 
                    show_summarized_path=True, show_cleaned_path=True, 
                    print_summarized_path_info=True, print_cleaned_path_info=True):
    #
    # show_maze: show the original maze.
    # print_path: print the path.
    # print_directions: print the movement directions.
    # show_summarized_path: plot the summarized path.
    # show_cleaned_path:plot the cleaned path.
    # ptint_summarized_path_info: print the summarized path information.
    # print_cleaned_path_info: print the cleaned path information.
    #

    output_image_filename = f"maze-map-path-{output_image_filename}.png"
    
    # if path found
    if path:

        path_coordinates= path
        move_directions = directions

        # get X and Y coordinates from path
        path_x_coords = [coord[0] for coord in path_coordinates]
        path_y_coords = [coord[1] for coord in path_coordinates]

        # call summerize_path funtion to get summerized path
        # summarized_moves = summarize_path(move_directions, path_coordinates)


        summarized_moves = summarize_path(move_directions)

        initial_x, initial_y = start_point[0], start_point[1]
        x_movement, y_movement = [initial_x], [initial_y]

        # adjust X and Y coordinates based on the movement direction and distance ('cm') from summerized list
        for movement in summarized_moves:
            
            if movement["direction"] == "left":
                x_movement.append(x_movement[-1] - movement["cm"])  # Move left
                y_movement.append(y_movement[-1])
            if movement["direction"] == "right":
                x_movement.append(x_movement[-1] + movement["cm"])  # Move right
                y_movement.append(y_movement[-1])
            if movement["direction"] == "down":
                x_movement.append(x_movement[-1])
                y_movement.append(y_movement[-1] - movement["cm"])  # Move down
            if movement["direction"] == "up":
                x_movement.append(x_movement[-1])
                y_movement.append(y_movement[-1] + movement["cm"])  # Move up

        # clean up the summerized path (remove samll movement and add them to big moves)
        cleaned_path_result = clean_up_path(summarized_moves, start_point)
        cleaned_path,_,_ = cleaned_path_result
        cleaned_x_movement, cleaned_y_movement = [initial_x], [initial_y]

        # adjust X and Y coordinates based on the movement direction for cleaned up list
        for key in sorted(list(cleaned_path.keys())):
            movement = cleaned_path[key]

            if "left" in movement:
                cleaned_x_movement.append(cleaned_x_movement[-1] - movement["left"])  # Move left
                cleaned_y_movement.append(cleaned_y_movement[-1])
            if "right" in movement:
                cleaned_x_movement.append(cleaned_x_movement[-1] + movement["right"])  # Move right
                cleaned_y_movement.append(cleaned_y_movement[-1])
            if "down" in movement:
                cleaned_x_movement.append(cleaned_x_movement[-1])
                cleaned_y_movement.append(cleaned_y_movement[-1] - movement["down"])  # Move down
            if "up" in movement:
                cleaned_x_movement.append(cleaned_x_movement[-1])
                cleaned_y_movement.append(cleaned_y_movement[-1] + movement["up"])  # Move up

        # Showing and Ploting:
        plt.figure(figsize=(fig_size,fig_size)) 
        if show_maze:
            imshow_maze_matrix(maze_grid)  # Display the maze
            plt.scatter([start_point[0]], [start_point[1]], color="yellow", s=100, marker='x')
            plt.scatter([goal_point[0]], [goal_point[1]], color="blue", s=100, marker='*')
            
        if print_directions:
            print("\nPath Directions:", *move_directions, sep='\n')

        if print_summarized_path_info:
            print("Summarized Path Movements:", *summarized_moves, sep='\n')

        if print_cleaned_path_info:
            print("\nNumber of movements in cleaned path: ", len(cleaned_path))
            print("\nCleaned Path Movements:", *cleaned_path.values(), sep='\n')

        if print_path:
            plt.scatter(path_y_coords, path_x_coords, s=10, color="r")  # Plot the path

        if show_summarized_path:
            plt.plot(x_movement, y_movement, color='white', linestyle='dashed')

        if show_cleaned_path:
            plt.plot(cleaned_x_movement, cleaned_y_movement, color='yellow', linewidth=7, alpha=0.5)
            
           
        plt.savefig(output_image_filename)
        plt.show()  # Display the image

    else:
        print("There is no path from the start to the goal.")

    
#######      use polynomial curve to the sensor values and actual distances and returns calibration coefficients.
def fit_calibration_curve(sensor_values, actual_distances, sensor_id, calibration_coefficients=None, polynomial_degree=1):

    # check if calibration data is not exist, make new one
    if calibration_coefficients is None:
        calibration_coefficients = {}
    
    # use polynomial curve and get the coefficients
    coefficients = np.polyfit(sensor_values, actual_distances, polynomial_degree)
    
    # store the coefficients for each sensors
    calibration_coefficients[sensor_id] = coefficients

    return calibration_coefficients

#calculate calibrated data
def calculate_calibrated_distance(sensor_reading_tuple, calibration_coefficients):

    # seperate sensor value and id
    sensor_reading, sensor_id = sensor_reading_tuple
    
    # check if the sensor has been calibrated then get coefficients and calculate distance by ploycal to get distance
    if sensor_id in calibration_coefficients:
        coefficients = calibration_coefficients[sensor_id] 
        return np.polyval(coefficients, sensor_reading)
    else:
        raise ValueError(f"No calibration data found for sensor {sensor_id}.")


######      visulizing sensor data for spesific sensors    
def visualize_sensor_data(sensor_list, actual_distances, selected_sensors):
    
    # loop through the selected sensors and plot each one
    for sensor_id in selected_sensors:
        sensor_values = sensor_list[sensor_id]
        plt.plot(actual_distances, sensor_values, 'o-', label=f"Sensor {sensor_id}")
    
    plt.title(f"Sensor Calibration Curves for Sensors {', '.join(map(str, selected_sensors))}")
    plt.xlabel("Actual Distance (cm)")
    plt.ylabel("Sensor Value")
    plt.grid(True)
    plt.legend()
    plt.show()

######      wall detection filtering by convolv for each position in a-star path
def wall_detection_filtering(maze, path, filter_size, directions):
    # create vertical and horizontal filters
    template_size = 80
    mid_point_idx = int(template_size/2)
    filter_template = np.zeros((template_size,template_size))
    filter_width = 10
    filter_length = filter_size
    
    filter_up = filter_template.copy()
    filter_up[mid_point_idx - filter_length: mid_point_idx,
              mid_point_idx - int(filter_width/2): mid_point_idx - int(filter_width/2) + filter_width] = 1
    
    filter_down = filter_template.copy()
    filter_down[mid_point_idx: mid_point_idx + filter_length,
                mid_point_idx - int(filter_width/2): mid_point_idx - int(filter_width/2) + filter_width] = 1
    
    filter_left = filter_template.copy()
    filter_left[mid_point_idx - int(filter_width/2): mid_point_idx - int(filter_width/2) + filter_width,
                 mid_point_idx - filter_length: mid_point_idx] = 1
    
    
    filter_right = filter_template.copy()
    filter_right[mid_point_idx - int(filter_width/2): mid_point_idx - int(filter_width/2) + filter_width,
                 mid_point_idx: mid_point_idx + filter_length] = 1
    # apply 2D convolution for vertical and horizontal directions
    up_walls = sp.signal.convolve2d(maze, filter_up, "same")
    down_walls = sp.signal.convolve2d(maze, filter_down, "same")
    right_walls = sp.signal.convolve2d(maze, np.flip(filter_right, axis=1), "same")
    left_walls = sp.signal.convolve2d(maze, np.flip(filter_left, axis=1), "same")

    # count walls on the path for each direction and if it is more than 0, make it 1
    # REMEMBER (see above): indexing in maze is: idx_1 = pos_y, idx_2 = pos_x 
    up_wall_count = np.array([up_walls[y, x] for x, y in path])
    up_wall_count[up_wall_count > 0] = 1

    down_wall_count = np.array([down_walls[y, x] for x, y in path])
    down_wall_count[down_wall_count > 0] = 1

    print("before", down_wall_count[129])

    right_wall_count = np.array([right_walls[y, x] for x, y in path])
    right_wall_count[right_wall_count > 0] = 1

    left_wall_count = np.array([left_walls[y, x] for x, y in path])
    left_wall_count[left_wall_count > 0] = 1

    # list to store the direction, position, and wall statuses
    wall_statuses = []


    # define the mapping for each direction
    direction_mapping = {
        'up':    {'front': up_wall_count, 'left': left_wall_count, 'right': right_wall_count},
        'down':  {'front': down_wall_count, 'left': right_wall_count, 'right': left_wall_count},
        'left':  {'front': left_wall_count, 'left': down_wall_count, 'right': up_wall_count}, 
        'right': {'front': right_wall_count, 'left': up_wall_count, 'right': down_wall_count},
    }

    # loop through directions, positions, and wall statuses to store the results
    for idx, (direction, position) in enumerate(zip(directions, path)):
        mapping = direction_mapping[direction]
        
        wall_statuses.append({
            "direction": direction,
            "position": position,
            "front_wall": int(mapping['front'][idx]),
            # "back_wall": None,  # Set all back walls to None
            "left_wall": int(mapping['left'][idx]),
            "right_wall": int(mapping['right'][idx]) # Can be None based on the direction
        })
        if idx==129:
            print("fucking here", int(mapping['front'][idx]), down_wall_count[idx])
        
    return wall_statuses    


######      plot path status based on wall condistion
def plot_wall_conditions(maze, wall_statuses):
    
    color_map = {
        (0, 0, 0): 'white',     # No walls
        (1, 0, 0): 'red',       # Wall only in front
        (0, 1, 0): 'green',     # Wall only to the right
        (0, 0, 1): 'blue',      # Wall only to the left
        (1, 1, 0): 'yellow',    # Walls in front and right
        (1, 0, 1): 'orange',    # Walls in front and left
        (0, 1, 1): 'purple',    # Walls to the left and right
        (1, 1, 1): 'black'      # Walls on all sides
    }
    
    fig = plt.figure(figsize=(15,15))
    imshow_maze_matrix(maze)
    # Loop over the wall statuses and plot the corresponding color for each position
    for status in wall_statuses:
        position = status['position']
        # REMEMBER again: indexing in maze is with maze(pos[1], pos[0])
        x, y = position
        
        # Get the wall conditions (1 for wall, 0 for no wall)
        front_wall = status['front_wall']
        right_wall = status['right_wall']
        left_wall = status['left_wall']
        
        # Determine the color based on the walls
        wall_condition = (front_wall, right_wall, left_wall)
        color = color_map.get(wall_condition, 'gray')  # Default to gray if the condition isn't found
        plt.scatter(x,y, color=color,s=10, axes=plt.gca())


    
    ######      virtual robot class
class VirtualRobot:
    def __init__(self, maze, start_x, start_y, end_x, end_y, 
                astar_path, astar_path_bandwidth, 
                sensor_range=1):
        # initializeation
        self.start_x_actual = start_x  
        self.start_y_actual = start_y
        self.end_x_actual = end_x      
        self.end_y_actual = end_y
        self.astar_path = astar_path
        self.astar_path_bandwidth = astar_path_bandwidth
        self.maze = np.array(maze)
        self.sensor_range = sensor_range
        
        # start conditons
        self.x = start_x  
        self.y = start_y
        self.heading = 180  # facing right (0 degrees)
        self.history = [(self.x, self.y, self.heading)]  # Log initial position and heading

    # simulate reading sensors for front, left, and right positions
    def read_sensors(self):
        front = self._check_sensor(self.heading)
        left = self._check_sensor((self.heading + 65) % 360)
        right = self._check_sensor((self.heading - 65) % 360)
        return {'front': front, 'left': left, 'right': right}
        
    # calculate and return the distance to the nearest obstacle in the given angle
    def _check_sensor(self, angle):
        
        rad_angle = np.radians(angle)
        for distance in np.arange(0.5, self.sensor_range + 0.5, 0.5):
            check_x = int(round(self.x + distance * np.cos(rad_angle)))
            check_y = int(round(self.y + distance * np.sin(rad_angle)))
            if not self._within_bounds(int(check_x), int(check_y)) or self.maze[check_y][check_x] == 1:
                return distance
        return 1000  # large value indicating no obstacle
        
    # Ensure the given position is within the maze's boundaries
    def _within_bounds(self, x, y):
        return 0 <= x < self.maze.shape[1] and 0 <= y < self.maze.shape[0]

    # move robot forward if there is no obstacle within the specified distance
    def move(self, distance=1):
        
        if self._check_sensor(self.heading) >= distance:
            rad_heading = np.radians(self.heading)
            self.x = self.x + distance * np.cos(rad_heading)
            self.y = self.y + distance * np.sin(rad_heading)
            self.history.append((self.x, self.y, self.heading))  # log the movement in robot history

    # rotate the robot by a specified angle
    def rotate(self, angle):
        # Rotate the robot by a specified angle
        self.heading = (self.heading + angle) % 360

    def get_history(self):
        return self.history
    
######      check if the robot is within the boundaries of path
def is_within_astar_boundaries(robot, path, distance, angle): 
    
    ideal_path = path
    idea_path_bandwidth = robot.astar_path_bandwidth
    check_x = int(round(robot.x + distance * np.cos(np.radians(angle))))
    check_y = int(round(robot.y + distance * np.sin(np.radians(angle))))
    
    # check distance by eculidian
    for i in range(len(ideal_path) - 1):
        
        x1, y1 = ideal_path[i]
        distance = math.sqrt((check_x - x1) ** 2 + (check_y - y1) ** 2)

        if distance <= idea_path_bandwidth:
            return True  
    
    return False  # move is outside the path boundary  

######      robot follow left-wall-follow algorithm
def follow_left_wall(robot, path, desired_left_distance=12, rotation_angle=15,
                    move_distance=1, robot_sensor_angle=65):
    
    sensors = robot.read_sensors() # read sensor data from the robot

    # calculate new headings for checking left, right, and front positions
    # with respect to the current heading, to see if those are within path boundaries
    left_check_heading = robot.heading + robot_sensor_angle
    within_astar_left = is_within_astar_boundaries(robot, path, distance=10, angle=left_check_heading)

    right_check_heading = robot.heading - robot_sensor_angle
    within_astar_right = is_within_astar_boundaries(robot, path, distance=10, angle=right_check_heading)

    front_check_heading = robot.heading
    within_astar_front = is_within_astar_boundaries(robot, path, distance=10, angle=front_check_heading)

    # update sensor readings based on astar boundary checks.
    # if a direction is out of bounds, set that sensor to 0.
    sensors['left'] = sensors['left'] if within_astar_left else 0
    sensors['right'] = sensors['right'] if within_astar_right else 0
    sensors['front'] = sensors['front'] if within_astar_front else 0

    # Decision making based on sensor values
    if sensors['left'] > robot.sensor_range and sensors['front'] > robot.sensor_range: 
        # If the left and front are clear, turn left and move forward
        print('Left is open, turning left')
        robot.rotate(rotation_angle)
        robot.move(move_distance)
            
    elif sensors['front'] > robot.sensor_range:
        # If only the front is clear, move forward
        print('Left is closed, front is open, moving forward')
        robot.move(move_distance)
            
    elif sensors['right'] > robot.sensor_range:
        # If left and front are blocked but right is clear, turn right and move
        print('Left and front are closed, turning right')
        robot.rotate(-rotation_angle)
        robot.move(move_distance)
    else:
        # If all directions are blocked, perform a U-turn
        print('All directions are blocked, performing U-turn')
        robot.rotate(180) 

######      calculate angle between to vector
def angle_between_vectors(A, B):
    # Convert inputs to numpy arrays
    A = np.array(A)
    B = np.array(B)
    
    dot_product = np.dot(A, B)
    mag_A = np.linalg.norm(A)
    mag_B = np.linalg.norm(B)
    
    angle_rad = np.arccos(dot_product / (mag_A * mag_B))
    angle_deg = np.degrees(angle_rad)
    
    # calculate the cross product (in 2D, this is a scalar, not a vector)
    cross_product = A[0] * B[1] - A[1] * B[0]
    # determine the sign based on the cross product
    if cross_product < 0:
        # if cross product is negative, B is to the right of A, so we rotate clockwise
        angle_deg = -angle_deg
    
    return angle_deg

######      Checks if a robot position(x1, y1) is within a certain distance (threshold) to a goal (x2, y2)
def is_within_distance_to_goal(x1, y1, x2, y2, threshold):
    return True if np.abs((x1 - x2) ** 2 + (y1 - y2) ** 2) < threshold else False

######      move to goal point
def move_robot_to_goal(my_robot, goal):
    # calculate the direction vectors
    x_new = my_robot.x + np.cos(np.radians(my_robot.heading))
    y_new = my_robot.y + np.sin(np.radians(my_robot.heading))
    vector_heading_dirc = [x_new - my_robot.x, y_new - my_robot.y]
    vector_end_direction = [goal[0] - my_robot.x, goal[1] - my_robot.y]
    
    # determine distance and angle to the goal
    size_vector_end_direction = np.sqrt(vector_end_direction[0]**2 + vector_end_direction[1]**2)
    turn_degree = angle_between_vectors(vector_heading_dirc, vector_end_direction)
    
    # rotate and move
    if turn_degree != 0:
        my_robot.rotate(turn_degree)
    my_robot.move(size_vector_end_direction)
    
    # Print the final position
    print(f"Reached end point: Position ({round(my_robot.x)}, {round(my_robot.y)})")

    # run robot
def robot_run(robot, path, goal, move_count=2000):
    for i in range(move_count):
        follow_left_wall(my_robot,clean_full_path,8)
        if is_within_distance_to_goal(my_robot.x, my_robot.y, my_robot.end_x_actual, my_robot.end_y_actual, threshold=200):
            move_robot_to_goal(my_robot, goal)
            break

######      visualizing robot path
def visualize_robot_path(maze,robot,fig_size):
    x = [h[0] for h in robot.history]
    y = [h[1] for h in robot.history]
    plt.figure(figsize=(fig_size,fig_size))
    plt.imshow(maze)
    plt.imshow(result, axes=plt.gca(), alpha=0.5)
    plt.gca().invert_yaxis()
    plt.scatter(x,y, axes=plt.gca(), s=10, color="r")    
