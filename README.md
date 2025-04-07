
# Integrating A* Pathfinding and Virtual Wall Guidance for Left Wall-Following on iRobot Create 3

This project involves developing a robot that solves mazes using a combination of image processing, the A* algorithm for pathfinding, and a left-wall-following navigation strategy. The robot adjusts its movements based on sensor inputs to ensure it stays on the optimal path.

## Methodology


In our maze-solving approach we start by capturing an image of the maze and converting it into a matrix. We then use the A* algorithm to find the best path and set a boundary around it. The robot uses a left wall-following strategy to solve the maze but whenever this approach is approaching the boundary its sensors are updated to create a virtual wall guiding it back within the path’s limits. The following sections will explain each step of this methodology in detail.

1. Digital Mapping of the Maze and Start/End Point Detection

To prepare the maze image for the robot we first capture the image and transform it to correct for any perspective issues. Using four ArUco markers placed at each corner we apply OpenCV’s warpPerspective function ensuring the image is flat and aligned. After transforming the image we locate the start and end points using OpenCV’s matchTemplate function which compares small images of the start and end symbols with the main maze image to identify their positions.

Once the start and end points are located we proceed with edge detection. First we apply OpenCV’s GaussianBlur to the maze image to reduce noise then use OpenCV’s Canny function to detect the edges that outline the maze walls. After this initial edge detection we mask the start and end points to avoid any false edge detection around them applying OpenCV’s bitwise_and to combine the mask with the edge-detected image. Following this masking we reapply GaussianBlur and Canny to refine the maze’s edge map.

To align with Cartesian coordinates we flip the resulting edge map vertically. This edge map is then converted into a binary matrix where walls are marked as 1 and open paths as 0. The start and end points are also adjusted to Cartesian coordinates for easier path planning. This final matrix along with the start and end positions in Cartesian format is then used as input for the next steps.

2. Finding the Optimal Path

To find a path from the start to the goal we use the maze_astar_solver function which implements the A* algorithm.

The A* algorithm calculates a total cost called f_score for each position. This f_score is made up of two parts: g_score which is the distance traveled from the start and increases by 1 with each step and h_score which estimates the distance to the goal using manhattan_distance. This distance function helps the robot estimate how far it is from the goal.

We use a single list called the open list to track positions that need to be evaluated. Each position in this list has an f_score which lets us focus on the most promising paths first. At each step the position with the lowest f_score is removed from the open list and added to a dictionary called came_from which records both the previous position and the direction taken to reach it. This approach is similar to a closed list by preventing revisits and it also lets us reconstruct the path later.

The function starts by checking if the start and goal positions are valid and safely away from walls using the is_valid function. Inside is_valid the row and column values are swapped to match the Cartesian orientation as our matrix has already been flipped. The function then checks if a position is within maze boundaries, is not a wall, and is surrounded by a buffer zone that avoids walls. This buffer prevents the robot from entering small gaps and ensures enough space for it to move (about 60 cm wide).

During the search, the function checks moves in four directions (right, down, left, up). If a move is valid and improves the path, it updates that position and adds it to the open list. Once the goal is reached, reconstruct_path is used to trace the path from the goal back to the start, recording the directions taken. The final output is a list of coordinates for the path and the corresponding movement directions. If no path is found, the function returns None.

3. Navigation Through Physical and Virtual Walls

After finding the path from start to goal, the robot navigates the maze using a left wall-following approach. Before each move, the robot checks its position to make sure it stays within the path boundaries. If the robot is about to move outside the boundary, the sensor values are overwritten with a high value simulating a wall in that direction. This virtual wall effect keeps the robot on track within the path.

### Path Boundary Check

The is_within_path_boundary function checks if the robot is within a safe distance from the path. It calculates a new position based on the robot’s current location, distance, and angle, then compares this position with each point in the path. If this position is within a set distance (the “path bandwidth”), the function returns True, meaning the robot is still on the path.

### Main Navigation Loop

The navigation loop guides the robot based on sensor data and path checks:

1. **Getting Position and Sensors**: The robot reads its current position and IR sensor data from the left, front, and right sensors. These sensors help it detect nearby walls.
2. **Boundary Checks and Sensor Overwriting**: The robot checks if it is within the path boundary to the left, right, and front directions by using is_within_path_boundary. If the robot is moving close to the boundary, the function overwrites the IR sensor value with a high number simulating a wall. This tells the robot to stay within the path limits.
3. **Left Wall-Following Strategy**: The robot follows the wall on its left side:
   - **Left Sensor**: If there is space on the left, the robot turns left. If the left side is too close to a wall, it turns right slowly.
   - **Right Sensor**: If the right side is too close, the robot turns slightly left to avoid the wall.
   - **Front Sensor**: If there is an obstacle in front, the robot turns right. For very close obstacles, it turns sharply right.
4. **Goal Check**: The robot checks if it is near the goal using is_within_distance_to_goal, which measures the distance to the goal. If the robot is close enough, it stops, lights up blue, and plays a sound by calling the sing function.

This loop allows the robot to follow the path while keeping to the left wall. If it nears the boundary, the high sensor values make it think there is a wall guiding it back onto the path. The robot continues until it reaches the goal.

### Generalized Pseudocode for the Maze-Solving Process

```python
# Step 1: Process the maze image and extract necessary information
function solve_maze(maze_image):
    # Take the maze image and transform it into a matrix with walls and paths
    # Identify the start and end positions in the maze
    maze_matrix, start, end = process_image_to_matrix(maze_image)
    return maze_matrix, start, end

# Step 2: Use pathfinding algorithm to determine the optimal route
function find_optimal_path(maze_matrix, start, end):
    # Apply a pathfinding algorithm (e.g., A*) to find the shortest path from start to end
    path = apply_pathfinding_algorithm(maze_matrix, start, end)
    return path

# Step 3: Guide the robot through the maze using the path
function navigate_through_maze(path):
    while not at_goal:
        # Check if the robot is approaching the boundaries of the path
        if out_of_left_boundary: set left sensor high (simulate wall)
        if out_of_right_boundary: set right sensor high (simulate wall)
        if out_of_front_boundary: set front sensor high (simulate wall)

        # Adjust the robot's movement based on sensor data
        if left sensor detects space: turn left
        elif left sensor too close: turn slightly right
        if right sensor too close: turn slightly left
        if front sensor detects obstacle: turn right
        if front sensor very close: turn sharply right

        if robot has reached the goal: stop and celebrate

# Full Maze-Solving Process
function main(maze_image):
    # 1. Convert maze image to matrix and extract start/end points
    maze_matrix, start, end = solve_maze(maze_image)
    
    # 2. Find the optimal path using a pathfinding algorithm
    path = find_optimal_path(maze_matrix, start, end)
    
    # 3. If a valid path is found, navigate through the maze
    if path:
        navigate_through_maze(path)
    else:
        print("No valid path found.")
```

## Utilities

# Project Utilities Overview

This project contains several utility functions to handle various tasks such as image processing, pathfinding, and more. Below is a list of the main functions and their purposes:

### `transform_and_convert_image_to_matrix`
- **Parameters**: image_path (str), start_image_path (str), goal_image_path (str), transforming (bool), transformed_file_name (str), maze_height (int), maze_width (int)
- **Output**: Returns a binary matrix representing the maze, with walls as 1 and paths as 0.

Description: Handles transform and convert image to matrix functionality in the project.

### `imshow_maze_matrix`
- **Parameters**: maze_matrix (2D array), title (str)
- **Output**: Displays the matrix as an image with walls and open paths.

Description: Handles imshow maze matrix functionality in the project.

### `robot_movement_on_maze`
- **Parameters**: maze_matrix (2D array), path (list of tuples)
- **Output**: Displays the robot's movement through the maze using the given path.

Description: Handles robot movement on maze functionality in the project.

### `maze_astar_solver`
- **Parameters**: maze (2D array), start (tuple), goal (tuple), buffer_size (int)
- **Output**: Returns the optimal path as a list of coordinates from start to goal using the A* algorithm.

Description: Handles maze astar solver functionality in the project.

### `transform_path_to_origin`
- **Parameters**: path (list of tuples)
- **Output**: Transforms the path to align with the origin (0,0).

Description: Handles transform path to origin functionality in the project.

### `summarize_path`
- **Parameters**: path (list of tuples)
- **Output**: Returns a simplified version of the path by merging consecutive moves in the same direction.

Description: Summarizes consecutive movements in the same direction and counts the number of consecutive moves.
summarize path functionality in the project.

### `clean_up_path`
- **Parameters**: summarized_path (list of tuples)
- **Output**: Cleans up the summarized path by merging smaller movements into larger ones for better efficiency.

Description: clean up summerized path by merging small moves into prev or next large move
clean up path functionality in the project.

### `visualize_paths`
- **Parameters**: maze_matrix (2D array), astar_path (list of tuples), summarized_path (list of tuples), cleaned_path (list of tuples)
- **Output**: Displays the maze with the visualized A* path, summarized path, and cleaned path.

Description: Handles Visualize A* Path, Summarized Path, and Cleaned Path on Maze
visualize paths functionality in the project.

### `fit_calibration_curve`
- **Parameters**: sensor_data (list of floats), actual_distances (list of floats)
- **Output**: Returns polynomial coefficients for calibrating sensor data to actual distances.

Description: use polynomial curve to the sensor values and actual distances and returns calibration coefficients.
fit calibration curve functionality in the project.

### `calculate_calibrated_distance`
- **Parameters**: sensor_value (float), calibration_coefficients (list of floats)
- **Output**: Returns the calibrated distance based on the sensor value.

Description: Handles calculate calibrated data
calculate calibrated distance functionality in the project.

### `visualize_sensor_data`
Description: Handles visulizing sensor data for spesific sensors    
visualize sensor data functionality in the project.

### `generate_and_visualize_filters`
- **Parameters**: None
- **Output**: Generates and visualizes different filters used in the process.

Description: Handles generate and visualize filters functionality in the project.

### `wall_detection_filtering`
- **Parameters**: astar_path (list of tuples), maze_matrix (2D array)
- **Output**: Applies filtering to detect walls along the A* path.

Description: Handles wall detection filtering by convolv for each position in a-star path
wall detection filtering functionality in the project.

### `plot_wall_conditions`
- **Parameters**: path (list of tuples), wall_conditions (list of bools)
- **Output**: Plots the wall conditions along the path to visualize potential obstacles.

Description: Handles plot path status based on wall condistion
plot wall conditions functionality in the project.

### `VirtualRobot`
Description: virtual robot 

### `is_within_astar_boundaries`
- **Parameters**: current_position (tuple), path (list of tuples), bandwidth (int)
- **Output**: Returns True if the current position is within the boundary limits of the path.

Description: Handles check if the robot is within the boundaries of path
is within astar boundaries functionality in the project.


### `move_robot_to_goal`
- **Parameters**: robot (VirtualRobot), path (list of tuples)
- **Output**: Moves the virtual robot along the given path to the goal.

Description: Handles move to goal 
move robot to goal functionality in the project.

### `follow_left_wall`
- **Parameters**: robot (VirtualRobot), sensors (dict of sensor data)
- **Output**: Executes the left wall-following strategy for the robot.

Description: Handles robot follow left-wall-follow algorithm
follow left wall functionality in the project.

### `angle_between_vectors`
- **Parameters**: vector1 (tuple), vector2 (tuple)
- **Output**: Returns the angle between two vectors.

Description: Handles angle between vectors functionality in the project.

### `robot_run`
- **Parameters**: robot (VirtualRobot), path (list of tuples)
- **Output**: Runs the robot along the given path, checking for obstacles and staying within boundaries.

Description: Handles run robot
robot run functionality in the project.

### `all_x, all_y = np.meshgrid`
Description: Handles all x, all y = np.meshgrid functionality in the project.

### `visualize_robot_path`
Description: Handles visualizing robot path
visualize robot path functionality in the project.



## iRobot Create3 SDK Installation

To install and configure the iRobot Create3 SDK for your project, follow the steps below:

1. **Install Python Dependencies**
   ```
   pip install irobot-sdk
   pip install opencv-python numpy scipy matplotlib
   ```

2. **Clone the SDK Repository**
   ```
   git clone https://github.com/iRobotEducation/create3-python-sdk.git
   cd create3-python-sdk
   ```

3. **Run the SDK**
   Ensure your robot is powered on and connected to your network, then run the provided examples to test the SDK installation.

4. **Documentation**
   Refer to the [official SDK documentation](https://github.com/iRobotEducation/create3-python-sdk/wiki) for more details.

---

## License

The content of this repository is licensed under the MIT License. See the `LICENSE.md` file for more information.

