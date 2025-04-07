from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, Color, Robot, Create3
from irobot_edu_sdk.music import Note
import random
import cv2 as cv
import numpy as np
import asyncio
import math
from utilities import transform_and_convert_image_to_matrix, maze_astar_solver, summarize_path, \
                     clean_up_path, is_within_distance_to_goal

# Connect to the robot via Bluetooth
robot = Create3(Bluetooth('Robot 2'))

# Set up variables with user prompts and default values
maze_width = input("Enter the maze width (default is 450): ")
maze_width = int(maze_width) if maze_width else 450

maze_height = input("Enter the maze height (default is 390): ")
maze_height = int(maze_height) if maze_height else 390

speed = input("Enter the robot speed (default is 10.0): ")
speed = float(speed) if speed else 10.0

path_distance = input("Enter the path distance (default is 15): ")
path_distance = int(path_distance) if path_distance else 15

first_turn = input("Enter the initial turn angle (default is 180): ")
first_turn = int(first_turn) if first_turn else 180

clean_path_selection = input("Use clean path selection? (True/False, default is True): ")
clean_path_selection = clean_path_selection.lower() == 'true' if clean_path_selection else True

maze_image_path = input("Enter the path for the maze image file (default is 'image20.jpg'): ")
maze_image_path = maze_image_path if maze_image_path else 'image_game.jpg'

maze_image_transformed_name = input("Enter the transformed maze image file name (default is 'image_game'): ")
maze_image_transformed_name = maze_image_transformed_name if maze_image_transformed_name else 'image_game'


# Additional parameters
Distance_Left_far, Distance_Left_close = 90, 250
Distance_Right_close, Distance_Front = 1200, 90
file_path = "robot_movement_history.txt"


# Load maze and solve using A* algorithm
maze_matrix, start, goal = transform_and_convert_image_to_matrix(
    image_path=maze_image_path, start_image_path='start_point.png', goal_image_path='end_point.png',
    transforming=True, transformed_file_name=maze_image_transformed_name,
    maze_height=maze_height, maze_width=maze_width
)
solved = maze_astar_solver(maze_matrix, start, goal, 12)
path, directions = solved
summarized_path = summarize_path(directions)
clean_result = clean_up_path(summarized_path, (0, 0), 15)
cleaned_path, clean_full_path, clean_direction = clean_result
goal_00 = clean_full_path[-1]
print(goal_00)

# Function to transform path to origin for comparison
def transform_path_to_origin(path):
    if not path:
        return []
    start_x, start_y = path[0]
    return [(x - start_x, y - start_y) for x, y in path]

# Select whether to use clean path
if not clean_path_selection:
    path_00 = transform_path_to_origin(path)
    goal_00 = path_00[-1]
    clean_full_path = path_00

# Check if robot is within specified distance to path boundaries
def is_within_path_boundary(path, distance, angle, robot_x, robot_y):
    path_bandwidth = 35
    check_x = int(round(robot_x + distance * np.cos(np.radians(angle))))
    check_y = int(round(robot_y + distance * np.sin(np.radians(angle))))
    for (x1, y1) in path:
        if math.sqrt((check_x - x1) ** 2 + (check_y - y1) ** 2) <= path_bandwidth:
            return True
    return False

# Play a melody and flash lights when goal is reached
async def sing(robot):
    starwars_song = [
        (Note.G4, Note.HALF), (Note.G4, Note.HALF), (Note.G4, Note.HALF),
        (Note.D4_SHARP, Note.EIGHTH),
        (Note.A4_SHARP, Note.EIGHTH),
        (Note.G4, Note.EIGHTH),
        (Note.D4_SHARP, Note.EIGHTH),
        (Note.A4_SHARP, Note.EIGHTH),
        (Note.G4, Note.WHOLE),
        (Note.D5, Note.HALF), (Note.D5, Note.HALF), (Note.D5, Note.HALF),
        (Note.D5_SHARP, Note.EIGHTH),
        (Note.A4_SHARP, Note.EIGHTH),
        (Note.G4, Note.HALF),
        (Note.D4_SHARP, Note.EIGHTH),
        (Note.A4_SHARP, Note.EIGHTH),
        (Note.G4, Note.WHOLE),
]

    await robot.set_wheel_speeds(10, -10)

    for note in starwars_song:
        await robot.play_note(note[0], note[1])

        # Generate random RGB values for lights for every note 
        r, g, b = random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)
        await robot.set_lights_spin_rgb(r, g, b)

    await robot.set_wheel_speeds(0, 0)
    pose = await robot.get_position()  # Get the current position
    current_heading = pose.heading     # Extract the heading (angle)
    turn_angle = -current_heading -90    # Calculate the shortest way to turn back to 0 degrees
    await robot.turn_left(turn_angle)
    await robot.set_lights_blink_rgb(0, 255, 0)
    print(await robot.get_position())

# Event handler for right bumper hit
@event(robot.when_bumped, [False, True])
async def bumped_Right(robot):
    print("Bumped right")
    await robot.move(-10)
    await robot.turn_left(45)

# Event handler for left bumper hit
@event(robot.when_bumped, [True, False])
async def bumped_Left(robot):
    print("Bumped left")
    await robot.move(-10)
    await robot.turn_right(45)

# Main function controlling robot movement
@event(robot.when_play)
async def test(robot):
    # Log start position
    robot_position = await robot.get_position()
    print("robot position:", robot_position.x, robot_position.y, robot_position.heading)
    await robot.turn_right(first_turn) 
    
    # Write initial log
    with open(file_path, "w") as file:
        file.write("____________start_____________\n")
        
        while True:
            robot_movement = []

            # Get current position and sensor readings
            robot_position = await robot.get_position()
            sensor_data = (await robot.get_ir_proximity()).sensors
            left_ir, front_ir, right_ir = sensor_data[0], sensor_data[3], sensor_data[6]
            
            # Set headings and path checks
            left_check_heading, right_check_heading = robot_position.heading + 90, robot_position.heading - 90
            front_check_heading = robot_position.heading
            within_astar_left = is_within_path_boundary(clean_full_path, path_distance, left_check_heading, robot_position.x, robot_position.y)
            within_astar_right = is_within_path_boundary(clean_full_path, path_distance, right_check_heading, robot_position.x, robot_position.y)
            within_astar_front = is_within_path_boundary(clean_full_path, path_distance, front_check_heading, robot_position.x, robot_position.y)

            # Adjust IR sensor values if robot is out of path
            left_ir = left_ir if within_astar_left else 800
            right_ir = right_ir  if within_astar_right else 800
            front_ir = front_ir if within_astar_front else 800

            # Define movement based on sensor data
            if left_ir < Distance_Left_far:
                robot_movement.append('left is open --> turn left')
                await robot.set_wheel_speeds(-speed, speed)
            if left_ir >= Distance_Left_close:
                robot_movement.append('left is closed and robot is near that -> turn right slow')
                await robot.set_wheel_speeds(speed / 2, speed / 7)
            if right_ir >= Distance_Right_close:
                robot_movement.append('robot is near right wall --> turn left slow')
                await robot.set_wheel_speeds(speed / 3, speed)
            if left_ir <= Distance_Left_far:
                await robot.set_wheel_speeds(speed / 3, speed)
            if front_ir > Distance_Front:
                robot_movement.append('front is closed, turn right')
                await robot.set_wheel_speeds(speed / 2, speed / 7)
            if front_ir > (Distance_Front + 100):
                robot_movement.append('front is very close, turn right')
                await robot.set_wheel_speeds(speed, -speed)

            # Check if robot reached the goal
            is_within_goal = is_within_distance_to_goal(robot_position.x, robot_position.y, goal_00[0], goal_00[1], 120)
            print("is within distance to goal: ", is_within_goal)
            if is_within_goal:
                await robot.set_lights_spin_rgb(0, 0, 255)
                await robot.set_wheel_speeds(0, 0)
                print("Enabled singing")
                await sing(robot) #Goes to the singing function to play final song
                break

            # Log current position, sensor data, and movements
            file.write(f"x={robot_position.x}, y={robot_position.y}, path check: left={within_astar_left}, front={within_astar_front}, right={within_astar_right}, sensor left= {left_ir}, sensor front= {front_ir}, sensor right= {right_ir}, movement: {robot_movement}\n")
            await asyncio.sleep(0.2)  # Reduced sleep for faster response

    await robot.set_lights_rgb(0, 255, 0)

# Start robot play mode
robot.play()