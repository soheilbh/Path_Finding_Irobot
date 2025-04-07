from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note
import heapq
import random
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
import scipy as sp
import matplotlib.colors as mcolors
aruco = cv.aruco
import asyncio
import math
from scipy.interpolate import interp1d
from utilities import transform_and_convert_image_to_matrix, imshow_maze_matrix, \
                     maze_astar_solver, summarize_path, clean_up_path, visualize_paths, \
                     angle_between_vectors, is_within_astar_boundaries, \
                     is_within_distance_to_goal, move_robot_to_goal

# Initialize the robot connection
robot = Create3(Bluetooth('Robot 2'))

# Set up initial variables
start = (90, 366)  # Starting position
goal = (360, 110)
maze_height, maze_width = 390, 450
speed, path_distance = 10.0, 15
Distance_Left_far, Distance_Left_close = 90, 250
Distance_Right_close, Distance_Front = 1200, 90
file_path = "robot_movement_history.txt"
algorithm_choice = 1  # Set to 1, 2, or 3 to select the desired algorithm


# Load and solve the maze
maze_matrix = transform_and_convert_image_to_matrix(
    'image_game.jpg', transforming=True, transformed_file_name='maze25',
    maze_height=maze_height, maze_width=maze_width
)
solved = maze_astar_solver(maze_matrix, start, goal, 13)
path, directions = solved
summarized_path = summarize_path(directions)
clean_result = clean_up_path(summarized_path, (0, 0), 15)
cleaned_path, clean_full_path, clean_direction = clean_result
# path+2 = transform_path_to_origin(path)
goal2 = clean_full_path[-1]
# goal2 = path[-1]
print(goal2)

def transform_path_to_origin(path):

    if not path:
        return []
    start_x, start_y = path[0]
    transformed_path = [(x - start_x, y - start_y) for x, y in path]
    return transformed_path

# path_2 = transform_path_to_origin(path)
# goal2=path_2[-1]
# clean_full_path=path_2

# Function to check if within path boundaries
def is_within_path_boundary(path, distance, angle, robot_x, robot_y):
    path_bandwidth = 33
    check_x = int(round(robot_x + distance * np.cos(np.radians(angle))))
    check_y = int(round(robot_y + distance * np.sin(np.radians(angle))))
    for (x1, y1) in path:
        if math.sqrt((check_x - x1) ** 2 + (check_y - y1) ** 2) <= path_bandwidth:
            return True
    return False

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
    turn_angle = -current_heading -90     # Calculate the shortest way to turn back to 0 degrees
    await robot.turn_left(turn_angle)
    await robot.set_lights_blink_rgb(0, 255, 0)
    print(await robot.get_position())

# Event when right bumper is hit
@event(robot.when_bumped, [False, True])
async def bumped_Right(robot):
    print("Bumped right")
    await robot.move(-10)
    await robot.turn_left(45)

# Event when left bumper is hit
@event(robot.when_bumped, [True, False])
async def bumped_Left(robot):
    print("Bumped left")
    await robot.move(-10)
    await robot.turn_right(45)


# Main robot movement function
@event(robot.when_play)
async def test(robot):
    # Initialize file for logging
    robot_position = await robot.get_position()
    print("robot position:",robot_position.x,robot_position.y,robot_position.heading)
    await robot.turn_right(180)
    robot_position = await robot.get_position()
    with open(file_path, "w") as file:
        file.write("____________start_____________\n")
        # Main control loop
        while True:
            robot_movement = []

            # Get robot position and sensor data
            robot_position = await robot.get_position()
            sensor_data = (await robot.get_ir_proximity()).sensors
            left_ir, front_ir, right_ir = sensor_data[0], sensor_data[3], sensor_data[6]
            
            # Adjusted position and headings
            left_check_heading, right_check_heading = robot_position.heading + 90, robot_position.heading - 90
            front_check_heading = robot_position.heading

            # Path boundary checks
            within_astar_left = is_within_path_boundary(clean_full_path, path_distance, left_check_heading, robot_position.x, robot_position.y)
            within_astar_right = is_within_path_boundary(clean_full_path, path_distance, right_check_heading, robot_position.x, robot_position.y)
            within_astar_front = is_within_path_boundary(clean_full_path, path_distance, front_check_heading, robot_position.x, robot_position.y)

            # Update sensor values if out of path
            left_ir = left_ir if within_astar_left else 800
            right_ir = right_ir  if within_astar_right else 800  
            front_ir = front_ir if within_astar_front else 800

            if left_ir == True or right_ir == True or front_ir == True:
                await robot.set_lights_spin_rgb(255, 115, 0)


            if algorithm_choice == 1:
                # Movement decisions
                if left_ir < Distance_Left_far:
                    robot_movement.append('left is open --> turn left')
                    await robot.set_wheel_speeds(-speed, speed)
                    await robot.set_lights_spin_rgb(255, 0, 0)
                if left_ir >= Distance_Left_close:
                    robot_movement.append('left is closed and robot is near that -> turn right slow')
                    await robot.set_wheel_speeds(speed/2, speed /7)
                    await robot.set_lights_spin_rgb(111, 71, 127)
                if right_ir >= Distance_Right_close:
                    robot_movement.append('robot is near right wall --> turn left slow')
                    await robot.set_wheel_speeds(speed / 3, speed)
                    await robot.set_lights_spin_rgb(255, 0, 0)
                if left_ir <= Distance_Left_far: # Go Left
                    await robot.set_wheel_speeds(speed/3,speed)
                    await robot.set_lights_spin_rgb(255, 0, 0)
                if front_ir > Distance_Front:
                    robot_movement.append('front is closed, turn right')
                    await robot.set_wheel_speeds(speed / 2, speed / 7)
                    await robot.set_lights_spin_rgb(0, 0, 255)
                if front_ir > (Distance_Front + 100):
                    robot_movement.append(' front is very close turn right')
                    await robot.set_wheel_speeds(speed, -speed)
                    await robot.set_lights_spin_rgb(0, 0, 255)
                else:
                    robot_movement.append('non of them, streight')
                    await robot.set_wheel_speeds(speed, speed)
                    await robot.set_lights_spin_rgb(0, 0, 255)

            elif algorithm_choice == 2:
                # Original Algorithm 2 (IR Value Based)
                if left_ir < 20:
                    robot_movement.append(' left completly open, turn left')
                    await robot.set_wheel_speeds(speed * 0.5 , speed)  # Turn right
                    await robot.set_lights_spin_rgb(255, 0, 0)
                elif front_ir > Distance_Front and left_ir > 150:
                    robot_movement.append('fornt open , a little bit close, trun right')
                    await robot.set_wheel_speeds(speed, -speed)
                    await robot.set_lights_spin_rgb(111, 71, 127)
                elif front_ir > Distance_Front and left_ir < 80:
                    robot_movement.append('forn open, left completly open, turn right')
                    await robot.set_wheel_speeds(speed, -speed)
                    await robot.set_lights_spin_rgb(111, 71, 127)
                elif left_ir > 400:
                    robot_movement.append('left completly colsed, turn right')
                    await robot.set_wheel_speeds(speed, speed * 0.4)  # Slight turn right
                    await robot.set_lights_spin_rgb(111, 71, 127)
                elif left_ir < 200:
                    robot_movement.append('left  is open, tunr left')
                    await robot.set_wheel_speeds(speed * 0.6, speed)  # Slight turn left
                    await robot.set_lights_spin_rgb(255, 0, 0)
                else:
                    robot_movement.append('non of them, streight')
                    await robot.set_wheel_speeds(speed, speed)
                    await robot.set_lights_spin_rgb(0, 0, 255)

            # Check if goal is reached
            is_whitin_goal = is_within_distance_to_goal(robot_position.x, robot_position.y, goal2[0], goal2[1], 140)
            print("is whitin distance to goal : ", is_whitin_goal )
            if is_whitin_goal:
                await robot.set_lights_spin_rgb(0, 0, 255)
                await robot.set_wheel_speeds(0, 0)
                print("Enabled singing")
                await sing(robot) #Goes to the singing function to play final song
                break

            # Log current position and sensor data
            # log_position(file, robot_position.x, robot_position.y, robot_position.h, sensor_data)
            file.write(f"x={robot_position.x}, y={robot_position.y}, path check: left={within_astar_left}, fron={within_astar_front}, right={within_astar_right}, sensor left= {left_ir}, sensor front= {front_ir},sensor right= {right_ir}, movement: {robot_movement}\n")
            await asyncio.sleep(0.2)  # Reduced sleep time for quicker response

    await robot.set_lights_rgb(0, 255, 0)

# Start the robot play
robot.play()