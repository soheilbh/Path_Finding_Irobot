from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note

#import MazeSolver as MS
import time

robot = Create3(Bluetooth("Robot 2"))

print('🐢 (x  y  heading) =', robot.pose)

prev_pos_x = robot.pose.x
prev_pos_y = robot.pose.y
prev_pos_h = robot.pose.heading
prev_poses = []

async def save_poses(robot):
    prev_poses = prev_poses.append(robot.pose)
    return (prev_poses)


@event(robot.when_play)
async def test(robot):
    while True:
        pos = await robot.get_position()
        if (pos.x - prev_pos_x) >= 10 or (pos.y - prev_pos_y) >= 10:
            save_poses(robot)
            prev_pos_x = pos.x
            prev_pos_y = pos.y
        if (pos.x - prev_pos_x) >= -10 or (pos.y - prev_pos_y) >= -10:
            save_poses(robot)
            prev_pos_x = pos.x
            prev_pos_y = pos.y

        # if 80 <= robot.pose[2] <= 100:
        #     print("robot is facing forward")
        #     await robot.turn_left(90)

        # if 170 <= robot.pose[2] <= 190:
        #     print("robot is facing right")
        #     await robot.turn_left(90)

        # if -10 <= robot.pose[2] <= 10:
        #     print("robot is facing left")
        #     await robot.turn_left(90)

        # if 260 <= robot.pose[2] <= 280:
        #     print("robot is facing backwards")
        #     await robot.turn_left(90)

# Check robot position angle 
    #if angle is between 80 and 100: robot is facing forward
    #if angle is between 170 and 190: robot is facing rigth
    #if angle is between -10 and 10: Robot is facing left
    #if angle is between 260 and 280: robot is facing backwards

robot.play()