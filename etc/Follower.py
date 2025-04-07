from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note

#import MazeSolver as MS
import time

robot = Create3(Bluetooth("Robot 2"))

path = [{'direction': 'left', 'cm': 150},
{'direction': 'down', 'cm': 17},
{'direction': 'left', 'cm': 2},
{'direction': 'down', 'cm': 1},
{'direction': 'left', 'cm': 77},
{'direction': 'down', 'cm': 1},
{'direction': 'left', 'cm': 130},
{'direction': 'down', 'cm': 1},
{'direction': 'left', 'cm': 2},
{'direction': 'down', 'cm': 3},
{'direction': 'left', 'cm': 1},
{'direction': 'down', 'cm': 37},
{'direction': 'left', 'cm': 1},
{'direction': 'down', 'cm': 29},
{'direction': 'left', 'cm': 1},
{'direction': 'down', 'cm': 2},
{'direction': 'left', 'cm': 1},
{'direction': 'down', 'cm': 5},
{'direction': 'left', 'cm': 1},
{'direction': 'down', 'cm': 52},
{'direction': 'right', 'cm': 1},
{'direction': 'down', 'cm': 2},
{'direction': 'right', 'cm': 1},
{'direction': 'down', 'cm': 1},
{'direction': 'right', 'cm': 2},
{'direction': 'down', 'cm': 1},
{'direction': 'right', 'cm': 41},
{'direction': 'down', 'cm': 160},
{'direction': 'right', 'cm': 1},
{'direction': 'down', 'cm': 2},
{'direction': 'right', 'cm': 3},
{'direction': 'down', 'cm': 10},
{'direction': 'right', 'cm': 3},
{'direction': 'down', 'cm': 1},
{'direction': 'right', 'cm': 23},
{'direction': 'down', 'cm': 1},
{'direction': 'right', 'cm': 100},
{'direction': 'down', 'cm': 1},
{'direction': 'right', 'cm': 41},
{'direction': 'down', 'cm': 23}]


#print(MS.summarized_path)

@event(robot.when_play)
async def play(robot):
    print(path[0])
    for s in path:
        print(s)
        print('🐢 (x  y  heading) =', robot.pose)

        if s["cm"] <= 3:
                print("too short")
        
        elif s["cm"] > 3:
    
            if s["direction"] == "left":
                await robot.turn_left(90)
                await robot.move(s["cm"])
                await robot.turn_left(-90)
                print('🐢 (x  y  heading) =', robot.pose)
                await robot.reset_navigation()

                time.sleep(5)
                
            if s["direction"] == "right":
                await robot.turn_left(270)
                await robot.move(s["cm"])
                await robot.turn_left(-270)
                print('🐢 (x  y  heading) =', robot.pose)
                await robot.reset_navigation()

                time.sleep(5)
                #robot.move(s["cm"])
                
            if s["direction"] == "down": #Down is up and up is down
                await robot.turn_left(0)
                await robot.move(s["cm"])
                await robot.turn_left(0)
                print('🐢 (x  y  heading) =', robot.pose)
                await robot.reset_navigation()

                time.sleep(5)
                #robot.move(s["cm"])
                
            if s["direction"] == "up": #Up is down and down is up
                await robot.turn_left(180)
                await robot.move(s["cm"])
                await robot.turn_left(-180)
                print('🐢 (x  y  heading) =', robot.pose)
                await robot.reset_navigation()

                time.sleep(5)
                #robot.move(s["cm"])

robot.play()