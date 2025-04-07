from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note
import time

robot = Create3(Bluetooth('Robot 2'))
State = True

@event(robot.when_play)
async def play(robot):
    while State == True:
        sensors = (await robot.get_ir_proximity()).sensors
        print("Sensor 1:", sensors[0])
        print("Sensor 2:", sensors[1])
        print("Sensor 3:", sensors[2])
        print("Sensor 4:", sensors[3])
        print("Sensor 5:", sensors[4])
        print("Sensor 6:", sensors[5])
        print("Sensor 7:", sensors[6])
        time.sleep(10)
robot.play()