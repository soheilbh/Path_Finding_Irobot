#
# Licensed under 3-Clause BSD license available in the License file. Copyright (c) 2021-2023 iRobot Corporation. All rights reserved.
#

from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note
import matplotlib.pyplot as plt
import numpy as np

robot = Create3(Bluetooth('Robot 2'))

speed = 10.0
DL = 40
DF = 100
frontSensorsDiff = 200
sensorLeftAvg = 0
sensorFrontAvg = 0
all_data = []


@event(robot.when_play)
async def test(robot):
    await robot.set_wheel_speeds(speed,speed)
    count = 0
    while count<50:

        sensors = (await robot.get_ir_proximity()).sensors
        # print(f"sensor 2:{sensors[2]} sensor 3:{sensors[3]} sensor4{sensors[4]}")
        sensorLeftAvg = sensors[0]
        sensorFrontAvg = sensors[3]

        if sensorLeftAvg > DL  :
            await robot.set_wheel_speeds(speed,speed)

        if sensorLeftAvg < DL:
            print(f"******************sensor 0:{sensors[3]}**********************")
            await robot.set_wheel_speeds(speed/3,speed)

        if sensorFrontAvg > DF :

            # await robot.set_wheel_speeds(speed,3)
            print(f"_______________________________sensor 3:{sensors[3]}___________________________")
            await robot.turn_right(45)
            # if sensors[2] > DF+100 and sensors[4] > DF+f100 :
            #     await robot.set_wheel_speeds(speed/2,speed/6)
            # else:
            #     if sensors[2] < DF :
            #         await robot.turn_left(45)
            #     if sensors[4] < DF :
            #         await robot.turn_right(45)
        
        # print(f"move number : {count} , position{await robot.get_position()}, sensor values: {sensors}")
        
        await robot.get_position()

        all_data.append([robot.pose.x,robot.pose.y,robot.pose.heading]+sensors)
        print(f"movment {count} : all data is : {all_data[count]}")
        count += 1
        print("_______________________________________p")

        if np.mod(count,20) == 19 :
            print("_______________________________________p")
            all_pos_x = [data[0] for data in all_data]
            all_pos_y = [data[1] for data in all_data]
            plt.plot(all_pos_x, all_pos_y)
            plt.savefig("/Users/soheil/Desktop/Robot\ Project /plotrobot"+str(count)+".png")

   
robot.play()

# # if both left and right open: 
# #     if (np.random.rand() < 0.8) & (turn_left_cnt%4==0):
# #         turn left 
# #     else:
# #         turn right 