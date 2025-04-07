from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note
import numpy as np

robot = Create3(Bluetooth('Robot 2'))
state = True

"Sensor Angles from Datasheet"
Sensor_Angles = [-63.3,-38.0, -20.0, -3.0, 14.25, 34, 65.3] 

"Sensor Calibration Equations (May vary with different lighting)"



def print_pos(robot):
    print('🐢 (x  y  angle) =', robot.pose)


"""
If your starting point is (0,0), and your new point is r units away at an angle of θ, 
you can find the coordinates of that point using the equations x = r cosθ and y = r sinθ.
"""

def get_object(robot):
    Sensor_Distance = (robot.get_ir_proximity()).sensors
    for distance in Sensor_Distance:
        if distance >= 900:
            Sensor_Number = Sensor_Distance.index(distance)
    return Sensor_Number, distance
    
def calc_object(robot, Sensor_Number, distance): 
    x = robot.pose[0]
    y = robot.pose[1]
    Robot_Angle = robot.pose[2]
    theta = [Robot_Angle + angle for angle in Sensor_Angles]
    x_object = distance * np.cos(theta[Sensor_Number])
    y_object = distance * np.sin(theta[Sensor_Number])
    return x, y, x_object, y_object

@event(robot.when_play)
async def play(robot):
    while state == True:
        get_object(robot)
        calc_object(robot)
        
    

robot.play()
