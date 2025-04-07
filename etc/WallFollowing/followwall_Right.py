from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note

robot = Create3(Bluetooth('Robot 2'))

speed = 15.0

Distance_Right_far = 90
Distance_Right_close = 200
Distance_Front = 90

@event(robot.when_play)
async def test(robot):
    await robot.set_wheel_speeds(speed,speed)
    while True:
        sensors = (await robot.get_ir_proximity()).sensors
        SensorFront = sensors[3]
        SensorRight = sensors[6]

        if SensorRight < Distance_Right_far: # Go straight
            await robot.set_wheel_speeds(speed,speed)

        if SensorRight >= Distance_Right_close: # Go Right
            await robot.set_wheel_speeds(speed/3,speed)

        if SensorRight <= Distance_Right_far: # Go Left
            await robot.set_wheel_speeds(speed,speed/3)

        if SensorFront > Distance_Front : # Go right
            await robot.set_wheel_speeds(speed/7,speed/2)
        
        if SensorFront > (Distance_Front+100) :
            await robot.set_wheel_speeds(-speed,speed)
    
robot.play()