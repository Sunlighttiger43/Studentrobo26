from sr.robot3 import *
import math
robot = Robot()


pi = math.pi

while True:
    robot.sleep(0.05)
    markers = robot.camera.see()
    if len(markers) > 0:
        Yaw = markers[0].position.horizontal_angle
        if Yaw <= pi/12 and Yaw >= -pi/12:
            robot.kch.leds[LED_A].colour = Colour.OFF
            robot.kch.leds[LED_C].colour = Colour.OFF
            robot.kch.leds[LED_B].colour = Colour.RED
        elif Yaw < -pi/12:
            robot.kch.leds[LED_B].colour = Colour.OFF
            robot.kch.leds[LED_C].colour = Colour.OFF
            robot.kch.leds[LED_A].colour = Colour.BLUE
        else:   
            robot.kch.leds[LED_A].colour = Colour.OFF
            robot.kch.leds[LED_B].colour = Colour.OFF
            robot.kch.leds[LED_C].colour = Colour.BLUE
