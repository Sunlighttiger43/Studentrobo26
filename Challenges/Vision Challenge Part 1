from sr.robot3 import *

robot = Robot()

markers = robot.camera.see()

while True:
    try:
        markersA = robot.camera.see()
        marker = markersA[0]
        # qr code scanner
        print(marker)
        if marker.position.horizontal_angle>315 or marker.position.horizontal_angle<45:
        
            pos = marker.position.distance
            if pos > 1500:
                robot.kch.leds[LED_B].colour = Colour.RED
            if pos >=300 and pos<=1500:
                robot.kch.leds[LED_B].colour = Colour.BLUE
            if pos<300:
                robot.kch.leds[LED_B].colour = Colour.GREEN
    except Exception as e:
        print(e)
        print("AHAHHAAH I'M BLIND")
        
# made last year
