from adafruit_servokit import ServoKit

kit = ServoKit(channels=16, address=0x40)

kit.servo[0].angle = 89
kit.servo[1].angle = 89
kit.servo[2].angle = 90
