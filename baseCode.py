
"""
This code has two control modes: 'Tank Mode' and 'Arcade Mode'. The Start
button on your gamepad switches the robot between the two modes.

Controls:

TANK_MODE:
Left Joystick (Up&Down)  - Left wheel
Right Joystick (Up&Down) - Right wheel 
Right Shoulder and Trigger - Move Arm Up&Down

ARCADE_MODE:
Left Joystick (Up&Down)    - Both Wheels
Left Joystick (Left&Right) - Turning
Right Joystick (Up&Down)   - Move Arm Up&Down

____________________________________
##### CONTROLLER DOCUMENTATION #####

A Button     - Autonomous Mode (Discontinued)
Y Button     - Autonomous Mode (For easier rounds) (Discontinued)
B Button     - deliver habitats
X Button (Hold)    - Hold Arm in place when high and stops autonomousMode
Left Shoulder and Trigger - open and close claw
Dpad (Up and down) - Arm Support

Start Button - Switching between tank and arcade modes
___________________________________
##### FINAL CONFIGURATION V1: #####

GPIO_1  goes to the microswitch that determines the upward limit of the arm
motor_left = MOTOR_2    |LEFT drive wheel
motor_right = MOTOR_3   |RIGHT drive wheel
motor_arm = MOTOR_4     |motor for main arm motion
servo_claw = SERVO_1    |servo for arm claw
servo_habitat = SERVO_2 |sigma pushing fanum tax (three sigmas and two tax frauds)
"""

import board
import pwmio
import digitalio
import time
from adafruit_motor import servo
from adafruit_simplemath import map_range, constrain
from circuitpython_gizmo import Gizmo

gizmo = Gizmo()

# Hardware constants

pwm_freq = 50  # Hertz
min_pulse = 1000  # milliseconds
max_pulse = 2000  # milliseconds
servo_range = 180  # degrees
servo_min_pulse = 500
servo_max_pulse = 2400

# Configure the motors & servos for the ports they are connected to
motor_left = servo.ContinuousServo(
    pwmio.PWMOut(gizmo.MOTOR_2, frequency=pwm_freq),
    min_pulse=min_pulse,
    max_pulse=max_pulse,
)
motor_right = servo.ContinuousServo(
    pwmio.PWMOut(gizmo.MOTOR_3, frequency=pwm_freq),
    min_pulse=min_pulse,
    max_pulse=max_pulse,
)
motor_support = servo.ContinuousServo(
    pwmio.PWMOut(gizmo.MOTOR_1, frequency=pwm_freq),
    min_pulse=min_pulse,
    max_pulse=max_pulse
)
motor_arm = servo.ContinuousServo(
    pwmio.PWMOut(gizmo.MOTOR_4, frequency=pwm_freq),
    min_pulse=min_pulse,
    max_pulse=max_pulse,
)
servo_claw = servo.Servo(
    pwmio.PWMOut(gizmo.SERVO_1, frequency=pwm_freq),
    actuation_range=servo_range,
    min_pulse=servo_min_pulse,
    max_pulse=servo_max_pulse,
)
servo_habitat = servo.Servo(
    pwmio.PWMOut(gizmo.SERVO_2, frequency=pwm_freq),
    actuation_range=servo_range,
    min_pulse=servo_min_pulse,
    max_pulse=servo_max_pulse,
)
# Configure the Sensors and Leds
led = digitalio.DigitalInOut(board.LED)
led.direction = digitalio.Direction.OUTPUT

upLimitSwitch = digitalio.DigitalInOut(gizmo.GPIO_1)
upLimitSwitch.switch_to_input()

# what is this elia??? # unused limit switch that is supposed to be down
clawSensor = digitalio.DigitalInOut(gizmo.GPIO_2)
clawSensor.switch_to_input()

# Mode
TANK_MODE = 0
ARCADE_MODE = 1
mode = ARCADE_MODE
prev_start_button = False
#  start on arcade mode as default

# Useful variables
servo_speed = 1
arm_speed = 1
previous_servo_angle = 160
max_angle = 160
min_angle = 75

# Constants
steering_constant = -1
habitat_push_constant = 80

# Useful Functions
def moveLeftWheel(input):  # -1 <input <1
    throttle_left = -input
    motor_left.throttle = throttle_left


def moveRightWheel(input):  # -1 <input <1
    throttle_right = input
    motor_right.throttle = throttle_right


# looks like something for auton
def moveBothWheels(input):  # -1 < input <1
    moveLeftWheel(input)
    moveRightWheel(input)


def moveArm(input):  # -1 <input <1
    throttle_task = input * arm_speed
    motor_arm.throttle = throttle_task

def moveArmSupport(input):  # -1 < input <1
    throttle_support = input
    motor_support.throttle = throttle_support

# elia what is this dogshit code!!!!! # bro is roasting me. Sorry albert :-(
# go over python objects and create a new object class for servos for our use
# may not need it tho # Nah it works already, I'm lazy
def setAndReturnServo1angle(angle):
    servo_angle = constrain(previous_servo_angle + angle, min_angle, max_angle)
    servo_claw.angle = servo_angle
    return servo_angle


def pushHabitat(input):
    if input:
        servo_habitat.angle = habitat_push_constant
    else:
        servo_habitat.angle = 0


def switchLed(seconds):  # turns the led and sleep for a determined ammount of time
    led.value = not led.value
    time.sleep(seconds)


def getServoAngle():  # servo_arm
    angle = 0
    if gizmo.buttons.left_trigger:
        angle += 0.8
    elif gizmo.buttons.left_shoulder:
        angle -= 0.8
    return angle * servo_speed

def getSupportInput():
    input = map_range(gizmo.axes.dpad_y, 0, 255, -1.0, 1.0)
    return input


def getMotorTaskInput(isArcadeMode):
    # removing to use gizmo.button.b for other things
    inputMult = 1
    if gizmo.buttons.x:
        input = 0.4
        inputMult = 0.33
    else:
        input = 0.2

    if isArcadeMode:  # joystick input is now set to a   # A WHAT ALBERT A WHAT????
        input += -map_range(gizmo.axes.right_y, 0, 255, -1.0, 1)
        if input < 0:
            input *= inputMult
    else:
        if gizmo.buttons.right_trigger:
            input += 0.7
        elif gizmo.buttons.right_shoulder:
            input -= 1.0
            input *= inputMult
 
    if input > 1.0:
        input = 1.0
    if upLimitSwitch.value and input > 0 and not gizmo.buttons.x:
        return 0.4
    elif upLimitSwitch.value and input < 0 and gizmo.buttons.x:
        return -0.4
    return input


# auton
def autonomousMode():
    setAndReturnServo1angle(-180)
    while not upLimitSwitch.value:
        moveArm(0.8)
        gizmo.refresh()
        if gizmo.buttons.x:
            break
    while not clawSensor.value:
        moveBothWheels(1)
        gizmo.refresh()
        if gizmo.buttons.x:
            break

    moveBothWheels(-0.3)
    time.sleep(1.0)
    moveBothWheels(0)

    moveArm(-0.3)
    time.sleep(0.2)
    moveArm(0.3)

    moveBothWheels(0.3)
    time.sleep(1)
    moveBothWheels(0)

    moveArm(-0.5)
    setAndReturnServo1angle(180)
    time.sleep(1.0)

    moveArm(0.8)
    time.sleep(0.2)
    moveArm(0.3)

    moveBothWheels(1)
    time.sleep(0.25)
    moveBothWheels(0)

    setAndReturnServo1angle(-120)
    time.sleep(0.4)

    while True:
        moveBothWheels(-1)
        gizmo.refresh()
        if gizmo.buttons.x:
            break


def autonomousEasyMode():
    setAndReturnServo1angle(-180)
    while not upLimitSwitch.value:
        moveArm(1)
        if gizmo.buttons.x:
            return

    moveBothWheels(-1)
    time.sleep(4.99)

    moveBothWheels(0.3)
    time.sleep(1.0)
    moveBothWheels(0)

    moveArm(-1)
    setAndReturnServo1angle(180)
    time.sleep(0.875)
    moveArm(0.3)

    moveBothWheels(-1)
    time.sleep(0.3141529)
    moveBothWheels(0)
    setAndReturnServo1angle(-180)
    while True:
        moveBothWheels(1)
        gizmo.refresh()
        if gizmo.buttons.x:
            break


# Keep running forever
"""
##### MAIN LOOP #####
"""
while True:
    gizmo.refresh()
    switchLed(0)
    # print(map_range(gizmo.axes.left_y, 0, 255, -1.0, 1.0))

    # modes
    if gizmo.buttons.start and not prev_start_button:
        if mode == TANK_MODE:
            mode = ARCADE_MODE 
        elif mode == ARCADE_MODE:
            mode = TANK_MODE
        prev_start_button = gizmo.buttons.start
    elif not gizmo.buttons.start == prev_start_button:
        prev_start_button = gizmo.buttons.start

    # move
    if mode == TANK_MODE:
        moveLeftWheel(map_range(gizmo.axes.left_y, 0, 255, -1.0, 1.0))  # left
        moveRightWheel(map_range(gizmo.axes.right_y, 0, 255, -1.0, 1.0))  # right
    elif mode == ARCADE_MODE:
        # Mix right joystick axes to control both wheels
        speed = map_range(gizmo.axes.left_y, 0, 255, -1.0, 1.0)
        steering = steering_constant * map_range(gizmo.axes.left_x, 0, 255, -1.0, 1.0)
        # steering constant is set to -1
        moveLeftWheel(constrain(speed + steering, -1.0, 1.0))
        moveRightWheel(constrain(speed - steering, -1.0, 1.0))

    # arms
    moveArm(getMotorTaskInput(mode == ARCADE_MODE))
    moveArmSupport(getSupportInput())
    previous_servo_angle = setAndReturnServo1angle(getServoAngle())
    pushHabitat(gizmo.buttons.b)
'''
    if gizmo.buttons.a:
        autonomousMode()
    if gizmo.buttons.y:
        autonomousEasyMode()
# Write your code here :-)
'''
