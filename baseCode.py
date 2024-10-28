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
COMMON: 
A Button     - Autonomous Mode
Y Button     - Autonomous Mode (For easier rounds)
X Button     - Stop Autonomous Mode
B Button     - Hold Arm in place when high

Start Button - Switching between tank and arcade modes

Confinguration:
GPIO_1  goes to the microswitch that determines the upward limit of the arm
GPIO_2  goes to the microswitch that detects a collision with the bottom of the arm
MOTOR_2 goes to the motor that controls the left wheel
MOTOR_3 goes to the motor that controls the right wheel
MOTOR_4 goes to the motor that moves the arm up and down
SERVO_1 should go to a servo that opens and closes the arm claw
"""

import board
import pwmio
import digitalio
import time
from adafruit_motor import servo
from adafruit_simplemath import map_range, constrain
from circuitpython_gizmo import Gizmo

gizmo = Gizmo()

pwm_freq = 50  # Hertz
min_pulse = 1000  # milliseconds
max_pulse = 2000  # milliseconds
servo_range = 180  # degrees

# Configure the motors & servos for the ports they are connected to
motor_left = servo.ContinuousServo(
    pwmio.PWMOut(gizmo.MOTOR_2, frequency=pwm_freq),
    min_pulse=min_pulse,
    max_pulse=max_pulse
)
motor_right = servo.ContinuousServo(
    pwmio.PWMOut(gizmo.MOTOR_3, frequency=pwm_freq),
    min_pulse=min_pulse,
    max_pulse=max_pulse
)
motor_stick = servo.ContinuousServo(
    pwmio.PWMOut(gizmo.MOTOR_1, frequency=pwm_freq),
    min_pulse=min_pulse,
    max_pulse=max_pulse
)
motor_task = servo.ContinuousServo(
    pwmio.PWMOut(gizmo.MOTOR_4, frequency=pwm_freq),
    min_pulse=min_pulse,
    max_pulse=max_pulse
)
servo_task = servo.Servo(
    pwmio.PWMOut(gizmo.SERVO_1, frequency=pwm_freq),
    actuation_range=servo_range,
    min_pulse=min_pulse - 500,
    max_pulse=max_pulse + 400,
)

# Configure the Sensors and Leds
led = digitalio.DigitalInOut(board.LED)
led.direction = digitalio.Direction.OUTPUT
upLimitSwitch = digitalio.DigitalInOut(gizmo.GPIO_1)
upLimitSwitch.switch_to_input()
clawSensor = digitalio.DigitalInOut(gizmo.GPIO_2)
clawSensor.switch_to_input()

# Mode
TANK_MODE = 0
ARCADE_MODE = 1
mode = TANK_MODE
prev_start_button = False

# Useful variables
servo_speed = 1
arm_speed = 1
previous_servo_angle = 0

# Useful Functions
def moveLeftWheel(input):  # -1 <input <1
    throttle_left = -input
    motor_left.throttle = throttle_left

def moveRightWheel(input):  # -1 <input <1
    throttle_right = input
    motor_right.throttle = throttle_right

def moveBothWheels(input):  # -1 < input <1
    moveLeftWheel(input)
    moveRightWheel(input)

def moveArm(input):  # -1 <input <1
    throttle_task = input * arm_speed
    motor_task.throttle = throttle_task
    
def moveStick(input):  # -1 < input <1
    throttle_stick = input
    motor_stick.throttle = throttle_stick

def setServo1angle(angle):
    servo_angle = constrain(previous_servo_angle + angle, 0, servo_range)
    servo_task.angle = servo_angle
    return servo_angle

def switchLed(seconds):  # turns the led and sleep for a determined ammount of time
    led.value = not led.value
    time.sleep(seconds)

def getServoAngle():
    angle = 0
    if gizmo.buttons.left_trigger:
        angle += 0.8
    elif gizmo.buttons.left_shoulder:
        angle -= 0.8
    return angle * servo_speed

def getStickInput():
    input = map_range(gizmo.axes.dpad_y, 0, 255, -1.0, 1.0)
    return input

def getMotorTaskInput(isArcadeMode):
    if (gizmo.buttons.b):
        input = 0.4
    else:
        input = 0.2
    if (isArcadeMode):
        input += -map_range(gizmo.axes.right_y, 0, 255, -1.0, 1.0)
        if (input < -0.7):
            input = -1
    else:
        if gizmo.buttons.right_trigger:
            input += 0.7
        elif gizmo.buttons.right_shoulder:
            input -= 1.2

    if (input > 1.0):
        input = 1.0
    if (upLimitSwitch.value):
        return 0.4
    return input
def autonomousMode():
    setServo1angle(-180)
    while not upLimitSwitch.value:
        moveArm(0.8)
        gizmo.refresh()
        if (gizmo.buttons.x):
            break
    while not clawSensor.value:
        moveBothWheels(1)
        gizmo.refresh()
        if (gizmo.buttons.x):
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
    setServo1angle(180)
    time.sleep(1.0)

    moveArm(0.8)
    time.sleep(0.2)
    moveArm(0.3)

    moveBothWheels(1)
    time.sleep(0.25)
    moveBothWheels(0)

    setServo1angle(-120)
    time.sleep(0.4)

    while True:
        moveBothWheels(-1)
        gizmo.refresh()
        if (gizmo.buttons.x):
            break

def autonomousEasyMode():
    setServo1angle(-180)
    while not upLimitSwitch.value:
        moveArm(1)
        if (gizmo.buttons.x):
            return

    moveBothWheels(-1)
    time.sleep(4.99)

    moveBothWheels(0.3)
    time.sleep(1.0)
    moveBothWheels(0)

    moveArm(-1)
    setServo1angle(180)
    time.sleep(0.875)
    moveArm(0.3)

    moveBothWheels(-1)
    time.sleep(0.3141529)
    moveBothWheels(0)
    setServo1angle(-180)
    while True:
        moveBothWheels(1)
        gizmo.refresh()
        if (gizmo.buttons.x):
            break
# Keep running forever
while True:
    gizmo.refresh()
    switchLed(0) 
    # print(getStickInput())
    
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
        steering = map_range(gizmo.axes.left_x, 0, 255, -1.0, 1.0)

        moveLeftWheel(constrain(speed + steering, -1.0, 1.0))
        moveRightWheel(constrain(speed - steering, -1.0, 1.0))

    # arms
    moveArm(getMotorTaskInput(mode == ARCADE_MODE))
    moveStick(getStickInput())
    previous_servo_angle = setServo1angle(getServoAngle()) 

    if (gizmo.buttons.a):
        autonomousMode()
    if (gizmo.buttons.y):
        autonomousEasyMode()
