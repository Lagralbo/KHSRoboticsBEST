"""
This code has two control modes: 'Tank Mode' and 'Arcade Mode'. The Start
button on your gamepad switches the robot between the two modes.

Here are the controls for Tank Mode:
Left Joystick Up/Down    - Motor 1 Fwd/Rev
Right Joystick Up/Down   - Motor 2 Fwd/Rev

Here are the controls for Arcade Mode:
Left Joystick Up/Down    - Robot Fwd/Rev
Left Joystick Left/Right - Robot Turn Left/Right

These controls work in both modes:
Right Trigger            - Motor 4 Forward
Right Shoulder Button    - Motor 4 Reverse
Left Trigger             - Servo 1 to 0 degrees
Left Shoulder Button     - Servo 1 to 90 degrees

Confinguration:
SERVO_1 goes to the motor that moves the arm up and down
SERVO_2 goes to the motor that controls the left wheel
SERVO_3 goes to the motor that controls the right wheel
SERVO_4 should go to a servo that opens and closes the arm(?)

When neither the left trigger nor shoulder button are pressed, the servo will
go to 45 degrees.
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
servo_range = 90  # degrees

# Configure the motors & servos for the ports they are connected to
motor_left = servo.ContinuousServo(
    pwmio.PWMOut(gizmo.SERVO_2, frequency=pwm_freq),
    min_pulse=min_pulse,
    max_pulse=max_pulse
)
motor_right = servo.ContinuousServo(
    pwmio.PWMOut(gizmo.SERVO_3, frequency=pwm_freq),
    min_pulse=min_pulse,
    max_pulse=max_pulse
)
motor_task = servo.ContinuousServo(
    pwmio.PWMOut(gizmo.SERVO_1, frequency=pwm_freq),
    min_pulse=min_pulse,
    max_pulse=max_pulse
)
servo_task = servo.Servo(
    pwmio.PWMOut(gizmo.SERVO_4, frequency=pwm_freq),
    actuation_range=servo_range,
    min_pulse=min_pulse,
    max_pulse=max_pulse
)

# Configure the Sensors and Leds
led = digitalio.DigitalInOut(board.LED)
led.direction = digitalio.Direction.OUTPUT
upLimitSwitch = digitalio.DigitalInOut(gizmo.GPIO_1)
upLimitSwitch.switch_to_input()
downLimitSwitch = digitalio.DigitalInOut(gizmo.GPIO_2)
downLimitSwitch.switch_to_input()

# Mode
TANK_MODE = 0
ARCADE_MODE = 1
mode = ARCADE_MODE
prev_start_button = False

# Useful variables
servo_speed = 1
arm_speed = 1
previous_servo_angle = 0
isAutonomous = False

# Useful Functions
def moveLeftWheel(input):  # -1 <input <1
    throttle_left = input
    motor_left.throttle = throttle_left

def moveRightWheel(input):  # -1 <input <1
    throttle_right = input
    motor_right.throttle = throttle_right

def moveArm(input):  # -1 <input <1
    throttle_task = input * arm_speed
    motor_task.throttle = throttle_task

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
        angle += 1
    elif gizmo.buttons.left_shoulder:
        angle -= 1
    return angle * servo_speed

def getMotorTaskInput(isArcadeMode):
    input = 0.2 
    if (isArcadeMode): 
        input = -map_range(gizmo.axes.right_y, 0, 255, -1.0, 1.0)
    else: 
        if gizmo.buttons.right_trigger:
            input += 0.7
        elif gizmo.buttons.right_shoulder and not downLimitSwitch.value:
            input -= 0.7
    return input
def autonomousMode(): 
    while not upLimitSwitch.value:
        moveArm(1)
        if (gizmo.buttons.x):
            break
    moveArm(0.2)
    time.sleep(1.0) 
    moveArm(-0.2)
    time.sleep(1.5)
    
    
# Keep running forever
while True:
    gizmo.refresh()
    switchLed(0)
    # print(upLimitSwitch.value)

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
    previous_servo_angle = setServo1angle(getServoAngle())

    if (gizmo.buttons.a):
        isAutonomous = True 
        autonomousMode()
        isAutonomous = False
    

