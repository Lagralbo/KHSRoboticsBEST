"""
This code has two control modes: 'Tank Mode' and 'Arcade Mode'. The Start
button on your gamepad switches the robot between the two modes.

Here are the controls for Tank Mode:
Left Joystick Up/Down    - Motor 4 Fwd/Rev
Right Joystick Up/Down   - Motor 3 Fwd/Rev

Here are the controls for Arcade Mode:
Left Joystick Up/Down    - Robot Fwd/Rev
Left Joystick Left/Right - Robot Turn Left/Right

These controls work in both modes:
Right Trigger            - Motor 4 Forward
Right Shoulder Button    - Motor 4 Reverse
Left Trigger             - Servo 1 to 0 degrees
Left Shoulder Button     - Servo 1 to 90 degrees

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
    pwmio.PWMOut(gizmo.MOTOR_4, frequency=pwm_freq),
    min_pulse=min_pulse,
    max_pulse=max_pulse
)
motor_right = servo.ContinuousServo(
    pwmio.PWMOut(gizmo.MOTOR_3, frequency=pwm_freq),
    min_pulse=min_pulse,
    max_pulse=max_pulse
)
motor_task = servo.ContinuousServo(
    pwmio.PWMOut(gizmo.MOTOR_1, frequency=pwm_freq),
    min_pulse=min_pulse,
    max_pulse=max_pulse
)
servo_task = servo.Servo(
    pwmio.PWMOut(gizmo.SERVO_1, frequency=pwm_freq),
    actuation_range=servo_range,
    min_pulse=min_pulse,
    max_pulse=max_pulse
)

# Configure the built-in LED pin as an output
led = digitalio.DigitalInOut(board.LED)
led.direction = digitalio.Direction.OUTPUT

# Mode
TANK_MODE = 0
ARCADE_MODE = 1
mode = TANK_MODE
prev_start_button = False

# Useful Functions
def moveLeftWheel(input):  # -1 <input <1
    throttle_left = input
    motor_left.throttle = throttle_left

def moveRightWheel(input):  # -1 <input <1
    throttle_right = input
    motor_right.throttle = throttle_right

def moveMotor1(input):  # -1 <input <1
    throttle_task = input
    motor_task.throttle = throttle_task

def setServo1angle(angle):
    servo_angle = angle
    servo_task.angle = servo_angle

def switchLed(seconds):  # turns the led and sleep for a determined ammount of time
    led.value = not led.value
    time.sleep(seconds)

def switchOrGetMode():
    """
    nonlocal mode
    if mode == TANK_MODE:
        mode = ARCADE_MODE
    elif mode == ARCADE_MODE:
        mode = TANK_MODE
    nonlocal prev_start_button
    prev_start_button = gizmo.buttons.start
    """

def getServoAngle():
    angle = 45
    if gizmo.buttons.left_trigger:
        angle = 0
    elif gizmo.buttons.left_shoulder:
        angle = 90
    return angle

def getMotorTaskInput():
    input = 0
    if gizmo.buttons.right_trigger:
        input = 1.0
    elif gizmo.buttons.right_shoulder:
        input = -1.0
    return input


# Keep running forever
while True:
    gizmo.refresh()
    switchLed(0)

    # modes
    if gizmo.buttons.start and not prev_start_button:
        if mode == TANK_MODE:
            mode = ARCADE_MODE
        elif mode == ARCADE_MODE:
            mode = TANK_MODE
            mode = TANK_MODE
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

        moveLeftWheel(constrain(speed - steering, -1.0, 1.0))
        moveRightWheel(constrain(speed + steering, -1.0, 1.0))

    # arms
    moveMotor1(getMotorTaskInput())
    setServo1angle(getServoAngle())
