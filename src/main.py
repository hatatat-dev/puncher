
import math

from vex import *

# Joystick coordinate range around 0 considered dead or inactive
JOYSTICK_DEAD_RANGE = 5

# Joystick coordinate range around 0 considered center
JOYSTICK_CENTER_RANGE = 20

# Movement points to add to movement values when rotating
ROTATE_MOVEMENT = 50

# Scale output to motor by that coefficient
MOTOR_SCALE = 0.8

# Brain should be defined by default
brain = Brain()

# Robot configuration code
controller = Controller(PRIMARY)

motor_direction = True

RED_CARTRIDGE = GearSetting.RATIO_36_1
GREEN_CARTRIDGE = GearSetting.RATIO_18_1
BLUE_CARTRIDGE = GearSetting.RATIO_6_1

# Motor class - use this to create an instance of a V5 smart motor
# Arguments:
#     port : The smartport this device is attached to
#     gears (optional) : The gear cartridge installed in the motor, default is the green 18_1
#     reverse (optional) : Should the motor's spin direction be reversed, default is False
#
# Returns:
#     A new Motor object.
#
# Examples:
#     motor1 = Motor(Ports.PORT1)
#     motor2 = Motor(Ports.PORT2, GearSetting.RATIO_36_1)
#     motor3 = Motor(Ports.PORT3, True)
#     motor4 = Motor(Ports.PORT4, GearSetting.RATIO_6_1, True)

motor_left_front = Motor(Ports.PORT11, BLUE_CARTRIDGE, motor_direction)
motor_left_back = Motor(Ports.PORT20, BLUE_CARTRIDGE, motor_direction)

motor_right_front = Motor(Ports.PORT1, BLUE_CARTRIDGE, not motor_direction)
motor_right_back = Motor(Ports.PORT10, BLUE_CARTRIDGE, not motor_direction)

beyblade = Motor(Ports.PORT17, GREEN_CARTRIDGE, True)


def zero_to_three_sixty(angle: float) -> float:
    """Clip angle to [0, 360) range.

    Inefficient for angles way out of the range, but these rarely occur.
    """
    while angle < 0:
        angle += 360

    while angle >= 360:
        angle -= 360

    return angle


def compute_joystick_angle(x: float, y: float) -> float:
    """Compute angle for the joystick, in degrees."""
    if x >= 0:
        return zero_to_three_sixty(math.degrees(math.atan2(y, x)))
    else:
        return 180 - math.degrees(math.atan2(y, -x))


def compute_joystick_distance(x: float, y: float) -> float:
    """Compute distance for the joystick."""
    return max(abs(x), abs(y))


def compute_movement_left_front(angle: float, distance: float) -> float:
    """Compute movement for left front and right back Mecanum wheels."""
    return math.sin(math.radians(angle + 45)) * distance


def compute_movement_right_front(angle: float, distance: float) -> float:
    """Compute movement for right front and left back Mecanum wheels."""
    return math.sin(math.radians(angle - 45)) * distance


def spin_motor(motor: Motor, movement: float):
    """Spin motor in the direction of movement sign with movement velocity."""
    if movement == 0:
        motor.stop()
    elif movement < 0:
        motor.set_velocity(min(-movement, 100) *
                           MOTOR_SCALE, VelocityUnits.PERCENT)
        motor.spin(REVERSE)
    else:
        motor.set_velocity(min(movement, 100) * MOTOR_SCALE,
                           VelocityUnits.PERCENT)
        motor.spin(FORWARD)


def controller_function():
    """Continuously running function translating controller inputs into movement."""
    global controller, motor_left_front, motor_left_back, motor_right_front, motor_right_back

    while True:
        # Get left joystick coordiantes
        left_joystick_x = controller.axis4.position()
        left_joystick_y = controller.axis3.position()

        # Get right joystick coordinates
        right_joystick_x = controller.axis1.position()
        right_joystick_y = controller.axis2.position()

        # Convert cartesian coordinates to polar for left joystick
        left_joystick_angle = compute_joystick_angle(
            left_joystick_x, left_joystick_y)
        left_joystick_distance = compute_joystick_distance(
            left_joystick_x, left_joystick_y)

        if left_joystick_distance < JOYSTICK_DEAD_RANGE:
            # Left joystick is in dead or inactive range
            movement_left_front = 0
            movement_right_front = 0
        else:
            # Left joystick is active
            movement_left_front = compute_movement_left_front(
                left_joystick_angle, left_joystick_distance)
            movement_right_front = compute_movement_right_front(
                left_joystick_angle, left_joystick_distance)

        if -JOYSTICK_CENTER_RANGE < right_joystick_x < JOYSTICK_CENTER_RANGE:
            # Right joystick is in the center range, horizontally
            rotate_left_front = 0
            rotate_left_back = 0
            rotate_right_front = 0
            rotate_right_back = 0
        elif right_joystick_x > 0:
            # Right joystick is tilted right
            if -JOYSTICK_CENTER_RANGE < right_joystick_y < JOYSTICK_CENTER_RANGE:
                # Right joystick is in the center range, vertically
                rotate_left_front = ROTATE_MOVEMENT
                rotate_left_back = ROTATE_MOVEMENT
                rotate_right_front = -ROTATE_MOVEMENT
                rotate_right_back = -ROTATE_MOVEMENT
            elif right_joystick_y > 0:
                # Right joystick is tilted forward
                rotate_left_front = ROTATE_MOVEMENT
                rotate_left_back = ROTATE_MOVEMENT
                rotate_right_front = 0
                rotate_right_back = 0
            else:
                # Right joystick is tilted backward
                rotate_left_front = ROTATE_MOVEMENT
                rotate_left_back = 0
                rotate_right_front = -ROTATE_MOVEMENT
                rotate_right_back = 0
        else:
            # Right joystick is tilted left
            if -JOYSTICK_CENTER_RANGE < right_joystick_y < JOYSTICK_CENTER_RANGE:
                # Right joystick is in the center range, vertically
                rotate_left_front = -ROTATE_MOVEMENT
                rotate_left_back = -ROTATE_MOVEMENT
                rotate_right_front = ROTATE_MOVEMENT
                rotate_right_back = ROTATE_MOVEMENT
            elif right_joystick_y > 0:
                # Right joystick is tilted forward
                rotate_left_front = 0
                rotate_left_back = 0
                rotate_right_front = ROTATE_MOVEMENT
                rotate_right_back = ROTATE_MOVEMENT
            else:
                # Right joystick is tilted backward
                rotate_left_front = 0
                rotate_left_back = -ROTATE_MOVEMENT
                rotate_right_front = 0
                rotate_right_back = ROTATE_MOVEMENT

        # Spin all four motors independently
        spin_motor(motor_left_front, movement_left_front + rotate_left_front)

        spin_motor(motor_left_back, movement_right_front + rotate_left_back)

        spin_motor(motor_right_front,
                   movement_right_front + rotate_right_front)

        spin_motor(motor_right_back, movement_left_front + rotate_right_back)

        if controller.buttonX.pressing():
            beyblade.set_velocity(100, VelocityUnits.PERCENT)
            beyblade.spin(FORWARD)
        elif controller.buttonA.pressing():
            beyblade.set_velocity(100, VelocityUnits.PERCENT)
            beyblade.spin(REVERSE)
        elif controller.buttonY.pressing():
            beyblade.set_velocity(1, VelocityUnits.PERCENT)
            beyblade.spin(FORWARD)
        elif controller.buttonB.pressing():
            beyblade.set_velocity(1, VelocityUnits.PERCENT)
            beyblade.spin(REVERSE)
        else:
            beyblade.stop()

        # Output some internal variables to the brain screen
        brain.screen.clear_screen()
        brain.screen.set_cursor(1, 1)
        brain.screen.print("left_joystick_x=%d" % left_joystick_x)
        brain.screen.next_row()
        brain.screen.print("left_joystick_y=%d" % left_joystick_y)
        brain.screen.next_row()
        brain.screen.print("left_joystick_angle=%f" % left_joystick_angle)
        brain.screen.next_row()
        brain.screen.print("left_joystick_distance=%f" %
                           left_joystick_distance)
        brain.screen.next_row()
        brain.screen.print("movement_left_front=%f" % movement_left_front)
        brain.screen.next_row()
        brain.screen.print("movement_right_front=%f" % movement_right_front)
        brain.screen.next_row()
        brain.screen.print("right_joystick_x=%d" % right_joystick_x)
        brain.screen.next_row()
        brain.screen.print("right_joystick_y=%d" % right_joystick_x)

        # wait before repeating the process
        wait(20, MSEC)


# Run controller function with a separate thread
controller_thread = Thread(controller_function)


def vexcode_auton_function():
    """Function for autonomous part of the competition."""

    # wait for the driver control period to end
    while (competition.is_autonomous() and competition.is_enabled()):
        # wait 10 milliseconds before checking again
        wait(10, MSEC)


def vexcode_driver_function():
    """Function for driver part of the competition."""

    # wait for the driver control period to end
    while (competition.is_driver_control() and competition.is_enabled()):
        # wait 10 milliseconds before checking again
        wait(10, MSEC)


# Register the competition functions
competition = Competition(vexcode_driver_function, vexcode_auton_function)
