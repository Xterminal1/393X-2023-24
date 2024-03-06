from vex import *
from math import e, pi

# robot configuration
brain = Brain()
controller = Controller(PRIMARY)

# servos
left_motor_1 = Motor(Ports.PORT20, GearSetting.RATIO_6_1, True)
left_motor_2 = Motor(Ports.PORT10, GearSetting.RATIO_6_1, True)
left_group = MotorGroup(left_motor_1, left_motor_2)

right_motor_1 = Motor(Ports.PORT13, GearSetting.RATIO_6_1, False)
right_motor_2 = Motor(Ports.PORT1, GearSetting.RATIO_6_1, False)
right_group = MotorGroup(right_motor_1, right_motor_2)

chassis = MotorGroup(left_motor_1, left_motor_2, right_motor_1, right_motor_2)

lift_motor_1 = Motor(Ports.PORT8, GearSetting.RATIO_36_1, True)
lift_motor_2 = Motor(Ports.PORT9, GearSetting.RATIO_36_1, False)
lift_group = MotorGroup(lift_motor_1, lift_motor_2)

flywheel_motor = Motor(Ports.PORT3, GearSetting.RATIO_6_1, False)
intake_motor = Motor(Ports.PORT18, GearSetting.RATIO_6_1, True)

# piston solenoids
left_wing = DigitalOut(brain.three_wire_port.b)
right_wing = DigitalOut(brain.three_wire_port.a)
ratchet = DigitalOut(brain.three_wire_port.c) # for lift

imu = Inertial(Ports.PORT19) # inertial measurement unit

def clear_screens():
    brain.screen.clear_screen()
    brain.screen.set_cursor(1, 1)
    controller.screen.clear_screen()
    controller.screen.set_cursor(1, 1)

def reset_positions():
    chassis.reset_position()
    lift_group.reset_position()
    flywheel_motor.reset_position()
    intake_motor.reset_position()

def set_stoppings():
    chassis.set_stopping(COAST)
    flywheel_motor.set_stopping(COAST)
    lift_group.set_stopping(HOLD)
    intake_motor.set_stopping(HOLD)

def intake(velocity):
    intake_motor.spin(FORWARD, velocity, VOLT)

def flywheel(velocity):
    flywheel_motor.spin(FORWARD, velocity, VOLT)

def lift(rotation):
    lift_group.spin_to_position(rotation, DEGREES, 100, PERCENT, False)

def wings(value):
    left_wing.set(value)
    right_wing.set(value)

# restrict input to range
def clamp(input, max, min):
    if input > max: 
        return max
    if input < min: 
        return min
    return input

def deadband(input, width):
    if abs(input) < width:
        return 0
    return input

def pre_auton():
    clear_screens()
    reset_positions()
    set_stoppings()
    imu.reset_heading()
    imu.reset_rotation()

wait(200)
print("\033[2J")

def recorder():
    while True:
        clear_screens()
        print()
        print("Time:", brain.timer.time(TimeUnits.SECONDS))
        print("Position:", chassis.position())
        print("Rotation:", imu.rotation())
        #print("Velocity:", to_volt(chassis.velocity(PERCENT)))
        print("Lift Position:", lift_group.position())
        #print("Flywheel Velocity:", to_volt(flywheel.velocity))
        print("Intake Temp:", intake_motor.temperature(TemperatureUnits.FAHRENHEIT))

        controller.screen.print("Drive", str(chassis.temperature(TemperatureUnits.FAHRENHEIT)), "F")
        controller.screen.next_row()
        controller.screen.print("Lift", str(lift_group.temperature(TemperatureUnits.FAHRENHEIT)), "F")
        controller.screen.next_row()
        controller.screen.print("Fwheel", str(flywheel_motor.temperature(TemperatureUnits.FAHRENHEIT)), "F")
        wait(10)

def drive(distance):
    reset_positions()

    # tuning params
    dt = 20
    kp, ki, kd = 0.015, 0.003, 1

    error = distance - chassis.position()
    integral, acceleration, prev_deriv, run_time = 0, 0, 0, 0
    prev_error = error

    # 0.5 gives most accurate error tolerance results
    tolerance = 0.5

    # can be changed, d/2 gives fastest results
    timeout = abs(distance) / 2

    while abs(error) > tolerance:
        # proportional
        error = distance - chassis.position()

        # integral
        integral += error * dt
        integral = clamp(integral, 5, -5) # restrict integral
        
        # derivative
        derivative = (error - prev_error) / dt
        if abs(derivative) > abs(acceleration + prev_deriv) and prev_deriv != 0:
            derivative = acceleration + prev_deriv
        
        # output
        output = kp * error + ki * integral + kd * derivative
        output = clamp(output, 12, -12)
        chassis.spin(FORWARD, output, VOLT)

        # timeout
        wait(dt)
        run_time += dt
        if run_time > timeout: # and timeout != 0:
            break

        acceleration = derivative - prev_deriv
        prev_deriv = derivative
        prev_error = error
    chassis.stop()

def turn(angle, timeout):

    # tuning params
    dt = 20
    kp, kd = 0.7, 0.03

    error = angle - imu.rotation()
    acceleration, prev_deriv, run_time = 0, 0, 0
    prev_error = error

    # 0.5 gives most accurate error tolerance results
    tolerance = 0.5

    while abs(error) > tolerance:
        # proportional
        error = angle - imu.rotation()
        
        # derivative
        derivative = (error - prev_error) / dt
        if abs(derivative) > abs(acceleration + prev_deriv) and prev_deriv != 0:
           derivative = acceleration + prev_deriv
        
        # output
        output = kp * error + kd * derivative
        left_group.spin(FORWARD, output, PERCENT)
        right_group.spin(REVERSE, output, PERCENT)
        
        # timeout
        wait(dt)
        run_time += dt
        if run_time > timeout: # and timeout != 0:
            break
        
        acceleration = derivative - prev_deriv
        prev_deriv = derivative
        prev_error = error
    chassis.stop()

def auto_defense():
    lift(45)
    intake(11)
    drive(500)
    lift(-30)

    drive(-2500)
    intake(0)
    turn(-30, 1000)
    left_wing.set(True)
    
    drive(-1000)
    wait(1000, MSEC)
    turn(-90, 1000)
    drive(-2000)

    left_wing.set(False)
    drive(1000)
    turn(90, 1000)
    intake(-11)
    wait(1000, MSEC)
    drive(1500)

    wait(1000, MSEC)
    intake(0)
    drive(-1000)

    turn(-55, 800)
    drive(-3000)
    wait(500, MSEC)

    drive(1000)
    wait(500, MSEC)
    drive(-3000)

    # RE DOWNLOAD CODE FOR CONSISTENT AUTON

def auto_offense():
    lift(45)
    drive(-3500)
    lift(-30)
    drive(900)
    turn(-25, 800)
    wings(True)
    drive(2500)
    turn(-200, 800)
    wings(False)
    turn(-220, 800)
    drive(-2500)

def auto_skills():
    # lift(45)
    # drive(-3500)
    # lift(-30)
    # drive(1500)
    # turn(-80, 800)
    # drive(-300)
    # flywheel(10)
    # wait(1000, MSEC) # change to different ime
    # flywheel(0)
    # turn(-25, 800)
    # drive(800)
    # turn(-52, 800)
    # drive(3000)
    lift(45)
    turn(-30, 800)
    lift(-30)
    drive(-500)
    
    flywheel(10)
    wait(1000, MSEC)
    flywheel(0)

    drive(200)
    turn(3, 800)
    drive(6000)

    turn(120, 800)
    drive(-2500)

    wait(500, MSEC)
    drive(300)
    turn(180, 800)
    drive(3000)
    
    turn(270, 800)
    drive(2500)
    turn(180, 800)

    wings(True)
    drive(-2000)

def autonomous():
    clear_screens()
    reset_positions()
    set_stoppings()
    # imu.calibrate()
    # while imu.is_calibrating():
    #     sleep(50)
    Thread(recorder)
    #auto_defense_2()
    #auto_OS()
    auto_skills()
 
# def curve(input, scale):
#     return (pow(2.718, -(scale/10)) + pow(2.718, abs(input) - 127/10) * (1 - pow(2.718, -(scale/10)))) * input

def control_arcade():
    while True:
        throttle = deadband(controller.axis2.value() * 0.12, 5)
        turn = deadband(controller.axis4.value() * 0.12, 5)
        left_group.spin(FORWARD, throttle + turn, VOLT)
        right_group.spin(FORWARD, throttle - turn, VOLT)
        wait(5)

def driver_control():
    clear_screens()
    set_stoppings()
    Thread(recorder)
    Thread(control_arcade)

active = False; # toggle boolean

def intake_button():
    global active
    active = not active
    if active:
        intake(11) # fwd spin
    else:
        intake(0) # intake stop

def outtake_button():
    global active
    active = not active
    if active:
        intake(-11) # rev spin
    else:
        intake(0) # intake stop

def flywheel_button():
    global active
    active = not active
    if active:
        flywheel(11)
    else:
        flywheel(0)

def wings_button():
    global active
    active = not active
    if active:
        wings(True)
    else:
        wings(False)

def lift_button():
    global active
    active = not active
    if active:
        lift(180)
    else:
        lift(0)

def anti_tip_button():
    global active
    active = not active
    if active:
        lift(500)
    else:
        lift(0)

# controller callback functions
controller.buttonL1.pressed(intake_button) 
controller.buttonL2.pressed(outtake_button)
controller.buttonR1.pressed(flywheel_button)
controller.buttonR2.pressed(wings_button)
controller.buttonY.pressed(lift_button) 
controller.buttonX.pressed(anti_tip_button)

# executing competition functions
competition = Competition(driver_control, autonomous)
pre_auton()
