# Libraries
import time
import GenConfig
import ControlDriveSteppers
from Motor import Motor
import DCMotor

import RPi.GPIO as GPIO

# GPIO Mode (BOARD / BCM)

GPIO.setmode(GPIO.BCM)
# set GPIO Pins
# GPIO_TRIGGER = 26
# GPIO_ECHO = 20
# GPIO_STEP_R = 23
# GPIO_DIR_R = 22
# GPIO_STEP_L = 13
# GPIO_DIR_L = 19

# set GPIO direction (IN / OUT)
# GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
# GPIO.setup(GPIO_ECHO, GPIO.IN)
# GPIO.setup(GPIO_STEP_R, GPIO.OUT)
# GPIO.setup(GPIO_DIR_R, GPIO.OUT)
# GPIO.setup(GPIO_STEP_L, GPIO.OUT)
# GPIO.setup(GPIO_DIR_L, GPIO.OUT)

'''
def distance():
    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)

    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)

    start_time = time.time()
    stop_time = time.time()

    # save start_time
    while GPIO.input(GPIO_ECHO) == 0:
        start_time = time.time()

    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        stop_time = time.time()

    # time difference between start and arrival
    time_elapsed = stop_time - start_time
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    return (time_elapsed * 34300) / 2


def long_map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def constrain(val, min_val, max_val):
    return min(max(val, min_val), max_val)


# takes input rpm and converts to step delay in seconds
def speed_calc(spd):
    return 1.8 * (1 / (spd / 60.0)) / 360


def direction_calc(move_dist, pin):
    if move_dist > 0:
        GPIO.output(pin, True)
    else:
        GPIO.output(pin, False)
    return True


# linear dist in steps,direction: (-180 to 180) speed in rpm
def step(lin_dist, direction, spd_r, spd_l):
    # max_rpm = 200
    # min_rpm = 90
    step_limit = 15
    dia = 20
    circumference = 3.1415926 * dia
    steps_deg = circumference / 360  # steps needed per degree of circle

    dist_l = constrain(int(round(-direction * steps_deg, 1) + lin_dist), -step_limit, step_limit)
    dist_r = -constrain(int(round(direction * steps_deg, 1) + lin_dist), -step_limit, step_limit)
    print(str(dist_l) + "  " + str(-dist_r))
    # print range(abs(dist_r))
    if dist_l == 0:
        dist_l = 1
    if dist_r == 0:
        dist_r = 1
    if abs(dist_r) > abs(dist_l):
        step_time_r = speed_calc(120)
        step_time_l = abs(step_time_r*(dist_l/dist_r))
    elif abs(dist_r) < abs(dist_l):
        step_time_r = speed_calc(120)
        step_time_l = abs(step_time_r*(dist_l/dist_r))
    else:
        step_time_r = speed_calc(120)
        step_time_l = step_time_r*(dist_l/dist_r)
    # for i in range(constrain(abs(num),0,step_limit)):
    step_time_l_i = step_time_l
    step_time_r_i = 0
    print(str(step_time_l) + "  " + str(step_time_r))
    if dist_r > dist_l:
        for i in range(20):
            step_time_r_i = step_time_r_i + step_time_r
            direction_calc(dist_r, GPIO_DIR_R)
            GPIO.output(GPIO_STEP_R, True)
            time.sleep(step_time_r)
            # time.sleep(long_map(abs(dist_l),35,55,speed_calc(min_rpm),speed_calc(max_rpm)))
            GPIO.output(GPIO_STEP_R, False)
            if step_time_l_i >= step_time_r_i:
                step_time_l_i = step_time_l_i+step_time_l
                direction_calc(dist_l, GPIO_DIR_L)
                GPIO.output(GPIO_STEP_L, True)
                time.sleep(step_time_l)
                # time.sleep(long_map(abs(dist_r),35,55,speed_calc(min_rpm),speed_calc(max_rpm)))
                GPIO.output(GPIO_STEP_L, False)
    else:
        for i in range(20):
            step_time_l_i = step_time_l_i+step_time_l
            direction_calc(dist_l, GPIO_DIR_L)
            GPIO.output(GPIO_STEP_L, True)
            time.sleep(step_time_l)
            # time.sleep(long_map(abs(dist_r),35,55,speed_calc(min_rpm),speed_calc(max_rpm)))
            GPIO.output(GPIO_STEP_L, False)
            if step_time_r_i >= step_time_l_i:
                step_time_r_i = step_time_r_i+step_time_r
                direction_calc(dist_r, GPIO_DIR_R)
                GPIO.output(GPIO_STEP_R, True)
                time.sleep(step_time_r)
                # time.sleep(long_map(abs(dist_l),35,55,speed_calc(min_rpm),speed_calc(max_rpm)))
                GPIO.output(GPIO_STEP_R, False)
    return True


if __name__ == '__main__':
    try:
        while True:
            dist = distance()
            print("Measured Distance = %.1f cm" % dist)
            sens = 20
            motor_dir = constrain(long_map(dist, 15, 25, -sens, sens), -sens, sens)
            adj = 4
            step(3*adj, motor_dir * adj, 300, 300)

            time.sleep(0.001)

        # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup()
'''
GPIO_TRIGGER = 23
GPIO_ECHO = 18
# brush_motor = l293d.DC(13,5,26, force_selection=True)
# pwm = l293d.PWM(freq=30, cycle=70)


def distance():

    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)

    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)

    StartTime = time.time()
    StopTime = time.time()

    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()

    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()

    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2

    return distance

if __name__ == '__main__':
    driver = ControlDriveSteppers.ControlDriverSteppers()
    #
    m_r = Motor(20, 27, 17, 1)
    m_l = Motor(20, 9, 10, 0)
    m_DC = DCMotor.DCMotor(13,5,26)
    # GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
    # GPIO.setup(GPIO_ECHO, GPIO.IN)

    #turn_and_drive(degrees, radius in cm, distance, left_right_straight,speed_LOW,speed_HIGH,microstep_divider):
    # GPIO.setup(3, GPIO.OUT) #MS


    GPIO.setup(22, GPIO.OUT) #MS1
    # GPIO.output(3, False) #
    GPIO.output(22, True) #MS1 State
    # m_DC.turn_motor(10,75,1)
    # driver.turn_and_drive(180, 10.5,400, "straight",10,30,2,0.00005)

    #driver.turn_and_drive(180, 20,400, "left",8,10,2,0.00005)
    #driver.turn_and_drive(180, 10,100, "straight",25,45,4)
    #driver.turn_and_drive(180, 15,20, "right",6.6,10,4)
    k=0
    while True:

        # dist = distance()
        # print ("Measured Distance = %.1f cm" % dist)
        # # next_step_time_r = time.time()
        # # next_step_time_l = time.time()
        m = False #Motor stepped status
        desired_steps = 250
        num_steps_taken = 0
        key = raw_input("direction:\n")
        top_spd_str = 20
        top_spd_str_back = 30
        if key == "w":
            driver.turn_and_drive(720, 20,140, "str",1,top_spd_str,1,0.00001)
            driver.turn_and_drive(720, 20,140, "str",top_spd_str,1,1,0.00001)
        elif key == "a":
            driver.turn_and_drive(20, 20,600, "ccw",4,15,1,0.00001)
            driver.turn_and_drive(20, 20,600, "ccw",15,4,1,0.00001)
        elif key == "s":
            driver.turn_and_drive(720, 20,140, "str",-1,-top_spd_str_back,1,0.00001)
            driver.turn_and_drive(720, 20,140, "str",-top_spd_str_back,-1,1,0.00001)
        elif key == "d":
            driver.turn_and_drive(20, 20,600, "cw",4,15,1,0.00001)
            driver.turn_and_drive(20, 20,600, "cw",15,4,1,0.00001)
        # m_DC.turn_motor(0.75,100,1)
        #BUG: Cannot set radius to 12.1 (width of robot)
        # driver.turn_and_drive(720, 20,600, "str",1,100,1,0.00001)

        # driver.turn_and_drive(180, 12.2,100, "cw",20,20,1,0.00001)
        print "Hello!"
        # driver.turn_and_drive(180, 12.2,100, "str",20,5,1,0.000005)
        # driver.turn_and_drive(180, 12.1,100, "straight",1,7,1,0.00001)
        # print("turning")
        k=k+1

        #while num_steps_taken < desired_steps:
        #while True:

            # next_step_Time_r, stepped_r = m_r.step_motor(time.time(), next_step_time_r, GenConfig.MotorDir.FORWARD, 20)
            # next_step_Time_l, stepped_l = m_l.step_motor(time.time(), next_step_time_l, GenConfig.MotorDir.REVERSE, 20)


        # driver.turn_and_drive(10, 800, "left", 10,15)

        # for i in range(1000):
            # time.sleep(0.01)

        #
        #driver.turn_and_drive(20, 30, "right",10,15)

        print k
        # #
        # driver.turn_and_drive(20, 30, "right")
        #
        # driver.turn_and_drive(20, 30, "left")


            #print m_l.steps_taken

        #time.sleep(0.1)


        #time.sleep(0.001)
    m_DC.stop_motor()
    GPIO.cleanup()
