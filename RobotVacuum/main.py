# Libraries

import time
import ControlDriveSteppers


import RPi.GPIO as GPIO
# GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)

# set GPIO Pins
GPIO_TRIGGER = 26
GPIO_ECHO = 20
GPIO_STEP_R = 23
GPIO_DIR_R = 22
GPIO_STEP_L = 13
GPIO_DIR_L = 19

# set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)
GPIO.setup(GPIO_STEP_R, GPIO.OUT)
GPIO.setup(GPIO_DIR_R, GPIO.OUT)
GPIO.setup(GPIO_STEP_L, GPIO.OUT)
GPIO.setup(GPIO_DIR_L, GPIO.OUT)

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
if __name__ == '__main__':
    driver = ControlDriveSteppers.ControlDriverSteppers()
    driver.turn_then_drive(100, 180, ControlDriveSteppers.RotateDir.CLOCKWISE)
