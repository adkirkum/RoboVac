# Libraries
import time
import GenConfig
import ControlDriveSteppers
from Motor import Motor
import DCMotor

import RPi.GPIO as GPIO

# GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)

GPIO_TRIGGER = 23
GPIO_ECHO = 18

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

    GPIO.setup(22, GPIO.OUT) #MS1
    # GPIO.output(3, False) #
    GPIO.output(22, True) #MS1 State - Set to true to half step, false to single step

    # m_DC.turn_motor(10,75,1)

    # driver.align_to_wall(160,10,2)

    # driver.step_motor_at_speed(10)

    k=0
    while True:

        driver.turn_and_drive(90, 2,50, "str",40,40,2,0.00005)
        driver.turn_and_drive(180, 2,140, "ccw",10,10,2,0.00005)
        driver.turn_and_drive(90, 2,50, "str",40,40,2,0.00005)
        driver.turn_and_drive(180, 2,140, "ccw",10,10,2,0.00005)
        # dist = distance()
        # print ("Measured Distance = %.1f cm" % dist)

        m = False #Motor stepped status
        desired_steps = 250
        num_steps_taken = 0

    #Basic WASD control
        # key = raw_input("direction:\n")
        # top_spd_str = 20
        # top_spd_str_back = 30

        # if key == "w":
        #     driver.turn_and_drive(720, 20,140, "str",1,top_spd_str,1,0.00001)
        #     driver.turn_and_drive(720, 20,140, "str",top_spd_str,1,1,0.00001)
        # elif key == "a":
        #     driver.turn_and_drive(20, 20,600, "ccw",4,15,1,0.00001)
        #     driver.turn_and_drive(20, 20,600, "ccw",15,4,1,0.00001)
        # elif key == "s":
        #     driver.turn_and_drive(720, 20,140, "str",-1,-top_spd_str_back,1,0.00001)
        #     driver.turn_and_drive(720, 20,140, "str",-top_spd_str_back,-1,1,0.00001)
        # elif key == "d":
        #     driver.turn_and_drive(20, 20,600, "cw",4,15,1,0.00001)
        #     driver.turn_and_drive(20, 20,600, "cw",15,4,1,0.00001)
        # m_DC.turn_motor(0.75,100,1)
        #BUG: Cannot set radius to 12.1 (width of robot)
        # driver.turn_and_drive(720, 20,600, "str",1,100,1,0.00001)

        # driver.turn_and_drive(180, 12.2,100, "cw",20,20,1,0.00001)
        # driver.turn_and_drive(180, 12.2,100, "str",20,5,1,0.000005)
        # driver.turn_and_drive(180, 12.1,100, "straight",1,7,1,0.00001)
        # print("turning")
        k=k+1

    # m_DC.stop_motor()
    GPIO.cleanup()
