#Libraries
import RPi.GPIO as GPIO
import time

#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)

#set GPIO Pins
GPIO_TRIGGER = 26
GPIO_ECHO = 20
GPIO_STEP_R = 23
GPIO_DIR_R = 22
GPIO_STEP_L = 13
GPIO_DIR_L = 19

#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)
GPIO.setup(GPIO_STEP_R, GPIO.OUT)
GPIO.setup(GPIO_DIR_R, GPIO.OUT)
GPIO.setup(GPIO_STEP_L, GPIO.OUT)
GPIO.setup(GPIO_DIR_L, GPIO.OUT)
next_step_time_L = time.time()
next_step_time_R = time.time()

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

def long_map(x, in_min, in_max, out_min, out_max):
    long_map = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    #print long_map
    return long_map

def constrain(val, min_val, max_val):
    if val < min_val: return min_val
    if val > max_val: return max_val
    return val

def speed_calc(spd): #takes input rpm and converts to step delay in seconds
    speed_calc = (1.8*(1/(spd/60))/360)
    return speed_calc

def direction_calc(move_dist, pin):
    if move_dist>0:
        GPIO.output(pin, True)
    else:
        GPIO.output(pin, False)
    return True

# def one_step(wait_time,step_pin,dir_pin,move_dist):
#     direction_calc(move_dist, dir_pin)
#     GPIO.output(step_pin,True)
#     time.sleep(wait_time)
#     GPIO.output(step_pin,False)
#     time.sleep(wait_time)

# def step(lin_dist,dir,next_step_time_L,next_step_time_R,bound_L, bound_R): #linear dist in steps,direction: (-180 to 180) speed in rpm
#     lin_dist = 8
#     strt_time = time.time()
#
#     max_rpm = 300
#     min_rpm = 100.0
#     step_limit = 10
#     dia = 20
#     circumf = 3.1415926*dia
#     steps_deg = circumf/360 #steps needed per degree of circle
#     print steps_deg
#     dist_R = dir*steps_deg+lin_dist
#     dist_L = -dir*steps_deg+lin_dist
#
#
#     sleep_time_L = long_map(abs(dist_R),bound_L,bound_U,speed_calc(min_rpm),speed_calc(max_rpm))
#     speed_L = long_map(abs(dist_L),bound_L,bound_U,min_rpm,max_rpm)
#     sleep_time_R = long_map(abs(dist_L),bound_L,bound_U,speed_calc(min_rpm),speed_calc(max_rpm))
#     speed_R = long_map(abs(dist_R),bound_L,bound_U,min_rpm,max_rpm)
#     print("Delay: " +str(sleep_time_L)+"  "+str(sleep_time_R) )
#     print("Rpm: " +str(speed_L)+"  "+str(speed_R) )
#     print("Dist: " +str(dist_L)+"  "+str(dist_R) )
#     print("Time: " +str(strt_time)+"  "+str(next_step_time_L) )
#     print( " ")
#     #print strt_time
#     if (next_step_time_L - strt_time) < 0:
#         next_step_time_L = strt_time + sleep_time_L
#         direction_calc(-dist_L,GPIO_DIR_L)
#         GPIO.output(GPIO_STEP_L,True)
#         time.sleep(0.00)
#         GPIO.output(GPIO_STEP_L,False)
#
#     if (next_step_time_R - strt_time) < 0:
#         next_step_time_R = strt_time + sleep_time_R
#         direction_calc(dist_R,GPIO_DIR_R)
#         GPIO.output(GPIO_STEP_R,True)
#         time.sleep(0.00)
#         GPIO.output(GPIO_STEP_R,False)
    # for i in range(constrain(abs(dist_R),0,step_limit)):
    #     one_step(speed_calc(80.0), GPIO_STEP_R, GPIO_DIR_R, dist_R)
    #     print "n"
    # for i in range(constrain(abs(dist_L),0,step_limit)):
    #     #one_step(speed_calc(80.0), GPIO_STEP_L, GPIO_DIR_L, -dist_L)
    #     print "n"
    #print range(abs(dist_R))
    # for i in range(constrain(abs(dist_R),0,step_limit)):
    #     direction_calc(dist_R,GPIO_DIR_R)
    #     GPIO.output(GPIO_STEP_R,True)
    #
    #     time.sleep(long_map(abs(dist_R),8,35,speed_calc(min_rpm),speed_calc(max_rpm)))
    #     #time.sleep(0.005)
    #     GPIO.output(GPIO_STEP_R,False)
    # for i in range(constrain(abs(dist_L),0,step_limit)):
    #     direction_calc(dist_L,GPIO_DIR_L)
    #     GPIO.output(GPIO_STEP_L,True)
    #     time.sleep(long_map(abs(dist_L),8,35,speed_calc(min_rpm),speed_calc(max_rpm)))
    #     #time.sleep(0.005)
    #     GPIO.output(GPIO_STEP_L,False)
    step = True
    return step

if __name__ == '__main__':

    try:

        while True:
            dist = distance()
            # if dist>150:
                # dist = 150
            print ("Measured Distance = %.1f cm" % dist)
            sens = 15
            dir = constrain(long_map(dist,10,20,-sens,sens),-sens, sens)
            bound_L = 5
            bound_U = 10

            #step(8,dir,next_step_time_L,next_step_time_R,bound_L,bound_U)

            lin_dist = 8
            strt_time = time.time()

            min_rpm = 100
            max_rpm = 200

            dia = 20
            circumf = 3.1415926*dia
            steps_deg = circumf/360 #steps needed per degree of circle
            print steps_deg
            dist_L = dir*steps_deg+lin_dist
            dist_R = -dir*steps_deg+lin_dist

            sleep_time_L = long_map(abs(dist_R),bound_L,bound_U,speed_calc(min_rpm),speed_calc(max_rpm))
            speed_L = long_map(abs(dist_L),bound_L,bound_U,min_rpm,max_rpm)
            sleep_time_R = long_map(abs(dist_L),bound_L,bound_U,speed_calc(min_rpm),speed_calc(max_rpm))
            speed_R = long_map(abs(dist_R),bound_L,bound_U,min_rpm,max_rpm)
            print("Delay: " +str(sleep_time_L)+"  "+str(sleep_time_R) )
            print("Rpm: " +str(speed_L)+"  "+str(speed_R) )
            print("Dist: " +str(dist_L)+"  "+str(dist_R) )
            print("Time: " +str(strt_time)+"  "+str(next_step_time_L))

            #print strt_time
'''
            if (next_step_time_L - strt_time) < 0:
                next_step_time_L = strt_time + sleep_time_L
                direction_calc(-dist_L,GPIO_DIR_L)
                GPIO.output(GPIO_STEP_L,True)
                time.sleep(0.000001)
                GPIO.output(GPIO_STEP_L,False)

                next_step_time = time.time()
                while True:
                    next_step_time, stepped = blah_motor.step(time.time(), next_motor_step, direction, speed)

            if (next_step_time_R - strt_time) < 0:
                next_step_time_R = strt_time + sleep_time_R
                direction_calc(dist_R,GPIO_DIR_R)
                GPIO.output(GPIO_STEP_R,True)
                time.sleep(0.000001)
                GPIO.output(GPIO_STEP_R,False)
'''

            print( " ")
            time.sleep(0.001)

        # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup()
