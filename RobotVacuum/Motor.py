import GenConfig
import time
import RPi.GPIO as GPIO


class Motor:
    def __init__(self, max_speed, step_pin, dir_pin, motor_index):
        self.__max_speed = max_speed
        self.motor_index = motor_index
        self.step_pin = step_pin
        self.dir_pin = dir_pin
        GPIO.setup(step_pin, GPIO.OUT)
        GPIO.setup(dir_pin, GPIO.OUT)
        # TODO: Set up GPIO direction (IN/OUT) here for pins above

    # takes input rpm and converts to step delay in seconds
    @staticmethod
    def speed_calc(speed):
        return 1.8 * (1 / (speed / 60.0)) / 360

    # TODO: Negate commands for the motor needs to be driven backwards here so we don't have to worry about it later
    # TODO: Probably want to add all motor sequencing into this class
    # TODO: Figure out micro-stepping and apply it to help when step_motor steps is not an integer
        # Step motor steps will probably always be an integer, unless we dynamically adjust microstep setting
        # on controller (we totally could).  So 14.625 steps would be
        # - Step 14 times
        # - Switch to 1/8th step setting, then step 5 times

    def motor_polarity(self, direction):
        return direction if self.motor_index == 0 else not direction

    def step_motor(self, cur_time, next_step_time, direction, speed):
        # TODO: Finish method to actually step the motors
        step_delay = self.speed_calc(speed)
        # dir_string = "forward" if direction == GenConfig.MotorDir.FORWARD else "reverse"
        # print("Moving " + str(steps) + " in direction " + dir_string)

        if cur_time >= next_step_time:
            GPIO.output(self.dir_pin, self.motor_polarity(True if direction == GenConfig.MotorDir.FORWARD else False))
            GPIO.output(self.step_pin, True)
            time.sleep(0.000001)
            GPIO.output(self.step_pin, False)
            return [next_step_time + step_delay, True]
        return next_step_time, False

    def step_motor_dist(self, dist, direction, speed):
        self.step_motor()