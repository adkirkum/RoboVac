import GenConfig


class Motor:
    def __init__(self, max_speed, step_pin, dir_pin, motor_index):
        self.__max_speed = max_speed
        self.__step_pin = step_pin
        self.__dir_pin = dir_pin
        self.motor_index = motor_index
        # TODO: Set up GPIO direction (IN/OUT) here for pins above

    # TODO: Negate commands for the motor needs to be driven backwards here so we don't have to worry about it later
    # TODO: Probably want to add all motor sequencing into this class
    # TODO: Figure out micro-stepping and apply it to help when step_motor steps is not an integer

    @staticmethod
    def step_motor(steps, direction):
        # TODO: Finish method to actually step the motors
        dir_string = "forward" if direction == GenConfig.MotorDir.FORWARD else "reverse"
        print("Moving " + str(steps) + " in direction " + dir_string)
        pass
