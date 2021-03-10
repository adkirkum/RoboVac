from GenConfig import RotateDir
from GenConfig import MotorDir
import Motor


class ControlDriverSteppers:
    def __init__(self):
        self.__motor_l = Motor.Motor(20, 13, 19, 0)
        self.__motor_r = Motor.Motor(20, 23, 22, 1)

    # def turn_then_drive(self, distance, degrees, direction: RotateDir):
    #     """
    #     :integer distance: distance to move in steps
    #     :integer degrees: degrees change from current heading
    #     :RotateDir direction: disambiguate which way to rotate
    #     """
    #     steps_per_full_rot = 200    # TODO: This value is probably not correct... just a placeholder
    #     steps_for_turn = (steps_per_full_rot * (degrees / 360)) / 2   # Divide by 2 because both motors are turning
    #
    #     # Do all rotation in place before moving forward
    #     if direction == RotateDir.CLOCKWISE:
    #         self.__step_motors(steps_for_turn, -steps_for_turn)
    #     else:
    #         self.__step_motors(-steps_for_turn, steps_for_turn)
    #
    #     # Now drive forward
    #     self.__step_motors(distance, distance)
    #     pass

    def turn_and_drive(self, degrees, radius):
        # TODO: Finish this method
        pass

    @staticmethod
    def __calc_dir(steps):
        return MotorDir.FORWARD if steps >= 0 else MotorDir.REVERSE

    def __step_motors(self, steps_l, steps_r):
        max_steps_per_loop = 6
        max_steps = max(abs(steps_l), abs(steps_r))
        primary_motor = self.__motor_l if max_steps == abs(steps_l) else self.__motor_r
        primary_steps = steps_l if primary_motor == self.__motor_l else steps_r
        secondary_motor = self.__motor_l if primary_motor == self.__motor_r else self.__motor_r
        secondary_steps = steps_l if secondary_motor == self.__motor_l else steps_r
        steps_ratio = 1 - ((abs(primary_steps) - abs(secondary_steps)) / max_steps)

        while max_steps > 0:
            loop_steps = max_steps_per_loop if max_steps >= max_steps_per_loop else max_steps
            primary_motor.step_motor(loop_steps, self.__calc_dir(primary_steps))
            secondary_motor.step_motor(loop_steps * steps_ratio, self.__calc_dir(secondary_steps))
            max_steps -= loop_steps
