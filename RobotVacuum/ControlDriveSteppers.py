
import time
from GenConfig import RotateDir
from GenConfig import MotorDir
import Motor

#from Motor import Motor

class ControlDriverSteppers:
    def __init__(self):
        self.__motor_l = Motor.Motor(20, 27, 17, 0)
        self.__motor_r = Motor.Motor(20, 9, 10, 1)

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

    def turn_and_drive(self, degrees, radius, distance, left_right_straight,speed_LOW,speed_HIGH,microstep_divider,accel):


        w = 12.1*2 #width of wheelbase in cm
        wheel_DIA = 6.7 #wheel diameter in cm
         #Turn radius from center of robot

        if left_right_straight == "ccw":
            Rl = radius - w / 2.0 #calculated radius through left wheel
            Rr = radius + w / 2.0 #calculated radius through right wheel
        elif left_right_straight == "cw":
            #print "right"
            Rl = radius + w / 2.0 #calculated radius through left wheel (cm)
            Rr = radius - w / 2.0 #calculated radius through right wheel (cm)



        step_dist = (3.14159265*wheel_DIA)/(200*microstep_divider) #one step move dist (cm/step)
        if left_right_straight == "str":
            dist_l = distance/step_dist
            dist_r = distance/step_dist
            speed_R = speed_LOW
            speed_ratio_r = 1
            speed_ratio_l = 1
            speed_L = speed_R
        else:
            dist_l = (( 2*3.14159265 * Rl*degrees)/360) /(step_dist) #arc length for specified number of degrees/step_dist
            dist_r = (( 2*3.14159265 * Rr*degrees)/360) /(step_dist)
            speed_R = speed_LOW*(Rr/radius)
            speed_L = speed_LOW*(Rl/radius)
            speed_ratio_r= Rr/radius
            speed_ratio_l= Rl/radius

        done_l = False #Indicators for when the motors are done stepping.
        done_r = False
        #print(str(dist_l))
        initial_steps_taken_r = self.__motor_r.steps_taken #steps taken at beginning of loop, point in time
        initial_steps_taken_l = self.__motor_l.steps_taken
        k_mult = speed_LOW / speed_HIGH #ratio of low to high speed (right motor)
        n_mult = speed_LOW / speed_HIGH #ratio of low to high speed (left motor)

        while done_l == False and done_r == False:
            #print n


            #print(str(k_mult) + "  " + str(time.time()) +"  "+ str(self.__motor_r.next_step_tm))
        #right wheel speed in rpm

            if self.__motor_r.steps_taken - initial_steps_taken_r < dist_r: #if under desired number of steps
                m = self.__motor_r.step_motor(time.time(),self.__motor_r.next_step_tm, speed_R)
                if k_mult<1:
                    k_mult=k_mult+accel
                elif k_mult>1:
                    k_mult=k_mult-accel
                else:
                    k_mult=1

                speed_R = speed_HIGH*(speed_ratio_r)*(k_mult)

            else:
                done_r = True
            #print(str(dist_l-self.__motor_l.steps_taken))
            if self.__motor_l.steps_taken - initial_steps_taken_l < dist_l:
                m = self.__motor_l.step_motor(time.time(),self.__motor_l.next_step_tm, speed_L)
                if n_mult<1:
                    n_mult=n_mult+accel
                elif n_mult>1:
                    n_mult=n_mult-accel
                else:
                    n_mult=1
                speed_L = speed_HIGH*(speed_ratio_l)*(n_mult) #left wheel speed in rpm

            else:
                done_l = True
        print(str(speed_L)+" "+str(speed_R) +" desired:" +str(dist_l) + " initial steps:" + str(initial_steps_taken_l)+ " current_steps:" +str(self.__motor_l.steps_taken))
        #pass

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
