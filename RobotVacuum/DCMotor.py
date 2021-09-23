import GenConfig
import time
import RPi.GPIO as GPIO


class DCMotor:
    def __init__(self, en_pin, input1_pin, input2_pin):
        self.en_pin = en_pin
        self.input1_pin = input1_pin
        self.input2_pin = input2_pin
        GPIO.setup(en_pin, GPIO.OUT)
        GPIO.setup(input1_pin, GPIO.OUT)
        GPIO.setup(input2_pin, GPIO.OUT)

    # @staticmethod
    def turn_motor(self, duty_cycle, frequency, direction):

        if direction >= 0:
            GPIO.output(self.input1_pin, GPIO.LOW)
            GPIO.output(self.input2_pin, GPIO.HIGH)
        else:
            GPIO.output(self.input1_pin, GPIO.HIGH)
            GPIO.output(self.input2_pin, GPIO.LOW)

        # GPIO.setmode(GPIO.BCM)
        # GPIO.setwarnings(False)

        # Motor1 = {'EN': 13, 'input1': 5, 'input2': 26}

        EN1 = GPIO.PWM(self.en_pin, frequency)
        # print("duty cycle set to 100")
        #EN2 = GPIO.PWM(Motor2['EN'], 100)

        EN1.start(duty_cycle)
        #EN2.start(0)


        # print (x)
        EN1.ChangeDutyCycle(duty_cycle) #start at 15
        print("duty:" + str(duty_cycle)+ " freq: "+str(frequency)+" en_pin: "+str(self.en_pin))
        EN1.ChangeFrequency(frequency) #start at 20
        # print("frequency set to "+ str(frequency))

    def stop_motor(self):
        EN1 = GPIO.PWM(self.en_pin, 100)
        EN1.ChangeDutyCycle(0)
        EN1.ChangeFrequency(frequency)
        GPIO.output(self.input1_pin, GPIO.LOW)
        GPIO.output(self.input2_pin, GPIO.LOW)

# sleep(5)

    # for x in range(40, 100):
    #     print ("BACKWARD MOTION")
    #     EN1.ChangeDutyCycle(x)
    #     # EN1.ChangeFrequency()
    # #    EN2.ChangeDutyCycle(x)
    #
    #     GPIO.output(Motor1['input1'], GPIO.LOW)
    #     GPIO.output(Motor1['input2'], GPIO.HIGH)
    #
    # #    GPIO.output(Motor2['input1'], GPIO.LOW)
    # #    GPIO.output(Motor2['input2'], GPIO.HIGH)
    #
    #     sleep(0.1)
    #
    # print ("STOP")
    # EN1.ChangeDutyCycle(0)
    # #EN2.ChangeDutyCycle(0)
    #
    # sleep(5)

# GPIO.cleanup()
