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

        EN1 = GPIO.PWM(self.en_pin, frequency)
        # print("duty cycle set to 100")
        #EN2 = GPIO.PWM(Motor2['EN'], 100)

        EN1.start(duty_cycle)

        # print (x)
        EN1.ChangeDutyCycle(duty_cycle) #start at 15
        print("duty:" + str(duty_cycle)+ " freq: "+str(frequency)+" en_pin: "+str(self.en_pin))
        EN1.ChangeFrequency(frequency) #start at 20
        # print("frequency set to "+ str(frequency))

    def stop_motor(self):
        # EN1 = GPIO.PWM(self.en_pin, 100)
        # EN1.ChangeDutyCycle(0)
        # EN1.ChangeFrequency(frequency)
        GPIO.output(self.input1_pin, GPIO.LOW)
        GPIO.output(self.input2_pin, GPIO.LOW)

# GPIO.cleanup()
