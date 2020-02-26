import RPi.GPIO as GPIO
import time
import serial

 GPIO.setmode(GPIO.BOARD)
ser = serial.Serial("/dev/ttyACM0", 9600)

 GPIO.setup(5,GPIO.OUT) #pin 5 for direction
GPIO.setup(20,GPIO.OUT) #pin 20 for motor 1 pwm
GPIO.setup(21,GPIO.OUT) #pin 21 for motor 2 pwm

 motor1=GPIO.PWM(20,100)
motor2=GPIO.PWM(21,100)
motor1.start(0)
motor2.start(0)

 val = 0

 while ser.in_waiting:
    ser.readline()

 while True:

     while ser.in_waiting:
        cc=str(ser.readline())
	val = int(cc)
        print val

     led.value = val/1023.00	 

 
