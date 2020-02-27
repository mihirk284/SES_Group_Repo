
import RPi.GPIO as GPIO
import time
import serial

 GPIO.setmode(GPIO.BOARD)
ser = serial.Serial("/dev/ttyACM0", 9600)

GPIO.setup(20,GPIO.OUT) #pin 20 for motor 1 pwm
GPIO.setup(21,GPIO.OUT) #pin 21 for motor 2 pwm

motor1=GPIO.PWM(20,100)
motor2=GPIO.PWM(21,100)
motor1.start(0)
motor2.start(0)

prevval = 0
val = 0
rot_ctr = 0

time.sleep(1)
while ser.in_waiting:
    ser.readline()
begin_time = time.time()

while True:
   passed_time - time.time() - begin_time
   if(passed_time >= 6):
      break
   
   dutyCycle = 100 * (6 - passed_time) / 6
   motor1.ChangeDutyCycle(dutyCycle)
   motor2.ChangeDutyCycle(dutyCycle)
   while ser.in_waiting:
      cc=str(ser.readline())
      val = int(cc)
   
   if(prevval - val > 1000):
      rot_ctr +=1


motor1.ChangeDutyCycle(dutyCycle)
motor2.ChangeDutyCycle(dutyCycle)
print("Number of rotations in 6 seconds :- " + str(rot_ctr))
