import serial
from gpiozero import PWMLED
from time import sleep

ser = serial.Serial("/dev/ttyACM0", 9600)

led = PWMLED(17)
val = 0

while ser.in_waiting:
    ser.readline()

while True:
    
    while ser.in_waiting:
        cc=str(ser.readline())
	val = int(cc)
        print val

    led.value = val/1023.00	
