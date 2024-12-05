#Aqui poneis el Docstring que querais
#import Wire
#import Adafruit_PWMServoDriver
import board
import busio
import Jetson.GPIO as GPIO
import adafruit_pca9685
import time
i2c = busio.I2C(board.SCL, board.SDA)
from adafruit_servokit import ServoKit


#Declaro variables globales
MIN_PULSE_WIDTH=    650
MAX_PULSE_WIDTH=    2350
FREQUENCY      =    50

#Instancio el Driver del controlador de servos
#Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
pwm = adafruit_pca9685.PCA9685(i2c)
kit = Servokit(channels=16)

#Configuro el SetUP
time.sleep(5)                        #<--- So I have time to get controller to starting 
pca.frequency = FREQUENCY
GPIO.setmode(GPIO.BOARD)
hand = pwm.channel[0]
hand = adafruit_motor.servo.Servo(0) #cualquiera de las dos (coger la linea 26)
pwm.setPWMFreq(FREQUENCY)            
pwm.setPWM(X, 0, 90)                 #Set Gripper to 90 degrees (Close Gripeer) X en Jetson
pwm.begin()
GPIO.setup(channel, GPIO.IN)    # channel tiene que ser un  pin valido en jetson


def  moveMotor(controlIn, motorOut):
  pulse_wide, pulse_width, potVal = -3
  
#  potVal = analogRead(controlIn);                                                   //Read value of Potentiometer
  potVal = GPIO.input(controlIn)
  pulse_wide = map(potVal, 800, 240, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  pulse_width = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);                //Map Potentiometer position to Motor
  
#  pwm.setPWM(motorOut, 0, pulse_width);
  pwm = GPIO.PWM(motorOut, pulse_width)
 


While (True):   #aqui hemos quedado
 moveMotor(potWrist, wrist);
 moveMotor(potElbow, elbow);                                                         //Assign Motors to corresponding Potentiometers
 moveMotor(potShoulder, shoulder);
 moveMotor(potBase, base);                                                          
pushButton = GPIO.input(delcanalquesea)
if(pushButton == GPIO.LOW):

   pwm.setPWM(hand, 0, 180);                                                         //Keep Gripper closed when button is not pressed
   print("Grab")

 else:
  
   pwm.setPWM(hand, 0, 90);                                                          //Open Gripper when button is pressed
   print("Release")




int potWrist = A3;
int potElbow = A2;                        //Assign Potentiometers to pins on Arduino Uno
int potShoulder = A1;
int potBase = A0;

int hand = 11;
int wrist = 12;
int elbow = 13;                           //Assign Motors to pins on Servo Driver Board
int shoulder = 14;
int base = 15;