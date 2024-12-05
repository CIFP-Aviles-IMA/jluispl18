#Aqui poneis el Docstring que querais
"""
Este código está sacado de https://www.printables.com/model/818975-compact-robot-arm-arduino-3d-printed/files.
Este script está diseñado para controlar un brazo robótico utilizando servomotores y un controlador PWM PCA9685 conectado a una placa Jetson. El brazo robótico cuenta con varios servomotores que permiten mover sus articulaciones (hombro, codo, muñeca, base y garra) y sensores de posición (potenciómetros) que permiten el control de estos servos en tiempo real. Además, se incluye un botón que, al ser presionado, controla la apertura y cierre de la garra del brazo robótico.

Requisitos:
-Jetson.GPIO: Control de los pines GPIO en la placa Jetson.
-adafruit_pca9685: Comunicación con el controlador PWM PCA9685.
-adafruit_servokit: Gestión de servomotores a través de la librería Adafruit.
-time: Introducción de retrasos entre las acciones y configuración del sistema.

Funcionamiento:
- El script configura los servos a través del controlador PWM PCA9685, que maneja las señales PWM necesarias para controlar la posición de cada motor.
- Los potenciómetros son leídos a través de los pines GPIO y sus valores se mapean a un rango que se usa para ajustar el ángulo de los servos de las articulaciones (hombro, codo, muñeca, base).
- El valor del potenciómetro se convierte en un ancho de pulso PWM para controlar el movimiento de los servos.
- Un botón conectado al pin GPIO 15 permite controlar la garra del brazo. Cuando el botón no está presionado, la garra se cierra, y cuando está presionado, la garra se abre.
Funciones Principales:
- moveMotor (controlIn, motorOut): Lee el valor de un potenciómetro conectado a un pin GPIO y ajusta el movimiento del motor (servo) correspondiente según la posición del potenciómetro.
- El script entra en un bucle infinito donde se ajustan constantemente las posiciones de los servos de acuerdo con los valores de los potenciómetros y se controla la garra según el estado del botón.

Parámetros de Configuración:
- MIN_PULSE_WIDTH: Ancho de pulso mínimo para el movimiento de los servos (650 microsegundos).
- MAX_PULSE_WIDTH: Ancho de pulso máximo para el movimiento de los servos (2350 microsegundos).
- FREQUENCY: Frecuencia de actualización de la señal PWM (50 Hz).
"""




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
potWrist = GPIO.input(29)
potElbow = GPIO.input(31)  
potShoulder = GPIO.input(33)
potBase = GPIO.input(35)



#Configuro el SetUP
time.sleep(5)                        #<---So I have time to get controller to starting 
"pca.frequency" = FREQUENCY
GPIO.setmode(GPIO.BOARD)
hand = adafruit_motor.servo.Servo(1) #cualquiera de las dos
wrist = adafruit_motor.servo.Servo(1)
elbow = adafruit_motor.servo.Servo(2)
shoulder = adafruit_motor.servo.Servo(3) 
base = adafruit_motor.servo.Servo(4)
potWrist = adafruit_motor.servo.Servo(5)
potElbow = adafruit_motor.servo.Servo(6)
potShoulder = adafruit_motor.servo.Servo(7)
potBase = adafruit_motor.servo.Servo(8)

pwm.setPWMFreq(FREQUENCY) 
pwm.setPWM(32, 0, 90)                 #Set Gripper to 90 degrees (Close Gripeer) X en Jetson
pwm.begin()
GPIO.setup(7, GPIO.IN)    # Channel tiene que ser un  pin válido en jetson


def  moveMotor(controlIn, motorOut):
 Descripcion(controlIn, motorOut)
"""
Args:
controLin (int):El valor del potenciómetro se lee a través del pin GPIO designado, que se obtiene mediante el pin que se selecciona.
motorOut (int):El pin de salida del motor se emplea para transmitir la señal PWM al motor que se quiere controlar.
Returns:
Esta funcion implica el retroceso de la posicion del robot segun el valor que de el potenciómetro.
"""
pulse_wide, pulse_width, potVal = -7
  
#  potVal = analogRead(controlIn);                                                #//Read value of Potentiometer
potVal = GPIO.input(controlIn)
pulse_wide = map(potVal, 800, 240, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH)
pulse_width = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);              #//Map Potentiometer position to Motor
#  pwm.setPWM(motorOut, 0, pulse_width);
pwm = GPIO.PWM(motorOut, 0,  pulse_width)
 


  While(True):
  moveMotor(potWrist, wrist)
  moveMotor(potElbow, elbow)                                                         #//Assign Motors to corresponding Potentiometers
  moveMotor(potShoulder, shoulder)
  moveMotor(potBase, base) 
  pushButton = GPIO.input(7)
  if(pushButton == GPIO.LOW):
    pwm.setPWM(hand, 0, 180);                                                        # //Keep Gripper closed when button is not pressed
    print("Grab")
    
  else:

    pwm.setPWM(hand, 0, 90);                                                          #//Open Gripper when button is pressed
    print("Release")
    
  GPIO.cleanup()