#!/usr/bin/env python3

from ev3dev2.motor import MoveSteering, OUTPUT_C, OUTPUT_B
#from ev3dev2.sensor import INPUT_2, INPUT_4
from ev3dev2.sensor.lego import UltrasonicSensor, ColorSensor
from ev3dev2.sound import Sound
from ev3dev2.led import Leds
from time import sleep
import math

# Funcion para transformar distancia en grados. En base al diámetro de la rueda del robot
def distancia_a_grados(distancia):
    diametro_ruedas = 5.6 # Debe ser calibrado para el diámetro real de la rueda
    grados = (distancia / (diametro_ruedas * math.pi)) * 360
    return grados

# Función para el calculo del giro de 90 grados en base a la distancia entre las ruedas y diametro de las ruedas
# Longitud entre las 2 ruedas = 12 cm
# Diametro ruedas = 5.6 cm
# Grados para los giros de 90 grados ~= 192 grados
def grados_para_giro_90_grados(): 
    diametro_ruedas = 5.6       # Debe ser calibrado para el diámetro real de la rueda
    longitud_entre_ruedas = 12
    return (((math.pi * longitud_entre_ruedas) / 4) / (math.pi * diametro_ruedas)) * 360

# Clase para controlar los Sensores
class SensorManager:
    def __init__(self, color_sensor, ultrasonic_sensor):
        self.color_sensor = color_sensor
        self.ultrasonic_sensor = ultrasonic_sensor

    def get_color(self):
        return self.color_sensor.color

    def get_distance(self):
        return self.ultrasonic_sensor.distance_centimeters

    def is_object_detected(self, threshold):
        return self.ultrasonic_sensor.distance_centimeters < threshold
    
# Clase para controlar el movimiento
class MovementController:
    def __init__(self, left_motor, right_motor, steering_drive):
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.steering_drive = steering_drive

    def move_forward_steering(self, speed, distance_cm):
        degrees = distancia_a_grados(distance_cm)
        self.steering_drive.on_for_degrees(0, speed, degrees, brake=True, block=True)
    
    def stop_steering(self):
        self.steering_drive.off()

    def turn_left_steering(self, speed, degrees=grados_para_giro_90_grados()):
        self.steering_drive.on_for_degrees(-100, speed, degrees, brake=True, block=True)

    def turn_right_steering(self, speed, degrees=grados_para_giro_90_grados()):
        self.steering_drive.on_for_degrees(100, speed, degrees, brake=True, block=True)

    # Métodos antiguos actuando directamente sobre los motores
    def move_forward(self, speed, distance_cm):
        degrees = distancia_a_grados(distance_cm)
        self.left_motor.on_for_degrees(speed, degrees, brake=True, block=False)
        self.right_motor.on_for_degrees(speed, degrees, brake=True, block=True)

    def stop(self):
        self.left_motor.off()
        self.right_motor.off()

    def turn_left(self, speed, degrees=grados_para_giro_90_grados()):
        self.left_motor.on_for_degrees(-speed, degrees, brake=True, block=False)
        self.right_motor.on_for_degrees(speed, degrees, brake=True, block=True)

    def turn_right(self, speed, degrees=grados_para_giro_90_grados()):
        self.left_motor.on_for_degrees(speed, degrees, brake=True, block=False)
        self.right_motor.on_for_degrees(-speed, degrees, brake=True, block=True)

    
# Inicializar los motores y los sensores
steering_drive = MoveSteering(OUTPUT_B, OUTPUT_C)
color_sensor = ColorSensor()
ultrasonic_sensor = UltrasonicSensor()
sound = Sound()
leds = Leds()
# Controlador de los sensores
sensor_manager = SensorManager(color_sensor, ultrasonic_sensor)
# Controlador del movimiento
movement_controller = MovementController(steering_drive.left_motor, steering_drive.right_motor, steering_drive)




# Función para localizar si el robot está a la derecha o la izquierda de la línea negra.
def maniobra_inicial():
    derecha = False
    izquierda = False
    # Giramos a la izquierda y miramos si hay algún obstaculo mientras giramos
    for _ in range(9):
        movement_controller.turn_left_steering(10, degrees=(grados_para_giro_90_grados() / 9))
        if sensor_manager.is_object_detected(50):
            izquierda = True
            return izquierda, derecha
    # Volvemos a la posición inicial.
    movement_controller.turn_right_steering(10)
    # Giramos a la derecha y miramos si hay algún obstaculo mientras giramos
    for _ in range(9):
        movement_controller.turn_right_steering(10, degrees=(grados_para_giro_90_grados() / 9))
        if sensor_manager.is_object_detected(50):
            derecha = True
            return izquierda, derecha
    return izquierda, derecha
    

# Función para seguir la linea negra y evitar obstáculos
def maniobra_evitar_obstaculos():
    movement_controller.turn_left_steering(10)
    sleep(0.5)
    movement_controller.move_forward_steering(10,21)
    sleep(0.5)
    movement_controller.turn_right_steering(10)
    sleep(0.5)
    movement_controller.move_forward_steering(10, 46)
    sleep(0.5)
    movement_controller.turn_right_steering(10)
    sleep(0.5)
    movement_controller.move_forward_steering(10,21)
    sleep(0.5)
    movement_controller.turn_left_steering(10)


# Programa Principal
def main():
    # Cambiar el color de los leds y hacer un sonido
    leds.set_color("LEFT", "RED")
    leds.set_color("RIGHT", "GREEN")
    sound.beep()

    izquierda, derecha = maniobra_inicial()
    sleep(3)
    if izquierda == True:
        movement_controller.move_forward_steering(30,30)
    else:
        sound.beep()
    if derecha == True:
        movement_controller.turn_left_steering(20)  
    else:
        sound.beep()
        sound.beep()
    #maniobra_evitar_obstaculos()
    

if __name__ == "__main__":
    main()