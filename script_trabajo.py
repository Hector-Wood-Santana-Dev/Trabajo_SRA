#!/usr/bin/env python3

from ev3dev2.motor import MoveSteering, OUTPUT_C, OUTPUT_B
from ev3dev2.sensor import INPUT_2, INPUT_4
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

    def move_forward(self, speed, distance_cm):
        degrees = distancia_a_grados(distance_cm)
        self.steering_drive.on_for_degrees(0, speed, degrees, brake=True, block=True)
    
    def stop(self):
        self.steering_drive.off()

    def turn_left_steering(self, speed, degrees=grados_para_giro_90_grados()):
        self.steering_drive.on_for_degrees(-100, speed, degrees, brake=True, block=True)

    def turn_right_steering(self, speed, degrees=grados_para_giro_90_grados()):
        self.steering_drive.on_for_degrees(100, speed, degrees, brake=True, block=True)

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
color_sensor = ColorSensor(INPUT_4)
ultrasonic_sensor = UltrasonicSensor(INPUT_2)
sound = Sound()
leds = Leds()
# Controlador de los sensores
sensor_manager = SensorManager(color_sensor, ultrasonic_sensor)
# Controlador del movimiento
movement_controller = MovementController(steering_drive.left_motor, steering_drive.right_motor, steering_drive)



# Función para buscar la linea negra y llegar al primer obstáculo
def buscar_linea_y_obstaculo(color, threshold):
    while color != "BLACK":
        while movement_controller.turn_left():
            if sensor_manager.is_object_detected(threshold):
                return True
        movement_controller.turn_right()
        while movement_controller.turn_right():
            if sensor_manager.is_object_detected(threshold):
                return True
            
    
# Función para seguir la linea negra y evitar obstáculos
def maniobra_evitar_obstaculos():
    movement_controller.turn_left(10)
    sleep(1)
    movement_controller.move_forward(10,20)
    sleep(1)
    movement_controller.turn_right(10)
    sleep(1)
    movement_controller.move_forward(10, 46)
    sleep(1)
    movement_controller.turn_right(10)
    sleep(1)
    movement_controller.move_forward(10,20)
    sleep(1)
    movement_controller.turn_left(10)




# Programa Principal
def main():
    # Cambiar el color de los leds y hacer un sonido
    leds.set_color("LEFT", "RED")
    leds.set_color("RIGHT", "GREEN")
    sound.beep()
    #maniobra_evitar_obstaculos()
    movement_controller.move_forward(10, 20)
    movement_controller.turn_left_steering(10)
    movement_controller.turn_right_steering(10)
    sleep(5)
    movement_controller.turn_left(10)
    movement_controller.turn_right(10)

if __name__ == "__main__":
    main()