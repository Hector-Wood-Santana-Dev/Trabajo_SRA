#!/usr/bin/env python3

from ev3dev2.motor import MoveSteering, OUTPUT_C, OUTPUT_B
from ev3dev2.sensor import INPUT_2, INPUT_4
from ev3dev2.sensor.lego import UltrasonicSensor, ColorSensor
from ev3dev2.sound import Sound
from ev3dev2.led import Leds
from time import sleep
import math

# Funcion para transformar distancia en grados
def distancia_a_grados(distancia):
    return (distancia / (5.6 * math.pi)) * 360

# Función para el calculo del giro de 90 grados en base a la distancia entre las ruedas y diametro de las ruedas
# Longitud entre las 2 ruedas = 12 cm
# Diametro ruedas = 5.6 cm
# Grados para los giros de 90 grados ~= 192 grados
def grados_para_giro_90_grados():
    diametro_ruedas = 5.6
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
    def __init__(self, left_motor, right_motor):
        self.left_motor = left_motor
        self.right_motor = right_motor

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
steering_drive = MoveSteering(OUTPUT_C, OUTPUT_B)
color_sensor = ColorSensor(INPUT_4)
ultrasonic_sensor = UltrasonicSensor(INPUT_2)
sound = Sound()
leds = Leds()
# Controlador de los sensores
sensor_manager = SensorManager(color_sensor, ultrasonic_sensor)
# Controlador del movimiento
movement_controller = MovementController(steering_drive.left_motor, steering_drive.right_motor)



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
def seguir_linea_y_evitar_obstaculos():
    False



# Programa Principal
def main():
    # Cambiar el color de los leds y hacer un sonido
    leds.set_color("LEFT", "RED")
    leds.set_color("RIGHT", "GREEN")
    sound.beep()

    movement_controller.move_forward(30, 30)
    sleep(1)
    movement_controller.turn_left(30)
    sleep(1)
    movement_controller.move_forward(30, 30)
    sleep(1)
    movement_controller.turn_right(30)
    sleep(1)
    movement_controller.move_forward(30, 30)
    sleep(1)

    while True:
        break
        color = sensor_manager.get_color()
        distance = sensor_manager.get_distance()

        #buscar_linea_y_obstaculo(color, threshold=30)
    

        time.sleep(0.1)  # Small delay to prevent overwhelming the sensors

if __name__ == "__main__":
    main()