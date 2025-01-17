#!/usr/bin/env python3

from ev3dev2.motor import MoveSteering, OUTPUT_C, OUTPUT_B
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
    longitud_entre_ruedas = 12  # Debe ser calibrado para longitud real entre las ruedas
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

    def move_backwards_steering(self, speed, distance_cm):
        degrees = distancia_a_grados(distance_cm)
        self.steering_drive.on_for_degrees(0, -speed, degrees, brake=True, block=True)
    
    def stop_steering(self):
        self.steering_drive.off()

    def turn_left_steering(self, speed, degrees=grados_para_giro_90_grados()):
        self.steering_drive.on_for_degrees(-100, speed, degrees, brake=True, block=True)

    def turn_right_steering(self, speed, degrees=grados_para_giro_90_grados()):
        self.steering_drive.on_for_degrees(100, speed, degrees, brake=True, block=True)


    
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


# Función para localizar si el robot está a la derecha o la izquierda de la línea negra y nos deja mirando al obstáculo.
def maniobra_inicial(detection_range):
    deteccion = False
    # Giramos a la izquierda y miramos si hay algún obstaculo mientras giramos
    for i in range(9):
        movement_controller.turn_left_steering(10, degrees=(grados_para_giro_90_grados() / 9))
        if sensor_manager.is_object_detected(detection_range):
            movement_controller.turn_left_steering(10, degrees=(grados_para_giro_90_grados() / 9))
            deteccion = True
            return deteccion
    # Volvemos a la posición inicial.
    movement_controller.turn_right_steering(10)
    # Giramos a la derecha y miramos si hay algún obstaculo mientras giramos
    for i in range(9):
        movement_controller.turn_right_steering(10, degrees=(grados_para_giro_90_grados() / 9))
        if sensor_manager.is_object_detected(detection_range):
            deteccion = True
            movement_controller.turn_right_steering(10, degrees=(grados_para_giro_90_grados() / 9))
            return deteccion
    # Volvemos a la posición inicial.
    movement_controller.turn_left_steering(10)
    return deteccion
    

# Función para seguir la linea negra y evitar obstáculos
def maniobra_evitar_obstaculos(distancia):
    movement_controller.turn_left_steering(10)
    sleep(0.5)
    movement_controller.move_forward_steering(10, (distancia + 12.8))
    sleep(0.5)
    movement_controller.turn_right_steering(10)
    sleep(0.5)
    movement_controller.move_forward_steering(10, (distancia + 5.6) * 2)
    sleep(0.5)
    movement_controller.turn_right_steering(10)
    sleep(0.5)
    movement_controller.move_forward_steering(10, (distancia))
    sleep(0.5)
    for i in range(20):
        if sensor_manager.get_color() != 1:
            movement_controller.move_forward_steering(5, 1)
        else:
            movement_controller.move_forward_steering(10, 7.5)
            break   
    sleep(0.5)
    movement_controller.turn_left_steering(10)

# Función para buscar la linea negra
def maniobra_buscar_linea():
    negro = False
    obstaculos = 0
    
    # Acercamos el robot a 25 cm o menos del obstaculo
    for _ in range(20):
        if sensor_manager.get_distance() > 35:
            sleep(1)
            movement_controller.move_forward_steering(10, 2)
            maniobra_inicial(50)    
        else:
            sound.beep()###################################
            movement_controller.stop_steering()
            break

    # Mide la distancia al obstaculo
    distancia_obstaculo = sensor_manager.get_distance()
    distancia_mod = (distancia_obstaculo + 20) / 10 
    distancia_mod2 = (((distancia_obstaculo + 5.6) * 2) + 20) / 10 
    
    # Gira 90º a la izquierda
    movement_controller.turn_left_steering(10)
    sound.beep()###################################
    # Mover para atrás
    movement_controller.move_backwards_steering(10, 15)

    sound.beep()###################################

    # Se va moviendo mientras mira si encuentra la linea negra
    for i in range(50):
        movement_controller.move_forward_steering(5, 1)
        # Si detecta "NEGRO"
        if sensor_manager.get_color() == 1:
            movement_controller.stop_steering()
            negro = True
            sound.beep()###################################
            break
    if negro == False:
        movement_controller.turn_right_steering(10)
        #  Se va moviendo mientras mira si encuentra la linea negra
        
        for i in range(100):
            movement_controller.move_forward_steering(5, 1)
            # Si detecta "NEGRO"
            if sensor_manager.get_color() == 1:
                movement_controller.stop_steering()
                negro = True
                sound.beep()###################################
                break

            bloqueo = 0
            counter_obj = 0
            step_aux1 = 0
            step_aux2 = 0

            if i % 20 == 0:
                movement_controller.turn_left_steering(10, degrees=45)
                for step in range(54):
                    movement_controller.turn_right_steering(5, degrees=5)
                    if sensor_manager.is_object_detected(50) & bloqueo <= 0:
                        if counter_obj == 1:
                            step_aux2 = step
                            break
                        else:
                            counter_obj += 1
                            bloqueo = 5
                            step_aux1 = step
                    if bloqueo > 0:
                        bloqueo -= 1
                
                steps_move = int((step_aux2 - step_aux1) / 2)
                for _ in range(steps_move):
                    movement_controller.turn_left_steering(5, degrees=5)

                    

    
    # Movemos el robot un poco más para poner las rueda encima de la línea.
    movement_controller.move_forward_steering(10, 7.5)
    sound.beep()###################################

    # Rotamos el robot 180º hacia un obstaculo y lo marcamos.
    for i in range(18):
        movement_controller.turn_right_steering(10, degrees=((grados_para_giro_90_grados() * 2) / 18))
        if sensor_manager.is_object_detected(60):
            obstaculos += 1
            sound.beep()###################################
            break
    # Una vez mirando hacia un obstaculo rotamos 90º 
    movement_controller.turn_right_steering(10)
    sound.beep()###################################

    # Rotamos el robot 90º y marcamos si hay un obstaculo.
    for i in range(18):
        movement_controller.turn_right_steering(10, degrees=((grados_para_giro_90_grados() * 2) / 18))
        if sensor_manager.is_object_detected(60):
            obstaculos += 1
            sound.beep()###################################
            break
    # Si hay 1 solo obstaculo.
    if obstaculos == 1:
        sound.beep()###################################
        movement_controller.turn_right_steering(10)
        for i in range(180):
            if sensor_manager.get_color() != 1:
                movement_controller.turn_right_steering(5, degrees=1)
            else:
                movement_controller.turn_right_steering(5, degrees=5)
                break
        sound.beep()###################################
        # Acercamos el robot a 20 cm o menos del obstaculo
        for _ in range(40):
            flag = True
            if sensor_manager.get_color() != 1:
                flag = True
            else: 
                flag = False
            while flag:
                movement_controller.turn_right_steering(5, 2)
                if sensor_manager.color_sensor() == 1:
                    flag = False
        
            if sensor_manager.get_distance() > 20:
                movement_controller.move_forward_steering(10, 1)
                sound.beep()###################################
            else:
                movement_controller.stop_steering()
                break

        sound.beep()###################################
        maniobra_evitar_obstaculos(sensor_manager.get_distance())
        sound.beep()###################################

    if obstaculos == 2:
        movement_controller.stop_steering()

# Programa Principal
def main():
    deteccion = False
    # Cambiar el color de los leds y hacer un sonido
    leds.set_color("LEFT", "RED")
    leds.set_color("RIGHT", "AMBER")
    sound.beep() 

    # Bucle para buscar el primer obstaculo y nos quedamos mirandolo
    while deteccion == False:
        # Maniobra inical para saber donde está la linea negra y orientarnos. (Detecta objetos a < N cm)
        deteccion = maniobra_inicial(50)
        if deteccion == False:
            movement_controller.move_forward_steering(10, 20)
    
    sound.beep()
    # Hacemos la maniobra para buscar la linea.
    maniobra_buscar_linea()

    # Cambiar el color de los leds y hacer un sonido
    leds.set_color("LEFT", "AMBER")
    leds.set_color("RIGHT", "GREEN")
    sound.beep()
    sound.beep()
    

if __name__ == "__main__":
    main()