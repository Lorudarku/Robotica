"""
Robótica
Grado en Ingeniería Informática
Universidade da Coruña
Author: Alejandro Paz

Ejemplo de uso de sensores/actuadores básicos y cámara con robot Khepera4 en Webots.
"""

from controller import Robot  # Módulo de Webots para el control el robot.
from controller import Camera  # Módulo de Webots para el control de la cámara.

import time  # Si queremos utilizar time.sleep().
import numpy as np  # Si queremos utilizar numpy para procesar la imagen.
import math
# import cv2  # Si queremos utilizar OpenCV para procesar la imagen.

# Máxima velocidad de las ruedas soportada por el robot (khepera4).
MAX_SPEED = 47.6
# Velocidad por defecto para este comportamiento.
CRUISE_SPEED = 8
# Time step por defecto para el controlador.
TIME_STEP = 32

# Nombres de los sensores de distancia basados en infrarrojo.
INFRARED_SENSORS_NAMES = [
    "rear left infrared sensor",
    "left infrared sensor",
    "front left infrared sensor",
    "front infrared sensor",
    "front right infrared sensor",
    "right infrared sensor",
    "rear right infrared sensor",
    "rear infrared sensor",
]

DISTANCIA_PARED = 140
GIRO = (90 * math.pi / 180) * (108.29 / 2) / 21
AVANCE = 250 / 21
MARGEN_ERROR = 0.01

LISTA_ESTADOS = [
    "CREAR_MAPA",
    "PATRULLAR",
    "VOLVER_INICIO",
]

def enable_distance_sensors(robot, timeStep, sensorNames):
    """
    Obtener y activar los sensores de distancia.

    Return: lista con los sensores de distancia activados, en el mismo orden
    establecido en la lista de  nombres (sensorNames).
    """

    sensorList = []

    for name in sensorNames:
        sensorList.append(robot.getDevice(name))

    for sensor in sensorList:
        sensor.enable(timeStep)

    return sensorList


def init_devices(timeStep):
    """
    Obtener y configurar los dispositivos necesarios.

    timeStep: tiempo (en milisegundos) de actualización por defecto para los sensores/actuadores
      (cada dispositivo puede tener un valor diferente).
    """

    # Get pointer to the robot.
    robot = Robot()

    # Si queremos obtener el timestep de la simulación.
    # simTimeStep = int(robot.getBasicTimeStep())

    # Obtener dispositivos correspondientes a los motores de las ruedas.
    leftWheel = robot.getDevice("left wheel motor")
    rightWheel = robot.getDevice("right wheel motor")

    # Configuración inicial para utilizar movimiento por posición (necesario para odometría).
    # En movimiento por velocidad, establecer posición a infinito (wheel.setPosition(float('inf'))).
    leftWheel.setPosition(0)
    rightWheel.setPosition(0)
    leftWheel.setVelocity(0)
    rightWheel.setVelocity(0)

    # Obtener una lista con los sensores infrarrojos ya activados
    irSensorList = enable_distance_sensors(robot, timeStep, INFRARED_SENSORS_NAMES)

    # Obtener el dispositivo de la cámara
    camera = robot.getDevice("camera")
    # Activar el dispositivo de la cámara (el tiempo de actualización de los frames
    # de la cámara no debería ser muy alto debido al alto volumen de datos que implica).
    camera.enable(timeStep * 10)

    # Obtener y activar los sensores de posición de las ruedas (encoders).
    posL = robot.getDevice("left wheel sensor")
    posR = robot.getDevice("right wheel sensor")
    posL.enable(timeStep)
    posR.enable(timeStep)

    # TODO: Obtener y activar otros dispositivos necesarios.
    # ...

    return robot, leftWheel, rightWheel, irSensorList, posL, posR, camera


def process_image_rgb(camera):
    """
    Procesamiento del último frame capturado por el dispositivo de la cámara
    (según el time_step establecido para la cámara).
    ¡ATENCIÓN!: Esta función no es thread-safe, ya que accede al buffer en memoria de la cámara.

    RECOMENDACIÓN: utilizar OpenCV para procesar más eficientemente la imagen
    (ej. hacer una detección de color en HSV).
    """

    W = camera.getWidth()
    H = camera.getHeight()

    image = camera.getImage()

    # Si es suficiente, podríamos procesar solo una parte de la imagen para optimizar .
    for x in range(0, W):
        for y in range(0, H):
            b = camera.imageGetBlue(image, W, x, y)
            g = camera.imageGetGreen(image, W, x, y)
            r = camera.imageGetRed(image, W, x, y)

            # TODO: Procesar el pixel (x,y) de la imagen.
            # ...

def andar(leftWheel, rightWheel, posL, posR, posMap, nextPos, orientacionX, orientacionY):
    print("ANDAR")
    #posMap = [posMap[0] + orientacionY, posMap[1] + orientacionX]
    posMap[0]=posMap[0] + orientacionY
    posMap[1]=posMap[1] + orientacionX
    nextPos[0]=posL.getValue()+AVANCE
    nextPos[1]=posR.getValue()+AVANCE
    leftWheel.setPosition(nextPos[0])
    rightWheel.setPosition(nextPos[1])
    leftWheel.setVelocity(CRUISE_SPEED)
    rightWheel.setVelocity(CRUISE_SPEED)
    return posMap, nextPos

def girarDerecha(leftWheel, rightWheel, posL, posR, nextPos, orientacionX, orientacionY):
    print("DERECHA")
    aux = orientacionX
    orientacionX = orientacionY
    orientacionY = -aux
    nextPos[0]=posL.getValue()+GIRO
    nextPos[1]=posR.getValue()-GIRO
    leftWheel.setPosition(nextPos[0])
    rightWheel.setPosition(nextPos[1])
    leftWheel.setVelocity(CRUISE_SPEED)
    rightWheel.setVelocity(CRUISE_SPEED)
    return nextPos, orientacionX, orientacionY

def girarIzquierda(leftWheel, rightWheel, posL, posR, nextPos, orientacionX, orientacionY):
    print("IZQUIERDA")
    aux = orientacionY
    orientacionY = orientacionX
    orientacionX = -aux
    nextPos[0]=posL.getValue()-GIRO
    nextPos[1]=posR.getValue()+GIRO
    leftWheel.setPosition(nextPos[0])
    rightWheel.setPosition(nextPos[1])
    leftWheel.setVelocity(CRUISE_SPEED)
    rightWheel.setVelocity(CRUISE_SPEED)
    return nextPos, orientacionX, orientacionY

# añadir variable bool para saber si se esta creando mapa o no
def crearMapa(leftWheel, rightWheel, irSensorList, posL, posR, mapa, posMap, nextPos, esquina, orientacionX, orientacionY):
    # 1-Left, 3-Front, 5-Right, 7-Rear
    
    """"
    print("Left " + str(irSensorList[1].getValue()))
    print("Front " + str(irSensorList[3].getValue()))
    print("Right " + str(irSensorList[5].getValue()))
    print("Rear " + str(irSensorList[7].getValue()))
    """

    # ORIENTARSE HACIA LA PARED
    
    # Pared izquierda = Orientado
    # Pared de frente = Girar derecha
    # Pared derecha = Girar derecha
    # Pared atras = Girar izquierda

    if (irSensorList[1].getValue() < DISTANCIA_PARED and mapa[posMap[0]+orientacionX,posMap[1]-orientacionY] == 0 and (not esquina)):
        nextPos, orientacionX,orientacionY = girarIzquierda(leftWheel, rightWheel, posL, posR, nextPos, orientacionX, orientacionY)
        return LISTA_ESTADOS[0], mapa, posMap, nextPos, True, orientacionX, orientacionY
    else:
        if ((not esquina) and mapa[posMap[0]+orientacionX,posMap[1]-orientacionY] == 0):
            mapa[posMap[0]+orientacionX,posMap[1]-orientacionY] = 1
        if (irSensorList[3].getValue() < DISTANCIA_PARED):
            posMap, nextPos = andar(leftWheel, rightWheel, posL, posR, posMap, nextPos, orientacionX, orientacionY)
        else:
            nextPos, orientacionX,orientacionY = girarDerecha(leftWheel, rightWheel, posL, posR, nextPos, orientacionX, orientacionY)
    return LISTA_ESTADOS[0], mapa, posMap, nextPos, False, orientacionX, orientacionY



def main():
    # Activamos los dispositivos necesarios y obtenemos referencias a ellos.
    robot, leftWheel, rightWheel, irSensorList, posL, posR, camera = init_devices(TIME_STEP)

    # Ejecutamos una sincronización para tener disponible el primer frame de la cámara.
    robot.step(TIME_STEP)

    # TODO Implementar arquitectura de control del robot.
    # ...

    estado = LISTA_ESTADOS[0]
    mapa = np.zeros((14,14))
    posMap = [1,1]
    nextPos = [0,0]
    esquina = False
    orientacionX = 1
    orientacionY = 0
    
    while(robot.step(TIME_STEP) != -1):

        #print(posL.getValue(), posR.getValue(), nextPos[0], nextPos[1])
        
        if (posL.getValue() >= (nextPos[0] - MARGEN_ERROR) and posR.getValue() >= (nextPos[1] - MARGEN_ERROR)):
            leftWheel.setVelocity(0)
            rightWheel.setVelocity(0)
            time.sleep(0.5)
            if (estado == LISTA_ESTADOS[0]):
                estado, mapa, posMap, nextPos, esquina, orientacionX, orientacionY = crearMapa(leftWheel, rightWheel, irSensorList, posL, posR, mapa, posMap, nextPos, esquina, orientacionX, orientacionY)
        
        #print(mapa)
        #print(posMap)


if __name__ == "__main__":
    main()
