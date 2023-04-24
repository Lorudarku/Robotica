"""
Robótica
Grado en Ingeniería Informática
Universidade da Coruña
Author: Alejandro Paz

Ejemplo de uso de sensores/actuadores básicos y cámara con robot Khepera4 en Webots.
"""

from controller import Robot  # Módulo de Webots para el control el robot.
from controller import Camera  # Módulo de Webots para el control de la cámara.

from queue import PriorityQueue
import time  # Si queremos utilizar time.sleep().
import numpy as np  # Si queremos utilizar numpy para procesar la imagen.
import math
#import cv2  # Si queremos utilizar OpenCV para procesar la imagen.


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

DISTANCIA_PARED = 150
GIRO = (90 * math.pi / 180) * (108.29 / 2) / 21
AVANCE = 250 / 21
MARGEN_ERROR = 0.001

LISTA_ESTADOS = [
    "ORIENTARSE",
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
    ####
    #Procesamiento del último frame capturado por el dispositivo de la cámara
    #(según el time_step establecido para la cámara).
    #¡ATENCIÓN!: Esta función no es thread-safe, ya que accede al buffer en memoria de la cámara.
    #
    #RECOMENDACIÓN: utilizar OpenCV para procesar más eficientemente la imagen
    #(ej. hacer una detección de color en HSV).
    ####

    yellowCounter = 0

    W = camera.getWidth()
    H = camera.getHeight()

    image = camera.getImage()

    #pixels = np.zeros((W, H, 3))

    # Si es suficiente, podríamos procesar solo una parte de la imagen para optimizar .
    for x in range(0, W):
        for y in range(0, H):
            b = camera.imageGetBlue(image, W, x, y)
            g = camera.imageGetGreen(image, W, x, y)
            r = camera.imageGetRed(image, W, x, y)

            if r >= 180 and g >= 180 and b <= 100:
            #if r >= 220 and g >= 220 and b >= 60:
                yellowCounter += 1

            #pixels[x, y] = [b, g, r]
 
    # pixeles totales 360960
    umbral = H*W/3
    print("Yellow pixels: ", yellowCounter)
    print("Umbral: ", umbral)
    
    if yellowCounter >= umbral:
        return True
    else:
        return False

""" 
def process_image_rgb(camera):
    ####
    #Procesamiento del último frame capturado por el dispositivo de la cámara
    #(según el time_step establecido para la cámara).
    #¡ATENCIÓN!: Esta función no es thread-safe, ya que accede al buffer en memoria de la cámara.
    #
    #RECOMENDACIÓN: utilizar OpenCV para procesar más eficientemente la imagen
    #(ej. hacer una detección de color en HSV).
    ####

    yellowCounter = 0

    W = camera.getWidth()
    H = camera.getHeight()

    image = camera.getImage()

    image = np.frombuffer(image, np.uint8).reshape((H, W, 4))
    
    # Convertir la imagen a formato HSV para facilitar la detección de color
    hsv_image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    # Definir los rangos de color para el amarillo en formato HSV
    lower_yellow = np.array([25, 100, 100])
    upper_yellow = np.array([35, 255, 255])

    # Aplicar la máscara para detectar el color amarillo
    mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
    # Contar los píxeles que cumplen la condición
    yellowCounter = np.count_nonzero(mask)
    print(yellowCounter)

    if yellowCounter >= H*W/2:
        return True
    else:
        return False
 """

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

def orientarse(leftWheel, rightWheel, irSensorList, posL, posR, nextPos, orientacionX, orientacionY):
    # 1-Left, 3-Front, 5-Right, 7-Rear

    # ORIENTARSE HACIA LA PARED
    
    # Pared izquierda = Orientado
    # Pared de frente = Girar derecha
    # Pared derecha = Girar derecha
    # Pared atras = Girar izquierda

    print("ORIENTARSE")
    if (irSensorList[1].getValue() >= DISTANCIA_PARED):
        return LISTA_ESTADOS[1], nextPos, orientacionX, orientacionY
    elif (irSensorList[3].getValue() >= DISTANCIA_PARED):
        nextPos,orientacionX,orientacionY = girarDerecha(leftWheel, rightWheel, posL, posR, nextPos, orientacionX, orientacionY)
        return LISTA_ESTADOS[0], nextPos, orientacionX, orientacionY
    elif (irSensorList[7].getValue() >= DISTANCIA_PARED):
        nextPos,orientacionX,orientacionY = girarIzquierda(leftWheel, rightWheel, posL, posR, nextPos, orientacionX, orientacionY)
        return LISTA_ESTADOS[0], nextPos, orientacionX, orientacionY
    elif (irSensorList[5].getValue() >= DISTANCIA_PARED):
        nextPos,orientacionX,orientacionY = girarDerecha(leftWheel, rightWheel, posL, posR, nextPos, orientacionX, orientacionY)
        return LISTA_ESTADOS[0], nextPos, orientacionX, orientacionY
    else:
        return LISTA_ESTADOS[0], nextPos, orientacionX, orientacionY


# añadir variable bool para saber si se esta creando mapa o no
def crearMapa(leftWheel, rightWheel, irSensorList, posL, posR, mapa, mapaCreado, posMap, nextPos, esquina, orientacionX, orientacionY):
    # 1-Left, 3-Front, 5-Right, 7-Rear
    
    """
    print("Left " + str(irSensorList[1].getValue()))
    print("Front " + str(irSensorList[3].getValue()))
    print("Right " + str(irSensorList[5].getValue()))
    print("Rear " + str(irSensorList[7].getValue()))
    """
    
    if (posMap == [1,1]):
        mapaCreado+=1
        if (mapaCreado == 2):
            if ((not esquina) and mapa[posMap[0]+orientacionX,posMap[1]-orientacionY] == 0):
                mapa[posMap[0]+orientacionX,posMap[1]-orientacionY] = 1 # Pared
            return LISTA_ESTADOS[2], mapa, mapaCreado, posMap, nextPos, False, orientacionX, orientacionY

    if (irSensorList[1].getValue() < DISTANCIA_PARED and (not esquina)):
        nextPos, orientacionX,orientacionY = girarIzquierda(leftWheel, rightWheel, posL, posR, nextPos, orientacionX, orientacionY)
        return LISTA_ESTADOS[1], mapa, mapaCreado, posMap, nextPos, True, orientacionX, orientacionY
    else:
        if ((not esquina) and mapa[posMap[0]+orientacionX,posMap[1]-orientacionY] == 0):
            mapa[posMap[0]+orientacionX,posMap[1]-orientacionY] = 1 # Pared
        if (irSensorList[3].getValue() < DISTANCIA_PARED):
            posMap, nextPos = andar(leftWheel, rightWheel, posL, posR, posMap, nextPos, orientacionX, orientacionY)
        else:
            nextPos, orientacionX,orientacionY = girarDerecha(leftWheel, rightWheel, posL, posR, nextPos, orientacionX, orientacionY)
            
    return LISTA_ESTADOS[1], mapa, mapaCreado, posMap, nextPos, False, orientacionX, orientacionY




def patrullar(leftWheel, rightWheel, irSensorList, posL, posR, mapa, posMap, nextPos, esquina, orientacionX, orientacionY):

    if (irSensorList[1].getValue() < DISTANCIA_PARED and (not esquina)):
        nextPos, orientacionX,orientacionY = girarIzquierda(leftWheel, rightWheel, posL, posR, nextPos, orientacionX, orientacionY)
        return LISTA_ESTADOS[2], mapa, posMap, nextPos, True, orientacionX, orientacionY
    else:
        if (irSensorList[3].getValue() < DISTANCIA_PARED):
            posMap, nextPos = andar(leftWheel, rightWheel, posL, posR, posMap, nextPos, orientacionX, orientacionY)
        else:
            nextPos, orientacionX,orientacionY = girarDerecha(leftWheel, rightWheel, posL, posR, nextPos, orientacionX, orientacionY)
            
    return LISTA_ESTADOS[2], mapa, posMap, nextPos, False, orientacionX, orientacionY



#Implementacion de la clase A*
class Nodo:
    def __init__(self, pos, padre=None, g=0, h=0):
        x, y = pos
        self.pos = (x, y) #Si no se pone como tupla, da error
        self.padre = padre
        self.g = g
        self.h = h
        self.f = g+h

    def __lt__(self, otro):
        return self.f < otro.f
    
    def camino(self):
        camino = []
        nodo = self
        while nodo != None:
            camino.append(nodo.pos)
            nodo = nodo.padre
        return camino[::-1] #Invertir la lista
    
def vecinos(pos, mapa):
    x, y = pos
    vecinos = [(x+1, y), (x-1, y), (x, y+1), (x, y-1)]
    vecinos_validos = []
    for i, j in vecinos:
            if mapa[i,j] != 1:
                vecinos_validos.append((i, j))
    return vecinos_validos
    
def a_estrella(inicio, meta, mapa):
    meta2tupla = (meta[0], meta[1]) #Como la pos es una tupla, hay que convertirlo
    print("Buscando camino")
    frontera = PriorityQueue()
    nodo_inicio = Nodo(inicio)
    frontera.put(nodo_inicio)
    visitados = set()
    while not frontera.empty():
        print("Frontera: " + str(frontera.qsize()) + " Visitados: " + str(len(visitados)))
        nodo_actual = frontera.get()
        if nodo_actual.pos == meta2tupla:
            print("Encontrado")
            return nodo_actual.camino()
        visitados.add(nodo_actual.pos)
        for vecino in vecinos(nodo_actual.pos, mapa):
            if vecino in visitados:
                continue #Salta a la siguiente iteracion del bucle
            nuevo_g = nodo_actual.g + 1
            nueva_h = abs(vecino[0] - meta[0]) + abs(vecino[1] - meta[1])
            print ("Nuevo g: " + str(nuevo_g))
            print ("Nueva h: " + str(nueva_h))
            nodo_vecino = Nodo(vecino, nodo_actual, nuevo_g, nueva_h)
            frontera.put(nodo_vecino)
    return None

    
    
def volverInicio(leftWheel, rightWheel, irSensorList, posL, posR, actualizarRuta, mapa, ruta, posMap, nextPos, esquina, orientacionX, orientacionY):
        print("VOLVIENDO VOLVIENDO VOLVIENDO VOLVIENDO VOLVIENDO")
        print("Posicion actual: " + str(posMap))
        print("Ruta: " + str(ruta))
        print(orientacionX, orientacionY)
        print("=====================================================")


        if ((orientacionX == 1) and (orientacionY == 0)): #Si mirando abajo
            if (ruta == (posMap[0], posMap[1]-1)): #y queremos ir hacia arriba
                nextPos, orientacionX,orientacionY = girarIzquierda(leftWheel, rightWheel, posL, posR, nextPos, orientacionX, orientacionY) 
            elif (ruta == (posMap[0], posMap[1]+1)): #y queremos ir hacia abajo
                posMap, nextPos = andar(leftWheel, rightWheel, posL, posR, posMap, nextPos, orientacionX, orientacionY)
            elif (ruta == (posMap[0]-1, posMap[1])): #y queremos ir hacia la izquierda
                nextPos, orientacionX,orientacionY = girarDerecha(leftWheel, rightWheel, posL, posR, nextPos, orientacionX, orientacionY)
            elif (ruta == (posMap[0]+1, posMap[1])): #y queremos ir hacia la derecha
                nextPos, orientacionX,orientacionY = girarIzquierda(leftWheel, rightWheel, posL, posR, nextPos, orientacionX, orientacionY)    

        elif ((orientacionX == -1) and (orientacionY == 0)): #Si mirando arriba
            if (ruta == (posMap[0], posMap[1]-1)): #y queremos ir hacia arriba
                posMap, nextPos = andar(leftWheel, rightWheel, posL, posR, posMap, nextPos, orientacionX, orientacionY)
            elif (ruta == (posMap[0], posMap[1]+1)): #y queremos ir hacia abajo
                nextPos, orientacionX,orientacionY = girarDerecha(leftWheel, rightWheel, posL, posR, nextPos, orientacionX, orientacionY)
            elif (ruta == (posMap[0]-1, posMap[1])): #y queremos ir hacia la izquierda
                nextPos, orientacionX,orientacionY = girarIzquierda(leftWheel, rightWheel, posL, posR, nextPos, orientacionX, orientacionY)
            elif (ruta == (posMap[0]+1, posMap[1])): #y queremos ir hacia la derecha
                nextPos, orientacionX,orientacionY = girarDerecha(leftWheel, rightWheel, posL, posR, nextPos, orientacionX, orientacionY)
                
        elif ((orientacionX == 0) and (orientacionY == 1)): #Si mirando derecha
            if (ruta == (posMap[0], posMap[1]-1)): #y queremos ir hacia arriba
                nextPos, orientacionX,orientacionY = girarIzquierda(leftWheel, rightWheel, posL, posR, nextPos, orientacionX, orientacionY)
            elif (ruta == (posMap[0], posMap[1]+1)): #y queremos ir hacia abajo
                nextPos, orientacionX,orientacionY = girarDerecha(leftWheel, rightWheel, posL, posR, nextPos, orientacionX, orientacionY)
            elif (ruta == (posMap[0]-1, posMap[1])): #y queremos ir hacia la izquierda
                nextPos, orientacionX,orientacionY = girarIzquierda(leftWheel, rightWheel, posL, posR, nextPos, orientacionX, orientacionY)
            elif (ruta == (posMap[0]+1, posMap[1])): #y queremos ir hacia la derecha
                posMap, nextPos = andar(leftWheel, rightWheel, posL, posR, posMap, nextPos, orientacionX, orientacionY)

        elif ((orientacionX == 0) and (orientacionY == -1)): #Si mirando izquierda
            if (ruta == (posMap[0], posMap[1]-1)): #y queremos ir hacia arriba
                nextPos, orientacionX,orientacionY = girarDerecha(leftWheel, rightWheel, posL, posR, nextPos, orientacionX, orientacionY)
            elif (ruta == (posMap[0], posMap[1]+1)): #y queremos ir hacia abajo
                nextPos, orientacionX,orientacionY = girarIzquierda(leftWheel, rightWheel, posL, posR, nextPos, orientacionX, orientacionY)
            elif (ruta == (posMap[0]-1, posMap[1])): #y queremos ir hacia la izquierda
                posMap, nextPos = andar(leftWheel, rightWheel, posL, posR, posMap, nextPos, orientacionX, orientacionY)
            elif (ruta == (posMap[0]+1, posMap[1])): #y queremos ir hacia la derecha
                nextPos, orientacionX,orientacionY = girarDerecha(leftWheel, rightWheel, posL, posR, nextPos, orientacionX, orientacionY)
        

        if (ruta == (posMap[0], posMap[1])):
            actualizarRuta = True
        
        return actualizarRuta, mapa, ruta, posMap, nextPos, esquina, orientacionX, orientacionY

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
    mapaCreado = 0
    amarillo = False
    actualizarRuta = True

    while(robot.step(TIME_STEP) != -1):

        #print(posL.getValue(), posR.getValue(), nextPos[0], nextPos[1])
        #print("Estado: " + str(estado))
        
        if (posL.getValue() >= (nextPos[0] - MARGEN_ERROR) and posR.getValue() >= (nextPos[1] - MARGEN_ERROR)):
            leftWheel.setVelocity(0)
            rightWheel.setVelocity(0)
            time.sleep(0.5)
            if (estado == LISTA_ESTADOS[0]):
                estado, nextPos, orientacionX, orientacionY = orientarse(leftWheel, rightWheel, irSensorList, posL, posR, nextPos, orientacionX, orientacionY)
            elif (estado == LISTA_ESTADOS[1]):
                estado, mapa, mapaCreado, posMap, nextPos, esquina, orientacionX, orientacionY = crearMapa(leftWheel, rightWheel, irSensorList, posL, posR, mapa, mapaCreado, posMap, nextPos, esquina, orientacionX, orientacionY)
            elif (estado == LISTA_ESTADOS[2]):
                amarillo = process_image_rgb(camera)
                if (amarillo):
                    #Planificar ruta de vuelta
                    camino = a_estrella(posMap, [1,1], mapa)
                    estado = LISTA_ESTADOS[3]
                else:
                    estado, mapa, posMap, nextPos, esquina, orientacionX, orientacionY = patrullar(leftWheel, rightWheel, irSensorList, posL, posR, mapa, posMap, nextPos, esquina, orientacionX, orientacionY)
            elif (estado == LISTA_ESTADOS[3]):
                if (actualizarRuta):
                    #Sacamos el primer elemento de la lista
                    ruta = camino.pop(0)   
                    actualizarRuta = False
                actualizarRuta, mapa, ruta, posMap, nextPos, esquina, orientacionX, orientacionY = volverInicio(leftWheel, rightWheel, irSensorList, posL, posR, actualizarRuta, mapa, ruta, posMap, nextPos, esquina, orientacionX, orientacionY)
                if (posMap == [1,1]):
                    print("fin")
                    break
    
        #print(mapa)
        #print(posMap)
        #print(estado)
    
if __name__ == "__main__":
    main()
