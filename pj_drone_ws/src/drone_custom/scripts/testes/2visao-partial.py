#!/usr/bin/env python 
import numpy as np

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Vector3, Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Empty

SPEED = 1.0
LimiarBinarizacao = 125  # este valor eh empirico. Ajuste-o conforme sua necessidade
AreaContornoLimiteMin = 5000  # este valor eh empirico. Ajuste-o conforme sua necessidade


# Funcao: trata imagem e retorna se o robo seguidor de linha deve ir para a esqueda ou direita
# Parametros: frame capturado da webcam e primeiro frame capturado
# Retorno: < 0: robo deve ir para a direita
#         > 0: robo deve ir para a esquerda
#         0:   nada deve ser feito

def TrataImagem(img):
    # obtencao das dimensoes da imagem
    height = np.size(img, 0)
    width = np.size(img, 1)
    QtdeContornos = 0
    DirecaoASerTomada = 0
    CoordenadaXCentroContorno = 0
    CoordenadaYCentroContorno = 0

    # tratamento da imagem
    # Converts images from BGR to HSV 
    rangomax = np.array([255, 150, 90])  # B, G, R
    rangomin = np.array([90, 20, 0])

    # Here we are defining range of bluecolor in HSV 
    # This creates a mask of blue coloured  
    # objects found in the frame. 
    mask = cv2.inRange(img, rangomin, rangomax)

    # The bitwise and of the frame and mask is done so  
    # that only the blue coloured objects are highlighted  
    # and stored in res 
    kernel = np.ones((5, 5), np.uint8)
    FrameBinarizado = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # descomente as linhas abaixo se quiser ver o frame apos binarizacao, dilatacao e inversao de cores
    # cv2.imshow('F.B.',mask)
    # cv2.waitKey(10)

    _, cnts, _ = cv2.findContours(FrameBinarizado.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(img, cnts, -1, (255, 0, 255), 3)

    for c in cnts:
        # se a area do contorno capturado for pequena, nada acontece
        if cv2.contourArea(c) < AreaContornoLimiteMin:
            continue

        QtdeContornos = QtdeContornos + 1

        # obtem coordenadas do contorno (na verdade, de um retangulo que consegue abrangir todo ocontorno) e
        # realca o contorno com um retangulo.
        (x, y, w, h) = cv2.boundingRect(c)  # x e y: coordenadas do vertice superior esquerdo
        # w e h: respectivamente largura e altura do retangulo

        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # determina o ponto central do contorno e desenha um circulo para indicar
        CoordenadaXCentroContorno = int((x + x + w) / 2)
        CoordenadaYCentroContorno = int((y + y + h) / 2)
        PontoCentralContorno = (CoordenadaXCentroContorno, CoordenadaYCentroContorno)
        cv2.circle(img, PontoCentralContorno, 1, (0, 0, 0), 5)
        DirecaoASerTomada = CoordenadaXCentroContorno - int(width / 2)  # em relacao a linha central

    # output da imagem
    # linha em azul: linha central / referencia
    # linha em verde: linha que mostra distancia entre linha e a referencia
    cv2.line(img, (int(width / 2), 0), (int(width / 2), height), (255, 0, 0), 2)

    if (QtdeContornos > 0):
        cv2.line(img, PontoCentralContorno, (int(width / 2), CoordenadaYCentroContorno), (0, 255, 0), 1)

    cv2.imshow('Analise de rota', img)
    cv2.waitKey(10)
    return DirecaoASerTomada, QtdeContornos, CoordenadaXCentroContorno, CoordenadaYCentroContorno, h, w


def SensorVerde(img):
    # obtencao das dimensoes da imagem
    height = np.size(img, 0)
    width = np.size(img, 1)
    QtdeContornos = 0

    # tratamento da imagem
    # Converts images from BGR to HSV 
    rango_max_sensor_verde = np.array([95, 180, 120])  # B, G, R
    rango_min_sensor_verde = np.array([25, 80, 45])

    # Here we are defining range of bluecolor in HSV 
    # This creates a mask of blue coloured  
    # objects found in the frame. 
    mask = cv2.inRange(img, rango_min_sensor_verde, rango_max_sensor_verde)

    # The bitwise and of the frame and mask is done so  
    # that only the blue coloured objects are highlighted  
    # and stored in res 
    kernel = np.ones((5, 5), np.uint8)
    FrameBinarizado = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # descomente as linhas abaixo se quiser ver o frame apos binarizacao, dilatacao e inversao de cores
    # cv2.imshow('F.B.',mask)
    # cv2.waitKey(10)

    _, cnts, _ = cv2.findContours(FrameBinarizado.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(img, cnts, -1, (255, 0, 255), 3)

    for c in cnts:
        # se a area do contorno capturado for pequena, nada acontece
        if cv2.contourArea(c) < AreaContornoLimiteMin:
            continue

        QtdeContornos = QtdeContornos + 1

        # obtem coordenadas do contorno (na verdade, de um retangulo que consegue abrangir todo ocontorno) e
        # realca o contorno com um retangulo.
        (x, y, w, h) = cv2.boundingRect(c)  # x e y: coordenadas do vertice superior esquerdo
        # w e h: respectivamente largura e altura do retangulo

        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # determina o ponto central do contorno e desenha um circulo para indicar
        CoordenadaXCentroContorno = int((x + x + w) / 2)
        CoordenadaYCentroContorno = int((y + y + h) / 2)
        PontoCentralContorno = (CoordenadaXCentroContorno, CoordenadaYCentroContorno)
        cv2.circle(img, PontoCentralContorno, 1, (0, 0, 0), 5)

    # output da imagem
    # linha em azul: linha central / referencia
    # linha em verde: linha que mostra distancia entre linha e a referencia
    cv2.line(img, (int(width / 2), 0), (int(width / 2), height), (255, 0, 0), 2)

    if (QtdeContornos > 0):
        cv2.line(img, PontoCentralContorno, (int(width / 2), CoordenadaYCentroContorno), (0, 255, 0), 1)

    cv2.imshow('Sensor Verde', img)
    cv2.waitKey(10)
    return QtdeContornos


def SensorVermelho(img):
    # obtencao das dimensoes da imagem
    height = np.size(img, 0)
    width = np.size(img, 1)
    QtdeContornos = 0

    # tratamento da imagem
    # Converts images from BGR to HSV 
    rango_max_sensor_vermelho = np.array([90, 110, 255])  # B, G, R
    rango_min_sensor_vermelho = np.array([30, 30, 90])

    # Here we are defining range of bluecolor in HSV 
    # This creates a mask of blue coloured  
    # objects found in the frame. 
    mask = cv2.inRange(img, rango_min_sensor_vermelho, rango_max_sensor_vermelho)

    # The bitwise and of the frame and mask is done so  
    # that only the blue coloured objects are highlighted  
    # and stored in res 
    kernel = np.ones((5, 5), np.uint8)
    FrameBinarizado = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # descomente as linhas abaixo se quiser ver o frame apos binarizacao, dilatacao e inversao de cores
    # cv2.imshow('F.B.',mask)
    # cv2.waitKey(10)

    _, cnts, _ = cv2.findContours(FrameBinarizado.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(img, cnts, -1, (255, 0, 255), 3)

    for c in cnts:
        # se a area do contorno capturado for pequena, nada acontece
        if cv2.contourArea(c) < AreaContornoLimiteMin:
            continue

        QtdeContornos = QtdeContornos + 1

        # obtem coordenadas do contorno (na verdade, de um retangulo que consegue abrangir todo ocontorno) e
        # realca o contorno com um retangulo.
        (x, y, w, h) = cv2.boundingRect(c)  # x e y: coordenadas do vertice superior esquerdo
        # w e h: respectivamente largura e altura do retangulo

        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # determina o ponto central do contorno e desenha um circulo para indicar
        CoordenadaXCentroContorno = int((x + x + w) / 2)
        CoordenadaYCentroContorno = int((y + y + h) / 2)
        PontoCentralContorno = (CoordenadaXCentroContorno, CoordenadaYCentroContorno)
        cv2.circle(img, PontoCentralContorno, 1, (0, 0, 0), 5)

    # output da imagem
    # linha em azul: linha central / referencia
    # linha em verde: linha que mostra distancia entre linha e a referencia
    cv2.line(img, (int(width / 2), 0), (int(width / 2), height), (255, 0, 0), 2)

    if (QtdeContornos > 0):
        cv2.line(img, PontoCentralContorno, (int(width / 2), CoordenadaYCentroContorno), (0, 255, 0), 1)

    cv2.imshow('Sensor Vermelho', img)
    cv2.waitKey(10)
    return QtdeContornos


def takeoff():
    takeoff_pub.publish(Empty())


def land():
    land_pub.publish(Empty())


def foward():
    cmdVel_pub.publish(Twist(linear=Vector3(SPEED, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0)))


def stop():
    cmdVel_pub.publish(Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0)))


def backward():
    cmdVel_pub.publish(Twist(linear=Vector3(-SPEED, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0)))


def left():
    cmdVel_pub.publish(Twist(linear=Vector3(0.0, SPEED, 0.0), angular=Vector3(0.0, 0.0, 0.0)))


def right():
    cmdVel_pub.publish(Twist(linear=Vector3(0.0, -SPEED, 0.0), angular=Vector3(0.0, 0.0, 0.0)))


def up():
    cmdVel_pub.publish(Twist(linear=Vector3(0.0, 0.0, SPEED), angular=Vector3(0.0, 0.0, 0.0)))


def down():
    cmdVel_pub.publish(Twist(linear=Vector3(0.0, 0.0, -SPEED), angular=Vector3(0.0, 0.0, 0.0)))


def turnLeft():
    cmdVel_pub.publish(Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, SPEED)))


def turnRight():
    cmdVel_pub.publish(Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, -SPEED)))


def callbackImage(data):
    try:
        frame = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    Direcao, QtdeLinhas = TrataImagem(frame)
    ContVerde = SensorVerde(frame)
    ContVermelho = SensorVermelho(frame)

    if (QtdeLinhas == 0):
        print ("Nenhuma linha encontrada. O robo ira parar.")

    if (Direcao > 10):
        print ("Distancia da linha de referencia: " + str(abs(Direcao)) + " pixels a direita")
        # right()

    if (Direcao < -10):
        print ("Distancia da linha de referencia: " + str(abs(Direcao)) + " pixels a esquerda")
        # left()

    if (Direcao >= -10 and Direcao <= 10):
        print ("Exatamente na linha de referencia!")
        # turnLeft()
        # stop()

    if (ContVerde != 0):
        print ("Sensor sem defeito!")

    if (ContVermelho != 0):
        print ("Sensor sem defeito!")


def menu():
    print ("t: takeoff")
    print ("l: land")
    print ("f: foward")
    print ("s: stop")
    print ("b: backward")
    print ("a: left")
    print ("d: right")
    print ("w: up")
    print ("s: down")
    print ("e: turn right")
    print ("q: turn left")


if __name__ == '__main__':
    rospy.init_node('ardrone_control_node', anonymous=True)

    takeoff_pub = rospy.Publisher("ardrone/takeoff", Empty, queue_size=10)
    takeoff_pub.publish(2, 3, 4)

    land_pub = rospy.Publisher("ardrone/land", Empty, queue_size=10)
    cmdVel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

    bridge = CvBridge()
    img_sbs = rospy.Subscriber("ardrone/image_raw", Image, callbackImage)
    rate = rospy.Rate(10)
    rospy.spin()
    # try:
    #     while not rospy.is_shutdown():
    #         # menu()
    #         # print ("input a key for action")
    #         # key = sys.stdin.read(1)

    #         # takeoff()

    #         # if (key == str('t')):
    #         #     takeoff()
    #         # elif (key == str('l')):
    #         #     land()
    #         # elif (key == str('f')):
    #         #     foward()
    #         # elif (key == str('p')):
    #         #     stop()
    #         # elif (key == str('b')):
    #         #     backward()
    #         # elif (key == str('a')):
    #         #     left()
    #         # elif (key == str('d')):
    #         #     right()
    #         # elif (key == str('w')):
    #         #     up()
    #         # elif (key == str('s')):
    #         #     down()
    #         # elif (key == str('e')):
    #         #     turnRight()
    #         # elif (key == str('q')):
    #         #     turnLeft()

    # except rospy.ROSInterruptException:
    #     pass
