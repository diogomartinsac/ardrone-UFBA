#!/usr/bin/env python
# coding=utf-8

from __future__ import print_function

import roslib;roslib.load_manifest('drone_custom')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    # self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/ardrone/bottom/image_raw",Image,self.callback)

  def TrataImagem(self,img):
          #obtencao das dimensoes da imagem
    altura_imagem = np.size(img,0)
    largura_imagem= np.size(img,1)
    QtdeContornos = 0
    DirecaoASerTomada = 0
     
    #tratamento da imagem
    # Converts images from BGR to HSV 
    rangomax = np.array([255, 150, 90]) # B, G, R
    rangomin = np.array([90, 20, 0])
  
    # Here we are defining range of bluecolor in HSV 
    # This creates a mask of blue coloured  
    # objects found in the frame. 
    mask = cv2.inRange(img, rangomin, rangomax)
  
    # The bitwise and of the frame and mask is done so  
    #that only the blue coloured objects are highlighted  
    # and stored in res  
    kernel = np.ones((5 ,5), np.uint8)
    FrameBinarizado = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)


     
    #descomente as linhas abaixo se quiser ver o frame apos binarizacao, dilatacao e inversao de cores
    #cv2.imshow('F.B.',mask)
    #
    #cv2.waitKey(10)
 
    _, cnts, _ = cv2.findContours(FrameBinarizado.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(img,cnts,-1,(255,0,255),3)
 
    for c in cnts:
        #se a area do contorno capturado for pequena, nada acontece
        if cv2.contourArea(c) < AreaContornoLimiteMin:
            continue
             
        QtdeContornos = QtdeContornos + 1
 
        #obtem coordenadas do contorno (na verdade, de um retangulo que consegue abrangir todo ocontorno) e
        #realca o contorno com um retangulo.
        
        #x e y: coordenadas do vertice superior esquerdo
        #w e h: respectivamente largura e altura do retangulo
        (x, y, w, h) = cv2.boundingRect(c)  
 
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
     
        #determina o ponto central do contorno e desenha um circulo para indicar
        CoordenadaXCentroContorno = int((x+x+w)/2)
        CoordenadaYCentroContorno = int((y+y+h)/2)
        PontoCentralContorno = (CoordenadaXCentroContorno,CoordenadaYCentroContorno)
        cv2.circle(img, PontoCentralContorno, 1, (0, 0, 0), 5)
         
        DirecaoASerTomada = CoordenadaXCentroContorno - int(largura_imagem/2)   #em relacao a linha central
      
    #output da imagem
    #linha em azul: linha central / referencia
    #linha em verde: linha que mostra distancia entre linha e a referencia
    cv2.line(img, (int(largura_imagem/2) , 0), (int(largura_imagem/2) , altura_imagem), (255,0,0),2)
     
    if (QtdeContornos > 0):
        cv2.line(img,PontoCentralContorno,(int(largura_imagem/2),CoordenadaYCentroContorno),(0,255,0),1)
     
    cv2.imshow('Analise de rota',img)
    cv2.waitKey(10)
    return DirecaoASerTomada, QtdeContornos

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)
    cv_image =  self.TrataImagem(cv_image)
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    # try:
    #   self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    # except CvBridgeError as e:
    #   print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)