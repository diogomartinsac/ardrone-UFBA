#!/usr/bin/env python
# coding=utf-8

from __future__ import print_function

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import rospy
import sys
import roslib;
roslib.load_manifest('drone_custom')


class image_converter:

    nome_tela_frontal = 'Camera frontal'
    nome_tela_baixo = 'Camera Baixo'
    nome_tela_Autonomo = 'Modo Autonomo'

    def __init__(self):
        # self.image_pub = rospy.Publisher("image_topic_2",Image)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/ardrone/front/image_raw", Image, self.callback)
        # self.image_sub2 = rospy.Subscriber(
        #     "/ardrone/bottom/image_raw", Image, self.callback)

        self.mostrar_tela_frontal = False
        self.mostrar_tela_baixo = False
        self.mostrar_tela_Autonomo = False
        self.status_tela_frontal = False
        self.status_tela_baixo = False
        self.status_tela_Autonomo = False

    def callback(self, data):
        if self.mostrar_tela_frontal or self.mostrar_tela_baixo or self.mostrar_tela_Autonomo:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                print(e)

            # (rows,cols,channels) = cv_image.shape
            # if cols > 60 and rows > 60 :
            #   cv2.circle(cv_image, (50,50), 10, 255)
            
            # if self.mostrar_tela_frontal:
            #     cv_image_baixo = cv_image
            cv2.imshow(self.nome_tela_frontal, cv_image)
                # self.status_tela_frontal = True
            # else:
            #     if self.status_tela_frontal is True:
            #         cv2.destroyWindow(self.nome_tela_Autonomo)
            #         self.status_tela_frontal = False

            # if self.mostrar_tela_baixo:
            #     cv_image_baixo = cv_image
            #     cv2.imshow(self.nome_tela_frontal, cv_image_baixo)
            #     self.status_tela_baixo = True
            # else:
            #     if self.status_tela_baixo is True:
            #         cv2.destroyWindow(self.nome_tela_Autonomo)
            #         self.status_tela_baixo = False

            # if self.mostrar_tela_Autonomo:
            #     cv_image_baixo = cv_image
                # cv2.imshow(self.nome_tela_frontal, cv_image_baixo)
            #     self.status_tela_Autonomo = True
            # else:
            #     if self.status_tela_Autonomo is True:
            #         cv2.destroyWindow(self.nome_tela_Autonomo)
            #         self.status_tela_Autonomo = False

        # cv2.waitKey(3)

        # try:
        #   self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        # except CvBridgeError as e:
        #   print(e)


def main(args):
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        # cv2.destroyWindow('max')
        # rate = rospy.Rate(10) # 10hz
        # while not rospy.is_shutdown():
        #     print('Menu\n'
        #           '1 - Mostrar/Ocultar Tela Frontal\n'
        #           '2 - Mostrar/Ocultar Tela Baixo\n'
        #           '3 - Mostrar/Ocultar Tela Frontal\n'
        #           '4 - Sair\n'
        #           'Informa a opcao: '
        #           )
        #     opcao = int(input())
        #     if opcao == 1:
        #         ic.mostrar_tela_frontal = not ic.mostrar_tela_frontal
        #     elif opcao == 2:
        #         ic.mostrar_tela_baixo = not ic.mostrar_tela_baixo
        #     elif opcao == 3:
        #         ic.mostrar_tela_Autonomo = not ic.mostrar_tela_Autonomo
        #     elif opcao == 4:
        #         break
        #     else:
        #         print('Opcao invalida')
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
