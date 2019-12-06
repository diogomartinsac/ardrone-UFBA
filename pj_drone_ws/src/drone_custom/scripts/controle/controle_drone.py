#!/usr/bin/env python
# coding=utf-8
import threading
import time
from asyncore import dispatcher
from docutils.nodes import status
from textwrap import dedent

# Importe as bibliotecas ROS e carregue o arquivo de manifesto que através de <depend package = ... /> nos dará acesso às dependências do projeto
import os
import sys
from enum import Enum
from ardrone_autonomy.msg import Navdata  # para receber feedback de navdata

# para pouso/decolagem/emergência. e madar msgs no ros
from PySide import QtGui
from std_msgs.msg import Empty, String
from sensor_msgs.msg import Image
from std_srvs.srv._Empty import Empty as Empty_srvs  # para abri o servico da camera
from geometry_msgs.msg import Twist  # para enviar comandos para o drone
import rospy
import roslib

from controle_drone_video import DisplayVideoDrone
from controle_status_drone import StatusDrone

roslib.load_manifest('drone_custom')

# Importe as mensagens que estamos interessados ​​em enviar e receber

# enumeração com os status do drone

# Constantes
COMMAND_PERIOD = 100  # ms


class ControleBasicoDrone(object):
    """
         Classe com os controle basicos do drone
         Esta classe implementa a funcionalidade básica de controle que usaremos em futuros tutoriais.
         Pode decolar comando / pouso / emergência, bem como movimento de drones
         Ele também rastreia o estado do drone com base no feedback dos dados de navegação
    """

    def __init__(self):
        # Mantém o status atual do drone
        self.status = -1

        # Assinando no tópico /ardrone/navdata, do tipo de mensagem navdata e chame self.receber_navdata quando uma mensagem for recebida
        self.sub_Navdata = rospy.Subscriber(
            '/ardrone/navdata', Navdata, self.receber_Navdata)

        # Permitir que o controlador publique nos tópicos /ardrone/takeoff, land e reset
        self.pub_pousar = rospy.Publisher(
            '/ardrone/land', Empty, queue_size=10)
        self.pub_decolar = rospy.Publisher(
            '/ardrone/takeoff', Empty, queue_size=10)
        self.pub_resetar = rospy.Publisher(
            '/ardrone/reset', Empty, queue_size=10)
        self.serv_trocar_camera = rospy.ServiceProxy(
            '/ardrone/togglecam', Empty_srvs)
        self.pub_camera_frente =  rospy.Publisher('/ardrone/front/image_raw', Image, queue_size=10)
        self.pub_camera_baixo =  rospy.Publisher('/ardrone/bottom/image_raw', Image, queue_size=10)

        # APermita que o controlador publique no tópico  /cmd_vel topic e assim controle o drone
        self.pub_comandar = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Configurar publicação regular de pacotes de controle
        self.comando = Twist()
        self.tempo_repeticao_pub_comando = rospy.Timer(
            rospy.Duration(COMMAND_PERIOD / 1000.0), self.comandar_drone)

        # Aterrissar o drone se estivermos desligando
        rospy.on_shutdown(self.pousar_drone)

    def receber_Navdata(self, navdata):
        # Embora haja muitos dados nesse pacote, estamos interessados ​​apenas no estado no momento
        self.status = navdata.state

    def decolar_drone(self):
        """
            Enviar uma mensagem de decolagem para o ardrone driver
        """
        # verificando se o drone esta pousado para decolar
        if self.status == StatusDrone.Aterrissado:
            self.pub_decolar.publish(Empty())
            rospy.loginfo('Decolando')
        else:
            rospy.loginfo('Drone já está em voo!')
    
    def mostrar_camera_frente(self):
        if self.status == StatusDrone.Voando:
            self.pub_camera_frente.publish(Image())
            rospy.loginfo('mostrando camera frontal')
        else:
            rospy.loginfo('Drone deve está em voo!')

    def mostrar_camera_baixo(self):
        if self.status == StatusDrone.Voando:
            self.pub_camera_baixo.publish(Image())
            rospy.loginfo('mostrando camera frontal')
        else:
            rospy.loginfo('Drone deve está em voo!')
    
    def trocar_camera(self):
        # from std_srvs.srv._Empty import Empty
        # print('teste')
        try:
            self.serv_trocar_camera()
        except rospy.ServiceException, e:
            print ("Falha ao trocar camera: %s" % e)

    def pousar_drone(self):
        """
            Enviar uma mensagem de aterrissagem para o ardrone driver
        """
        # Observe que é possivél enviar este comando para qualquer estado que do drone, o pouso pode acontecer
        # em qualquer momento
        self.pub_pousar.publish(Empty())

    def resetar_drone(self):
        """
            Enviar uma mensagem de emergência (ou reset) para o o ardrone driver
        """
        self.pub_resetar.publish(Empty())

    def alterar_comando(self, roll=0, pitch=0, yaw_velocity=0, z_velocity=0):
        """
            Preparar o comando para o drone, ou seja chama o programa principal para definir o comando atual
        :param roll:
        :param pitch:
        :param yaw_velocity:
        :param z_velocity:
        :return:
        """
        self.comando.linear.x = pitch
        self.comando.linear.y = roll
        self.comando.linear.z = z_velocity
        self.comando.angular.z = yaw_velocity

    def comandar_drone(self, event):
        """
            Enviar o comando previamente definido para o drone quando estiver voando(apenas)
        :param event:
        :return:
        """
        # O comando definido anteriormente é enviado periodicamente se o drone estiver voando
        if self.status == StatusDrone.Voando or self.status == StatusDrone.ModoFoco or self.status == StatusDrone.Girando:
            self.pub_comandar.publish(self.comando)


class ControleAutonomoDrone(ControleBasicoDrone):
    def __init__(self):
        # super(ControleAutonomoDrone, self).__init__()

        # Contruindo herança da classe
        super(ControleAutonomoDrone, self).__init__()

        # indica da distancia de metros por comando parado é 1
        self.distancia_metro_comando = 1

    def ir_para_frente(self, novo_pitch=1):
        novo_pitch *= self.distancia_metro_comando
        self.alterar_comando(roll=0, pitch=novo_pitch,
                             yaw_velocity=0, z_velocity=0)

    def ir_para_atras(self, novo_pitch=-1):
        novo_pitch *= self.distancia_metro_comando
        self.alterar_comando(roll=0, pitch=novo_pitch,
                             yaw_velocity=0, z_velocity=0)

    def ir_para_esquerda(self, novo_yaw_velocity=1):
        novo_yaw_velocity *= self.distancia_metro_comando
        self.alterar_comando(roll=0, pitch=0,
                             yaw_velocity=novo_yaw_velocity, z_velocity=0)

    def ir_para_direita(self, novo_yaw_velocity=-1):
        novo_yaw_velocity *= self.distancia_metro_comando
        self.alterar_comando(roll=0, pitch=0,
                             yaw_velocity=novo_yaw_velocity, z_velocity=0)

    def girar_esquerda(self, novo_rool=1):
        novo_rool *= self.distancia_metro_comando
        self.alterar_comando(roll=novo_rool, pitch=0,
                             yaw_velocity=0, z_velocity=0)

    def girar_direta(self, novo_rool=-1):
        novo_rool *= self.distancia_metro_comando
        self.alterar_comando(roll=novo_rool, pitch=0,
                             yaw_velocity=0, z_velocity=0)

    def subir(self, novo_z_velocity=1):
        novo_z_velocity *= self.distancia_metro_comando
        self.alterar_comando(roll=0, pitch=0, yaw_velocity=0,
                             z_velocity=novo_z_velocity)

    def descer(self, novo_z_velocity=-1):
        novo_z_velocity *= self.distancia_metro_comando
        self.alterar_comando(roll=0, pitch=0, yaw_velocity=0,
                             z_velocity=novo_z_velocity)

    def parar(self):
        self.alterar_comando(roll=0, pitch=0, yaw_velocity=0, z_velocity=0)

    def emergencia(self):
        self.resetar_drone()

    def fazer_quadrado(self, tamanho):
        self.distancia_metro_comando = tamanho
        #
        self.decolar_drone()
        #
        self.ir_para_frente()
        self.girar_direta()
        time.sleep(4)
        self.parar()
        time.sleep(2)

        #
        self.ir_para_frente()
        self.girar_direta()
        self.parar()
        time.sleep(2)
        #
        self.ir_para_frente()
        self.girar_direta()
        self.parar()
        time.sleep(2)
        #
        self.ir_para_frente()
        time.sleep(2)
        #
        self.pousar_drone()

    def fazer_triangulo(self, raio):
        print("ainda na fiz")

    def pilotar(self):
        opcao = ''
        menu_pilotar = 'Selecione as opcoes\n' \
                       'Decolar = Y Subir    = Q Emergencia     = J\n' \
                       'Pousar  = H Descer   = A Trocar Camera  = U\n' \
                       'Atrás   = D Esquerda = S Girar esquerda = W\n' \
                       'Frente  = E Direita  = F Girar direita  = R\n' \
                       'Parar   = P Sair    = 0\n' \
                       'Informe um comando: '
        while opcao != '0':
            print(menu_pilotar)
            opcao = input()
            # opcao = sys.stdin.read(1)
            if opcao.isdigit():
                str(opcao)
            opcao = opcao.upper()
            if opcao == 'Y':
                self.decolar_drone()
            elif opcao == 'Q':
                self.subir()
            elif opcao == 'J':
                self.emergencia()
            elif opcao == 'H':
                self.pousar_drone()
            elif opcao == 'A':
                self.descer()
            elif opcao == 'U':
                self.trocar_camera()
            elif opcao == 'D':
                self.ir_para_atras()
            elif opcao == 'W':
                self.ir_para_esquerda()
            elif opcao == 'S':
                self.girar_esquerda()
            elif opcao == 'E':
                self.ir_para_frente()
            elif opcao == 'R':
                self.ir_para_direita()
            elif opcao == 'F':
                self.girar_direta()
            elif opcao == 'P':
                self.parar()
            elif opcao == '0':
                self.pousar_drone()
                break
            else:
                print('Opcão inválida')
            # limpando a teka
            os.system('cls' if os.name == 'nt' else 'clear')

    def mostrar_camera_drone(self):
        app = QtGui.QApplication(sys.argv)
        display = DisplayVideoDrone()
        display.show()
        # executa o aplicativo QT
        status = app.exec_()
        return status

    def menu(self):
        menuPrincipal = 'Selececione\n' \
                        '1 - Pilotar\n' \
                        '2 - Fazer quadrado\n' \
                        '3 - Fazer circulo\n' \
                        '0 - Sair\n' \
                        'Informe um comando:'

        opcao = 1
        while opcao != 0:
            # opcao = int(input(menuPrincipal))
            print (menuPrincipal)
            opcao = int(input())
            # opcao = int(sys.stdin.read(1))
            if opcao == 1:
                self.pilotar()
            elif opcao == 2:

                tamanho = input('Informe o tamanho do quadrado em metros')
                self.fazer_quadrado(tamanho)
            elif opcao == 3:
                raio = input('Informe o raio do circulo em metros')
                self.fazer_triangulo(raio)
            elif opcao == 0:
                break
            else:
                print('Opcão inválida')


if __name__ == "__main__":
    # Primeiramente, configuramos um nó ros, para que possamos nos comunicar com os outros pacotes
    rospy.init_node('controlle_drone')
    drone_auto_piloto = ControleAutonomoDrone()
    # status = drone_auto_piloto.mostrar_camera_drone()
    # exibindo o menu
    drone_auto_piloto.menu()
    rospy.signal_shutdown('Great Voando!')
    # sys.exit(status)
