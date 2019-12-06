#!/usr/bin/env python
# coding=utf-8

# Importando as bibliotecas ROS e carregue o arquivo de manifesto que através de <depend package = ... /> nos dará
# acesso às dependências do projeto
from enum import Enum
import roslib; 
roslib.load_manifest('drone_custom')

import rospy

# Importando a classe ControleBasicoDrone para lidar com as  interações com o drone e a classe DisplayVideoDrone que
# lida com a exibição de vídeo
from controle_drone import ControleBasicoDrone
from controle_drone_video import DisplayVideoDrone, USAR_TELA_OPENCV

# Bibliotecas da GUI
from PySide import QtCore, QtGui



class MapaTeclas:
    """
        Classe que representa o mapa de teclas que serão utilizadas do teclado para controlar os drones nos voos
    """
    PitchForward = QtCore.Qt.Key.Key_E
    PitchBackward = QtCore.Qt.Key.Key_D
    RollLeft = QtCore.Qt.Key.Key_S
    RollRight = QtCore.Qt.Key.Key_F
    YawLeft = QtCore.Qt.Key.Key_W
    YawRight = QtCore.Qt.Key.Key_R
    IncreaseAltitude = QtCore.Qt.Key.Key_Q
    DecreaseAltitude = QtCore.Qt.Key.Key_A
    Takeoff = QtCore.Qt.Key.Key_Y
    Land = QtCore.Qt.Key.Key_H
    Emergency = QtCore.Qt.Key.Key_Space
    TrocarCamera = (ord('u'), ord('U') )
    AutoPiloto = (ord('a'), ord('A') )


class ControleDroneTeclado(DisplayVideoDrone):
    """
        O nó controlador do drone por teclado
        Este controlador especializa(herda) da classe DroneVideoDisplay, adicionando um manipulador de teclas para
        ativar o controle do drone pelo teclado
    """

    def __init__(self):
        super(ControleDroneTeclado, self).__init__()

        self.pitch = 0
        self.roll = 0
        self.yaw_velocity = 0
        self.z_velocity = 0

    def keyPressEvent(self, event):
        """
         #Metodo para capturar(ou reagir) o evento ao pressionar uma tecla no teclado na tela do DroneVideoDisplay(lembre-se essa classe herda dela)
         OBS:. O evento keyPressEvent é um metodo da classe QWidget que veio de herança
        :param event:Evento
        """
        key = event.key()

        # Se construímos o controlador drone e a tecla não é gerada a partir de uma tecla de repetição automática
        # If we have constructed the drone drone and the key is not generated from an auto-repeating key
        if drone is not None and not event.isAutoRepeat():
            # Lide com os casos importantes primeiro!
            # Handle the important cases first!
            if key == MapaTeclas.Emergency:
                drone.resetar_drone()
            elif key == MapaTeclas.Takeoff:
                drone.decolar_drone()
            elif key == MapaTeclas.Land:
                drone.pousar_drone()
            elif key in MapaTeclas.TrocarCamera:
                drone.trocar_camera()
            else:
                # Agora lidamos com a movimentação, observe que neste método é o oposto (+=) da método
                # keyReleaseEvent(liberação da tecla)
                if key == MapaTeclas.YawLeft:
                    self.yaw_velocity += 1
                elif key == MapaTeclas.YawRight:
                    self.yaw_velocity += -1

                elif key == MapaTeclas.PitchForward:
                    self.pitch += 1
                elif key == MapaTeclas.PitchBackward:
                    self.pitch += -1

                elif key == MapaTeclas.RollLeft:
                    self.roll += 1
                elif key == MapaTeclas.RollRight:
                    self.roll += -1

                elif key == MapaTeclas.IncreaseAltitude:
                    self.z_velocity += 1
                elif key == MapaTeclas.DecreaseAltitude:
                    self.z_velocity += -1

            # Finalmente definimos o comando a ser enviado.Note que a classe ControleBasicoDrone lida com o envio de
            # comandos em intervalos regulares(veja a linha self.tempo_repeticao_pub_comando ... na classe)
            drone.alterar_comando(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)

    def keyReleaseEvent(self, event):
        """
         #Metodo para capturar(ou reagir) o evento ao liberar uma tecla no teclado na tela do DroneVideoDisplay(lembre-se essa classe herda dela)
         OBS:. O evento keyReleaseEvent é um metodo da classe QWidget que veio de herança
        :param event: evento
        """
        key = event.key()

        # Se construímos o controlador drone e a tecla não é gerada a partir de uma tecla de repetição automática
        if drone is not None and not event.isAutoRepeat():
            # Observe que não lidamos com o lançamento de chaves de emergência/decolagem/pouso aqui,
            # não há necessidade. Agora lidamos com a movimentação, observe que neste método é o oposto (+=) da
            # método keypress (pressionamento da tecla) Agora lidamos com o movimento, observe que esta seção é o
            if key == MapaTeclas.YawLeft:
                self.yaw_velocity -= 1
            elif key == MapaTeclas.YawRight:
                self.yaw_velocity -= -1

            elif key == MapaTeclas.PitchForward:
                self.pitch -= 1
            elif key == MapaTeclas.PitchBackward:
                self.pitch -= -1

            elif key == MapaTeclas.RollLeft:
                self.roll -= 1
            elif key == MapaTeclas.RollRight:
                self.roll -= -1

            elif key == MapaTeclas.IncreaseAltitude:
                self.z_velocity -= 1
            elif key == MapaTeclas.DecreaseAltitude:
                self.z_velocity -= -1

            # Finalmente definimos o comando a ser enviado.Note que a classe ControleBasicoDrone lida com o envio de
            # comandos em intervalos regulares(veja a linha self.tempo_repeticao_pub_comando ... na classe)
            drone.alterar_comando(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
    
    def gerar_tela_qt(self):
        super(ControleDroneTeclado, self).gerar_tela_qt()        
        # novo_status_com_menu =  self.mensagem_status+ self.menu_teclado
        # self.statusBar().showMessage(novo_status_com_menu) 


    @property
    def menu_teclado(self):
      return    ('Decolar = Y Subir    = Q Emergencia     = Spaço\nPousar  = H Descer   = A Trocar Camera  = U\nAtrás   = D Esquerda = W Girar esquerda = S\nFrente  = E Direita  = R Girar direita  = F\nAuto Piloto    = A')


# Configurando o aplicativo
if __name__ == '__main__':
    import sys
   
    # Primeiramente, configuramos um nó ros, para que possamos nos comunicar com os outros pacotes
    rospy.init_node('drone_controlle_teclado')

    # Agora construímos nosso aplicativo QT(da GUI) e controladores e janelas associados
    app = QtGui.QApplication(sys.argv)
    drone = ControleBasicoDrone()
    display = ControleDroneTeclado()
    #descomentar essa linha para visualizar a tela de analise de rotas
    # display.mostrar_tela_analise_rota =True

    display.show()

    # executa o aplicativo QT
    status = app.exec_()

    rospy.loginfo(display.menu_teclado)

    # e só progride para aqui depois que o aplicativo for encerrado
    rospy.signal_shutdown('Great Voando!')
    sys.exit(status)
   
