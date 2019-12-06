#!/usr/bin/env python
# coding=utf-8


# Importando as bibliotecas ROS e carregue o arquivo de manifesto que através de <depend package = ... /> nos dará
# acesso às dependências do projeto
import roslib;

roslib.load_manifest('drone_custom')
import rospy

# Importando as mensagens que estamos interessados ​​em enviar e receber
from sensor_msgs.msg import Image  # para receber o feed de vídeo
from ardrone_autonomy.msg import Navdata  # para receber feedback de navdata

# É necessario usar o bloqueio de recursos para lidar com a sincronização entre o thread da GUI e os retornos de chamada
# de tópico do ROS
from threading import Lock

# enumeração com os status do drone
from controle_status_drone import StatusDrone
from controle_drone_visao import ControleVisaoDrone, TipoNoVisao

# Bibliotecas da GUI
from PySide import QtCore, QtGui
from cv_bridge import CvBridge, CvBridgeError
import cv2

# Constantes
PERIODO_CHECAGEM_CONEXAO = 250  # ms
PERIODO_ATUALIZACAO_GUI = 20  # ms
RAIO_CIRCULO = 4  # o raio do círculo desenhado quando uma etiqueta é detectada
# constantes
USAR_TELA_OPENCV = False

# classe DisplayVideoDrone

class DisplayVideoDrone(QtGui.QMainWindow):
    """
        Essa classe tem como objetivo exibir uma janela básica de exibição de vídeo
        Esta janela de exibição escuta os feeds de vídeo do drone e atualiza a exibição em intervalos regulares
        Ele também rastreia o status do drone e quaisquer problemas de conexão, exibindo-os na barra de status da janela
        Por padrão, ele não inclui funcionalidade de controle. A classe pode ser estendida para implementar ouvintes de chave ou mouse, se necessário
        Importe as bibliotecas ROS e carregue o arquivo de manifesto que através de <depend package = ... /> nos dará acesso às dependências do projeto
    """
    StatusMensagens = {
        StatusDrone.Emergencia: 'Emergência',
        StatusDrone.Inicializado: 'Initializado',
        StatusDrone.Aterrissado: 'Aterrissado',
        StatusDrone.Voando: 'Voando',
        StatusDrone.Girando: 'Girando',
        StatusDrone.Teste: 'Teste (?)',
        StatusDrone.Decolando: 'Decolando',
        StatusDrone.ModoFoco: 'Indo para Modo Foco',
        StatusDrone.Pousando: 'Pousando',
        StatusDrone.Looping: 'Looping (?)'
    }
    MessagemDesconectada = 'Desconectado'
    MensagemDesconhecida = 'Status desconhecido'
    def __init__(self):
        # Contruindo herança da classe
        super(DisplayVideoDrone, self).__init__()
        
        self.mostrar_tela_analise_rota = False
        # self.usar_tela_openCV = usar_tela_openCV

        # Assinando o tópico /ardrone/navdata, do tipo de mensagem navdata e chamando self.receber_Navdata quando uma mensagem for recebida
        self.sub_Navdata = rospy.Subscriber('/ardrone/navdata', Navdata, self.receber_navdata, queue_size=10)

        # Assinando o feed de vídeo do drone, chamando self.ReceiveImage quando um novo quadro(frame) é recebido
        self.sub_Video = rospy.Subscriber('/ardrone/image_raw', Image, self.receber_imagem,queue_size=10)

        # Mantém o quadro de imagem recebido do drone e posteriormente processado pela GUI
        self.imagem = None
        self.imagem_bloqueada = Lock()

        self.tags = []
        self.tagLock = Lock()

        # Contém a mensagem de status a ser exibida na próxima atualização da GUI
        self.mensagem_status = ''

        # Rastreia se recebemos dados desde a última verificação de conexão
        # Isso funciona porque os dados chegam a 50Hz, mas estamos verificando uma conexão em 4H
        self.communicationSinceTimer = False
        self.drone_conectado = False
        
        self.usar_tela_openCV=False
        self.bridge = CvBridge()

        self.titulo_tela = 'AR.Drone Video Controle Teclado'
        # Configurando nossa GUI muito básica - um rótulo que preenche toda a janela e mantém nossa imagem
        if self.usar_tela_openCV is False:

            self.setWindowTitle(self.titulo_tela)
            self.imageBox = QtGui.QLabel(self)
            self.setCentralWidget(self.imageBox)     

            # Cronômetro para verificar se ainda estamos conectados
            self.tempo_conexao_qt = QtCore.QTimer(self)
            self.tempo_conexao_qt.timeout.connect(self.ConnectionCallback)
            self.tempo_conexao_qt.start(PERIODO_CHECAGEM_CONEXAO)
            
            # Um cronômetro para redesenhar a GUI
            self.tempo_redesenhar_tela = QtCore.QTimer(self)
            self.tempo_redesenhar_tela.timeout.connect(self.gerar_tela_qt)
            self.tempo_redesenhar_tela.start(PERIODO_ATUALIZACAO_GUI)
        else:
            # A tempo para to redraw the GUI 
            #isso não esta  funcionando precisa trabalhar
            self.connectionTimer = rospy.Timer(rospy.Duration(PERIODO_CHECAGEM_CONEXAO),self.ConnectionCallback)      
            self.redrawTimer = rospy.Timer(rospy.Duration(PERIODO_ATUALIZACAO_GUI),self.gerar_tela_opencv)
            cv2.namedWindow(self.titulo_tela,cv2.WINDOW_AUTOSIZE)

    def ConnectionCallback(self):
        # Chama a cada CONNECTION_CHECK_PERIOD ms, para verificar se não recebeu nada desde o último retorno de chamada,
        #    assumime-se que esta com problemas de rede e exibe uma mensagem na barra de status 
        self.drone_conectado = self.communicationSinceTimer
        self.communicationSinceTimer = False	

    def gerar_tela_opencv(self):
        if self.image is not None:
            # We have some issues with locking between the display thread and the ros messaging thread due to the size of the image, so we need to lock the resources
            self.imageLock.acquire()
            try:
                # Todo: esta dando pau aqui
                image_cv = ToOpenCV(self.image)
            finally:
                self.imageLock.release()

            print ("showing image")
            cv2.ShowImage("windowimage", image_cv)
            cv2.WaitKey(2)
            # cv2.destroyAllWindows()
        # Update the status bar to show the current drone status & battery level
        # print (self.statusMessage if self.connected else self.DisconnectedMessage)

    def gerar_tela_qt(self):
        """

        """
        if self.imagem is not None:
            # Temos alguns problemas com o bloqueio entre o encadeamento da tela e o encadeamento de mensagens do
            # ROS devido ao tamanho da imagem, por isso precisamos bloquear os recursos
            image_atuaL =  None
            self.imagem_bloqueada.acquire()
            try:
                # Convertendo a imagem do ROS em uma QImage para exibir
                image_atuaL = QtGui.QPixmap.fromImage(
                    QtGui.QImage(self.imagem.data, self.imagem.width, self.imagem.height, QtGui.QImage.Format_RGB888))
                if len(self.tags) > 0:
                    self.tagLock.acquire()
                    try:
                        quadro_tela_atual = QtGui.QPainter()
                        quadro_tela_atual.begin(image_atuaL)
                        quadro_tela_atual.setPen(QtGui.QColor(0, 255, 0))
                        quadro_tela_atual.setBrush(QtGui.QColor(0, 255, 0))
                        for (x, y, d) in self.tags:
                            r = QtCore.QRectF((x * image_atuaL.width()) / 1000 - RAIO_CIRCULO,
                                              (y * image_atuaL.height()) / 1000 - RAIO_CIRCULO, RAIO_CIRCULO * 2,
                                              RAIO_CIRCULO * 2)
                            quadro_tela_atual.drawEllipse(r)
                            quadro_tela_atual.drawText((x * image_atuaL.width()) / 1000 + RAIO_CIRCULO,
                                                       (y * image_atuaL.height()) / 1000 - RAIO_CIRCULO,
                                                       str(d / 100)[0:4] + 'm')
                        quadro_tela_atual.end()
                    finally:
                        self.tagLock.release()
            finally:
                self.imagem_bloqueada.release()

            # Colocar aqui o processamento (do OpenCV)
            imagem_CV_pista = self.ToOpenCV(self.imagem)
            imagem_CV_sensor_verde = self.ToOpenCV(self.imagem)
            imagem_CV_sensor_vermelho = self.ToOpenCV(self.imagem)

            if self.mostrar_tela_analise_rota is True:   
                controleDroneVisao = ControleVisaoDrone(TipoNoVisao.NenhumNo)
                controleDroneVisao.mostrar_tela_pista = self.mostrar_tela_analise_rota
                controleDroneVisao.mostrar_tela_sensor_verde = self.mostrar_tela_analise_rota
                controleDroneVisao.mostrar_tela_sensor_vermelho = self.mostrar_tela_analise_rota
                controleDroneVisao.encontrar_pista_imagem(imagem_CV_pista)
                controleDroneVisao.encontrar_sensor_verde_imagem(imagem_CV_sensor_verde)
                controleDroneVisao.encontrar_sensor_vermelho_imagem(imagem_CV_sensor_vermelho)
                           
            # apenas exibir a janela.
            self.resize(image_atuaL.width(), image_atuaL.height())
            self.imageBox.setPixmap(image_atuaL)

        # Atualizar a barra de status para mostrar o status atual do drone e o nível da bateria
        self.statusBar().showMessage(self.mensagem_status if self.drone_conectado else self.MessagemDesconectada)

    def receber_imagem(self, data):
        # Indica que novos dados foram recebidos (por isso estamos conectados)
        self.communicationSinceTimer = True

        # Temos alguns problemas com o bloqueio entre o encadeamento da tela e o encadeamento de mensagens do
        # ROS devido ao tamanho da imagem, por isso precisamos bloquear os recursos
        self.imagem_bloqueada.acquire()
        try:
            self.imagem = data  # Salve a imagem ros para processamento pelo thread de exibição
        finally:
            self.imagem_bloqueada.release()

    def receber_navdata(self, navdata):
        # Indique que novos dados foram recebidos (por isso estamos conectados)
        # Indicate that new data has been received (thus we are drone_conectado)
        self.communicationSinceTimer = True

        # Atualizando a mensagem a ser exibida
        msg = self.StatusMensagens[
            navdata.state] if navdata.state in self.StatusMensagens else self.MensagemDesconhecida
        self.mensagem_status = '{} (Bateria: {}%)'.format(msg, int(navdata.batteryPercent))

        try:
            self.tagLock.acquire()
            if navdata.tags_count > 0:
                self.tags = [(navdata.tags_xc[i], navdata.tags_yc[i], navdata.tags_distance[i]) for i in
                             range(0, navdata.tags_count)]
            else:
                self.tags = []
            self.tagLock.release()
        except:
            # print ('FALHA:DisplayVideoDrone object has no attribute tagLock')
            pass
        # finally:
        #     self.tagLock.release()

    def ToOpenCV(self, ros_image):
        try:
            # cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            cv_image = CvBridge().imgmsg_to_cv2(ros_image, "bgr8")
            return cv_image
        except CvBridgeError, e:
            print (e)
            raise Exception("Falha ao converter para OpenCV imagem")

    def ToRos(self, cv_image):
        try:
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, desired_encoding="passthrough")
            return ros_image
        except CvBridgeError, e:
            print (e)
            raise Exception("Falha ao converter para ROS imagem")
 
def main(args):
    rospy.init_node('ardrone_video_display')
    app = QtGui.QApplication(args)
    display = DisplayVideoDrone()
    display.show()
    status = app.exec_()
    rospy.signal_shutdown('Great Voando!')
    cv2.destroyAllWindows()
    sys.exit(status)

if __name__ == '__main__':
    import sys
    main(sys.argv)
