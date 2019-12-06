#!/usr/bin/env python
# coding=utf-8

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3, Twist
from std_msgs.msg import String, Empty
import cv2
import numpy as np
import sys

import rospy
import roslib

roslib.load_manifest('drone_custom')
from drone_custom.msg import DadosVisao


class TipoNoVisao(object):
    NoCameraFrontal = 1
    NoCameraBaixo = 2
    NenhumNo = 3

class TipoObjetoEncotrado:
    Pista = 1
    SensorVerde = 2
    SensorVermelho = 3
    PistaInicio = 4
    PistaFinal = 5

class DadosControleVisao:

    def __init__(self):
        self.larg_retangulo = 0
        self.alt_retangulo = 0
        self.qtde_contornos = 0
        self.coordenada_X_centro_contorno = 0
        self.coordenada_Y_centro_contorno = 0
        self.sensor_verde_encontrado = False 
        self.sensor_vermelho_encontrado = False

class ControleVisaoDrone:
    SPEED = 2
    NOME_TELA_FRONTAL = ' Camera frontal'
    NOME_TELA_BAIXO = ' Camera Baixo'
    NOME_TELA_CV_PISTA = 'Analise Objeto Pista'
    NOME_TELA_CV_INICIO_PISTA = 'Analise Objeto Inicio da Pista'
    NOME_TELA_CV_FINAL_PISTA = 'Analise Objeto Final da Pista'
    NOME_TELA_CV_SENSOR_VERDE = 'Analise Objeto Sensor Verde'
    NOME_TELA_CV_SENSOR_VERMELHO = 'Analise Objeto Sensor Vermelho'

    NOME_TOPICO_CAMERA_FRONTAL = 'controle_visao_cam_frente'
    NOME_TOPICO_CAMERA_BAIXO = 'controle_visao_cam_baixo'
    
    LimiarBinarizacao = 125  # este valor eh empirico. Ajuste-o conforme sua necessidade
    # este valor eh empirico. Ajuste-o conforme sua necessidade
    area_contorno_limite_min = 5000
    # centroideSensor = (int((largura_img/2)-15)

    def __init__(self, tipo_topico_visao):
        self.bridge = CvBridge()
        #
        self.mostrar_tela_pista = False
        self.mostrar_tela_inicio_pista = False
        self.mostrar_tela_final_pista = False
        self.mostrar_tela_sensor_verde = False
        self.mostrar_tela_sensor_vermelho = False
        self.nome_tela = 'Controle de Visao'
        self.tipo_topico_visao = tipo_topico_visao

    def inicializar_topico_ros(self, args):
        #
        if self.tipo_topico_visao == TipoNoVisao.NoCameraFrontal:
            self.nome_tela += self.NOME_TELA_FRONTAL
            #
            rospy.init_node(self.NOME_TOPICO_CAMERA_FRONTAL)
            #
            self.sub_camera = rospy.Subscriber(
                '/ardrone/front/image_raw', Image, self.callback_camera)
            self.pub_camera = rospy.Publisher(
                self.NOME_TOPICO_CAMERA_FRONTAL, DadosVisao, queue_size=10)

        elif self.tipo_topico_visao == TipoNoVisao.NoCameraBaixo:
            self.nome_tela += self.NOME_TELA_BAIXO
            #
            rospy.init_node(self.NOME_TOPICO_CAMERA_BAIXO)
            #
            self.sub_camera = rospy.Subscriber(
                '/ardrone/bottom/image_raw', Image, self.callback_camera)
            self.pub_camera = rospy.Publisher(
                self.NOME_TOPICO_CAMERA_BAIXO, DadosVisao, queue_size=10)

        elif self.tipo_topico_visao == TipoNoVisao.NenhumNo:
            pass
        #
        #
        rospy.spin()
        #
        cv2.destroyAllWindows()

    def callback_camera(self, data):

        try:
            # cv_imagem = CvBridge().imgmsg_to_cv2(data, "bgr8")
            cv_imagem_pista           = CvBridge().imgmsg_to_cv2(data, "bgr8")
            cv_imagem_pista_inicio    = CvBridge().imgmsg_to_cv2(data, "bgr8")
            cv_imagem_pista_final     = CvBridge().imgmsg_to_cv2(data, "bgr8")
            cv_imagem_sensor_verde    = CvBridge().imgmsg_to_cv2(data, "bgr8")
            cv_imagem_sensor_vermelho = CvBridge().imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            cv_imagem = None

        dados_controle_visao_pista = self.encontrar_pista_imagem(cv_imagem_pista)
        
        dados_controle_visao_pista_inicio = self.encontrar_pista_inicio(cv_imagem_pista_inicio)

        dados_controle_visao_pista_final = self.encontrar_pista_final(cv_imagem_pista_final)

        dados_controle_visao_sensor_verde = self.encontrar_sensor_verde_imagem(cv_imagem_sensor_verde)

        dados_controle_visao_sensor_vermelho = self.encontrar_sensor_vermelho_imagem(cv_imagem_sensor_vermelho)

        try:
            if dados_controle_visao_pista is not None:
                retorno_calculo_ros = DadosVisao()

                # retorno_calculo_ros.direcao_a_ser_tomada = dados_controle_visao_pista.direcao_a_ser_tomada
                # retorno_calculo_ros.qtde_contornos = dados_controle_visao_pista.qtde_contornos
                # retorno_calculo_ros.coordenada_X_centroContorno = dados_controle_visao_pista.coordenada_X_centro_contorno
                # retorno_calculo_ros.coordenada_Y_centroContorno = dados_controle_visao_pista.coordenada_Y_centro_contorno
                # retorno_calculo_ros.larg_retangulo = dados_controle_visao_pista.larg_retangulo
                # retorno_calculo_ros.alt_retangulo = dados_controle_visao_pista.alt_retangulo
                # retorno_calculo_ros.sensor_verde_encontrado = dados_controle_visao_sensor_verde.sensor_verde_encontrado
                # retorno_calculo_ros.sensor_vermelho_encontrado = dados_controle_visao_sensor_vermelho.sensor_vermelho_encontrado

                retorno_calculo_ros.larg_retangulo_pista = dados_controle_visao_pista.larg_retangulo
                retorno_calculo_ros.alt_retangulo_pista = dados_controle_visao_pista.alt_retangulo

                retorno_calculo_ros.qtde_contornos_pista = dados_controle_visao_pista.qtde_contornos
                retorno_calculo_ros.coordenada_X_centro_contorno_pista = dados_controle_visao_pista.coordenada_X_centro_contorno
                retorno_calculo_ros.coordenada_Y_centro_contorno_pista = dados_controle_visao_pista.coordenada_Y_centro_contorno
               
                retorno_calculo_ros.qtde_contornos_pista_inicio = dados_controle_visao_pista_inicio.qtde_contornos
                retorno_calculo_ros.coordenada_X_centro_contorno_pista_inicio = dados_controle_visao_pista_inicio.coordenada_X_centro_contorno
                retorno_calculo_ros.coordenada_Y_centro_contorno_pista_inicio = dados_controle_visao_pista_inicio.coordenada_Y_centro_contorno
               
                retorno_calculo_ros.qtde_contornos_pista_final = dados_controle_visao_pista_final.qtde_contornos
                retorno_calculo_ros.coordenada_X_centro_contorno_pista_final = dados_controle_visao_pista_final.coordenada_X_centro_contorno
                retorno_calculo_ros.coordenada_Y_centro_contorno_pista_final = dados_controle_visao_pista_final.coordenada_Y_centro_contorno
               
                retorno_calculo_ros.sensor_verde_encontrado = dados_controle_visao_sensor_verde.sensor_verde_encontrado
                retorno_calculo_ros.sensor_vermelho_encontrado = dados_controle_visao_sensor_vermelho.sensor_vermelho_encontrado
                
                #ATENTE-SE AO RANGER DAS CORES POIS ESTA PEGANDO TUDO e a cor real da pista
                self.pub_camera.publish(retorno_calculo_ros)
        except CvBridgeError as e:
            print(e)

    def encontrar_objeto_imagem(self, imagem_tratamento, limite_max_BGR,
                                limite_min_BGR, tipo_objeto_encontrado,
                                mostrar_tela_analise_rota=False, titulo_tela_analise_rota='Analise de rota'):
        """
            Funcao: trata imagem e retorna se o robo seguidor de linha deve ir para a esqueda ou direita
            Parametros: frame capturado da webcam e primeiro frame capturado
            Retorno: < 0: robo deve ir para a direita
                    > 0: robo deve ir para a esquerda
                    0:   nada deve ser feito
        """
        # obtencao das dimensoes da imagem
        altura_img = np.size(imagem_tratamento, 0)
        largura_img = np.size(imagem_tratamento, 1)
        qtde_contornos = 0
        #direcao_a_ser_tomada = 0
        coordenada_X_centro_contorno = 0
        coordenada_Y_centro_contorno = 0
        coord_vertice_X = coord_vertice_Y = alt_retangulo = larg_retangulo = 0
        sensor_verde_encontrado = sensor_vermelho_encontrado = False

        # tratamento da imagem
        # Here we are defining range of bluecolor in HSV
        # This creates a mask of blue coloured
        # objects found in the frame.
        mascara_busca = cv2.inRange(imagem_tratamento, limite_min_BGR, limite_max_BGR)
        # The bitwise and of the frame and mask is done so
        # that only the blue coloured objects are highlighted
        # and stored in res
        kernel = np.ones((5, 5), np.uint8)
        frame_binarizado = cv2.morphologyEx(
            mascara_busca, cv2.MORPH_OPEN, kernel)

        # descomente as linhas abaixo se quiser ver o frame apos binarizacao, dilatacao e inversao de cores
        # cv2.imshow('F.B.',mask)
        # cv2.waitKey(10)

        _, contornos_encontrados, _ = cv2.findContours(
            frame_binarizado.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(imagem_tratamento,
                         contornos_encontrados, -1, (255, 0, 255), 3)

        for contorno in contornos_encontrados:
            # se a area do contorno capturado for pequena, nada acontece
            if cv2.contourArea(contorno) < self.area_contorno_limite_min:
                continue

            qtde_contornos = qtde_contornos + 1

            # obtem coordenadas do contorno (na verdade, de um retangulo que consegue abrangir todo ocontorno) e
            # realca o contorno com um retangulo.

            # x e y: coordenadas do vertice superior esquerdo
            # w e h: respectivamente largura e altura do retangulo
            (coord_vertice_X, coord_vertice_Y, alt_retangulo, larg_retangulo) = cv2.boundingRect(contorno)

            cv2.rectangle(imagem_tratamento, (coord_vertice_X, coord_vertice_Y),
                          (coord_vertice_X + alt_retangulo, coord_vertice_Y + larg_retangulo),
                          (0, 255, 0), 2)

            # determina o ponto central do contorno e desenha um circulo para indicar
            coordenada_X_centro_contorno = int((coord_vertice_X + coord_vertice_X + alt_retangulo) / 2)
            coordenada_Y_centro_contorno = int((coord_vertice_Y + coord_vertice_Y + larg_retangulo) / 2)
            ponto_central_contorno = (coordenada_X_centro_contorno, coordenada_Y_centro_contorno)
            cv2.circle(imagem_tratamento, ponto_central_contorno, 1, (0, 0, 0), 5)
            #direcao_a_ser_tomada = coordenada_X_centro_contorno - int(largura_img / 2)  # em relacao a linha central

        # output da imagem
        # linha em azul: linha central / referencia
        # linha em verde: linha que mostra distancia entre linha e a referencia

        if (qtde_contornos > 0):
            cv2.line(imagem_tratamento, ponto_central_contorno, (int(
                largura_img / 2), coordenada_Y_centro_contorno), (0, 255, 0), 1)

        if (tipo_objeto_encontrado == TipoObjetoEncotrado.SensorVerde or tipo_objeto_encontrado == TipoObjetoEncotrado.SensorVermelho or tipo_objeto_encontrado == TipoObjetoEncotrado.PistaInicio or tipo_objeto_encontrado == TipoObjetoEncotrado.PistaFinal):
            
            cv2.rectangle(imagem_tratamento, (int((largura_img/2)-50), int((altura_img/2) +50)), (int((largura_img/2) +50), int((altura_img/2) -50)), (255,0,0), 2)

            if (int((largura_img/2)-50) <= coordenada_X_centro_contorno) and (coordenada_X_centro_contorno <= int((largura_img/2)+50)):
                if (int((altura_img/2)-50) <= coordenada_Y_centro_contorno) and (coordenada_Y_centro_contorno <= int((altura_img/2)+50)):
                    if tipo_objeto_encontrado == TipoObjetoEncotrado.SensorVerde:
                        # print("Sensor OK!")
                        rospy.loginfo("Funcionamento Normal!")
                        sensor_verde_encontrado = True
                    else:
                        # print("Sensor NOK!")
                        rospy.loginfo("Falha no sensor")
                        sensor_vermelho_encontrado = True
        else:
            cv2.line(imagem_tratamento, (int(largura_img / 2), 0),
                            (int(largura_img / 2), altura_img), (255, 0, 0), 2)

        if mostrar_tela_analise_rota is True:
            cv2.imshow(titulo_tela_analise_rota, imagem_tratamento)
            cv2.waitKey(10)

        dados_controle_visao = DadosControleVisao()

        dados_controle_visao.larg_retangulo = larg_retangulo
        dados_controle_visao.alt_retangulo = alt_retangulo
        dados_controle_visao.qtde_contornos = qtde_contornos
        dados_controle_visao.coordenada_X_centro_contorno = coordenada_X_centro_contorno
        dados_controle_visao.coordenada_Y_centro_contorno = coordenada_Y_centro_contorno
        dados_controle_visao.sensor_verde_encontrado = sensor_verde_encontrado
        dados_controle_visao.sensor_vermelho_encontrado = sensor_vermelho_encontrado
        
        return dados_controle_visao
        # return direcao_a_ser_tomada, qtde_contornos, coordenada_X_centro_contorno, coordenada_Y_centro_contorno, larg_retangulo, alt_retangulo

    def encontrar_pista_imagem(self, imagem_tratamento):

        # Converts images from BGR to HSV
        limite_max_BGR_cor_pista = np.array([255, 150, 90])  # B, G, R
        limite_min_BGR_cor_pista = np.array([90, 0, 0])

        return self.encontrar_objeto_imagem(imagem_tratamento, 
                                            limite_max_BGR_cor_pista, limite_min_BGR_cor_pista,
                                            tipo_objeto_encontrado = TipoObjetoEncotrado.Pista,
                                            mostrar_tela_analise_rota=self.mostrar_tela_pista,
                                            titulo_tela_analise_rota=self.NOME_TELA_CV_PISTA)
    
    def encontrar_pista_inicio(self, imagem_tratamento):

        # Converts images from BGR to HSV
        limite_max_BGR_cor_pista_inicio = np.array([64, 228, 255])  # B, G, R np.array([40, 40, 50])
        limite_min_BGR_cor_pista_inicio = np.array([0, 95, 64]) #np.array([0, 0, 10])

        return self.encontrar_objeto_imagem(imagem_tratamento, 
                                            limite_max_BGR_cor_pista_inicio, limite_min_BGR_cor_pista_inicio,
                                            tipo_objeto_encontrado = TipoObjetoEncotrado.PistaInicio,
                                            mostrar_tela_analise_rota=self.mostrar_tela_inicio_pista,
                                            titulo_tela_analise_rota=self.NOME_TELA_CV_INICIO_PISTA)

    def encontrar_pista_final(self, imagem_tratamento):

        # Converts images from BGR to HSV
        limite_max_BGR_cor_pista_final = np.array([202, 0, 255])  # B, G, R np.array([190, 105, 170])
        limite_min_BGR_cor_pista_final = np.array([90, 0, 105]) #np.array([115, 50, 95])

        return self.encontrar_objeto_imagem(imagem_tratamento, 
                                            limite_max_BGR_cor_pista_final, limite_min_BGR_cor_pista_final,
                                            tipo_objeto_encontrado = TipoObjetoEncotrado.PistaFinal,
                                            mostrar_tela_analise_rota=self.mostrar_tela_final_pista,
                                            titulo_tela_analise_rota=self.NOME_TELA_CV_FINAL_PISTA)

    def encontrar_sensor_verde_imagem(self, imagem_tratamento):

        # Converts images from BGR to HSV
        limite_max_BGR_cor_sensor_verde = np.array([95, 255, 120])# B, G, R
        limite_min_BGR_cor_sensor_verde = np.array([0, 80, 0])

        return self.encontrar_objeto_imagem(imagem_tratamento, 
                                            limite_max_BGR_cor_sensor_verde, limite_min_BGR_cor_sensor_verde,
                                            tipo_objeto_encontrado = TipoObjetoEncotrado.SensorVerde,
                                            mostrar_tela_analise_rota=self.mostrar_tela_sensor_verde,
                                            titulo_tela_analise_rota=self.NOME_TELA_CV_SENSOR_VERDE)

    def encontrar_sensor_vermelho_imagem(self, imagem_tratamento):

        # Converts images from BGR to HSV

        limite_max_BGR_cor_sensor_vermelho = np.array([90, 110, 255])# B, G, R
        limite_min_BGR_cor_sensor_vermelho = np.array([0, 0, 90])

        return self.encontrar_objeto_imagem(imagem_tratamento, 
                                            limite_max_BGR_cor_sensor_vermelho, limite_min_BGR_cor_sensor_vermelho,
                                            tipo_objeto_encontrado = TipoObjetoEncotrado.SensorVermelho,
                                            mostrar_tela_analise_rota=self.mostrar_tela_sensor_vermelho,
                                            titulo_tela_analise_rota=self.NOME_TELA_CV_SENSOR_VERMELHO)


class TopicoCameraFrontal(ControleVisaoDrone):
    def __init__(self):
        super(TopicoCameraFrontal, self).__init__(
            TipoNoVisao.NoCameraFrontal)


class TopicoCameraBaixo(ControleVisaoDrone):
    def __init__(self, tipo_topico_visao):
        super(TopicoCameraBaixo, self).__init__(TipoNoVisao.NoCameraBaixo)
