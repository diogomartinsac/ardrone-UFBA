#!/usr/bin/env python
# coding=utf-8

import sys

from controle.controle_drone_visao import ControleVisaoDrone, TipoNoVisao

def main(args):
    controlevisaodrone = ControleVisaoDrone(TipoNoVisao.NoCameraFrontal)
    controlevisaodrone.mostrar_tela_pista = True
    #controlevisaodrone.mostrar_tela_inicio_pista = True
    #controlevisaodrone.mostrar_tela_final_pista = True
    #controlevisaodrone.mostrar_tela_sensor_verde = True
    #controlevisaodrone.mostrar_tela_sensor_vermelho = True
    controlevisaodrone.inicializar_topico_ros(args)


if __name__ == "__main__":
    main(sys.argv)
