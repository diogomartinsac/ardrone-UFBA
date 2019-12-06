#!/usr/bin/env python
import sys

import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty




class FuncoesArdrone:
    def __init__(self):
        self.decola_pub = rospy.Publisher('ardrone/takeoff', Empty, queue_size=10)
        self.pousa_pub = rospy.Publisher("ardrone/land", Empty, queue_size=10)

    def decolar(self):
        var_empty = Empty()
        self.decola_pub.publish(var_empty)

    def pousar(self):
        var_empty = Empty()
        self.pousa_pub.publish(var_empty)

    def ir_para_frente(self):
        pass

    def ir_para_atras(self):
        pass

    def subir(self):
        pass

    def descer(self):
        pass

    def parar(self):
        pass


def menu():
    print ("d: Decolar")
    print ("p: pousar")


if __name__ == '__main__':
    rospy.init_node('drone_custom', anonymous=True)
    funcoes_drone = FuncoesArdrone()
    try:
        while not rospy.is_shutdown():
            menu()
            # key= input("press a key for action")
            key = sys.stdin.read(1)
            if (key == str('d')):
                funcoes_drone.decola_pub()
            elif (key == str('p')):
                funcoes_drone.pousa_pub()
    except rospy.ROSInterruptException:
        pass
