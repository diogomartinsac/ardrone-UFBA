#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist


class Drone:
    def __init__(self):
        self.pub_locomover = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.pub_decolar = rospy.Publisher("ardrone/takeoff", Empty, queue_size=10)
        self.pub_pousar = rospy.Publisher("ardrone/land", Empty, queue_size=10)

    def decolar(self):
        self.pub_decolar.publish(Empty())

    def pousar(self):
        self.pub_pousar.publish(Empty())

    def ir_para_frente(self):
        var_Twist = Twist()
        # linear
        var_Twist.linear.x = 0
        var_Twist.linear.y = 0
        var_Twist.linear.z = 1
        # angular
        var_Twist.angular.x = 0
        var_Twist.angular.y = 0
        var_Twist.angular.z = 0
        self.pub_locomover.publish(var_Twist)

    def ir_para_atras(self):
        pass

    def subir(self):
        pass

    def descer(self):
        pass

    def parar(self):
        pass

    def menu(self):
        print ("d: Decolar")
        print ("p: pousar")
        print ("f: frente")
        print ("0: Sair")


if __name__ == '__main__':
    rospy.init_node('drone', anonymous=True)
    funcoes_drone = Drone()
    # rate = rospy.Rate(10) # 10hz
    try:
        while not rospy.is_shutdown():
            funcoes_drone.menu()
            # key= input("press a key for action")
            key = sys.stdin.read(1)
            if (key == str('d')):
                funcoes_drone.decolar()
            elif (key == str('p')):
                funcoes_drone.pousar()
            elif (key == str('f')):
                funcoes_drone.ir_para_frente()
            elif (key == str('0')):
                exit(0)
    except rospy.ROSInterruptException:
        pass
