#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HelloWorldSubscriber(Node):
    def __init__(self):
        super().__init__("hello_world_sub_node")
        self.sub = self.create_subscription(String,"hello_world",self.subscriber_callback,10)
    def subscriber_callback(self, msg):
        print("Recieved: "+ msg.data) 


#La fucion principal de main
def main(args = None):
    #inicializa el contexto de ROS2
    rclpy.init()
    #crea una instancia del "HelloWorldPublisher"
    my_sub = HelloWorldSubscriber()
    print("Waiting for data to be published...")

    #Espera a que el nodo gire, si detecta una interrupcion del teclado, se detruye 
    #el nodo
    try:
        rclpy.spin(my_sub)
    except KeyboardInterrupt:
        print("Terminating Node...")
        my_sub.destroy_node()

#Se verifica si el script esta siendo ejecutado directamente y en ese caso 
#se llama a la funcion "main"
if __name__ == '__main__':
    main()