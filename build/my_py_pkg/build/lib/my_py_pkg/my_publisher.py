#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import std_msgs.msg
import time

class StaticPointCloudPublisher(Node):
    def __init__(self):
        super().__init__('static_point_cloud_publisher')
        self.publisher_ = self.create_publisher(PointCloud2, '/front_camera/points', 10)
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.publish_static_point_cloud)

    def publish_static_point_cloud(self):
        # Crear un header
        header = std_msgs.msg.Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "front_camera_link_optical"

        # Definir los puntos de la nube de puntos
        points = [(0.0, 0.0, 0.0)]  # Un solo punto en el origen

        # Crear un PointCloud2 message
        cloud = point_cloud2.create_cloud_xyz32(header, points)

        # Publicar el mensaje
        self.publisher_.publish(cloud)
        self.get_logger().info('Publicando un punto estático en "/front_camera/points"')

def main(args=None):
    rclpy.init(args=args)
    static_point_cloud_publisher = StaticPointCloudPublisher()
    rclpy.spin(static_point_cloud_publisher)

    # Destrucción del nodo
    static_point_cloud_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




















































































# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String



# #se crea la clase llamada HelloWorldPublisher que hereda de la clase Nodo
# class HelloWorldPublisher(Node):
#     #Constructor de la clase nodo
#     def __init__(self):

#         #El constructor de la clase llama al constructor de la base de la clase 
#         #base "super()."y se inicializa el nodo con el nombre: hello_world_pub_node
#         super().__init__("hello_world_pub_node")

#         #secrea un publicador que publique mensajes del tipo String en el tema o topic
#         #"Hello_world" con una cola de tamaño de 10
#         self.pub = self.create_publisher(String,"hello_world",10)

#         #Se crea un temporizador que llama a la funcion "Publish_hello_world" cada 
#         #0.5 segundos 
#         self.timer = self.create_timer(0.5,self.publish_hello_world)
        
#         #se inicializa el contador con un valor de 1
#         self.counter = 1 
    
    
#     #Metodo para publicar mensajes 
    
     
#     def publish_hello_world(self):
#         #Este metodo crea un mensaje de tipo String
#         msg = String()
#         #Asigna el texto "Hello_world" seguido del valor actual del contador 
#         msg.data = "Hello_world " + str(self.counter)
#         #Publia el mensaje 
#         self.pub.publish(msg)
#         #incrementa el contador
#         self.counter += 1



# #La fucion principal de main
# def main(args = None):
#     #inicializa el contexto de ROS2
#     rclpy.init()
#     #crea una instancia del "HelloWorldPublisher(creamos nuestro nodo)"
#     my_pub = HelloWorldPublisher()
#     print("Publisher Node Running...")

#     #Espera a que el nodo gire, si detecta una interrupcion del teclado, se detruye 
#     #el nodo
#     try:
#         #spin mantiene el nodo corriendo y si alguna exception el nodo se destruye
#         rclpy.spin(my_pub)
#     except KeyboardInterrupt:
#         print("Terminating Node...")
#         my_pub.destroy_node()

# #Se verifica si el script esta siendo ejecutado directamente y en ese caso 
# #se llama a la funcion "main"
# if __name__ == '__main__':
#     main()