import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class RotationPublisher(Node):
    def __init__(self):
        super().__init__('rotation_publisher')
        self.publisher_ = self.create_publisher(
            Float32MultiArray, 'rotation_input', 10)
        self.subscriber = self.create_subscription(
            Float32MultiArray, 'vector_rotado', self.vector_rotado_callback, 10)


    def vector_rotado_callback(self, msg):
        vector_exacto = list(msg.data[0:3])
        vector_micro = list(msg.data[3:6])

        print(f"Vector de una sola rotacion: {vector_exacto}\n"
            f"vector con micro-rotaciones: {vector_micro}")

    def run(self):

        vector = list(map(float, input(
            "Ingresa las 3 componentes del vector (x,y,z) separados por espacio: ").split())
        )
        angulos = list(map(float, input(
            "Ingresa tres angulos separados por espacio: ").split())
        )
       

        paso = float(input("Ingrese la cantidad de paso:"))
        

        msg = Float32MultiArray()
        msg.data = vector + angulos + [paso]

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RotationPublisher()
    node.run()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
