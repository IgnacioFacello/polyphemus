import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray



class RotationPublisher(Node):
    def __init__(self):
        super().__init__('rotation_publisher')
        self.publisher_ = self.create_publisher(
            Float32MultiArray, 'rotation_input', 10)

    def run(self):

        vector = list(map(float, input(
            "Ingresa las 3 componentes del vector (x,y,z) separados por espacio: ").split())
)
        print(f"Valores ingresados: {vector[0]}, {vector[1]}, {vector[2]}")
        angulos = list(map(float, input(
            "Ingresa tres angulos separados por espacio: ").split())
)
        print(f"Valores ingresados: {angulos[0]},  {angulos[1]},  {angulos[2]}")

        paso = float(input("Ingrese la cantidad de paso:"))
        print("La cantidad de paso es:", paso)

        msg =Float32MultiArray()
        msg.data = vector + angulos + [paso]

        self.publisher_.publish(msg)




def main(args=None):
    rclpy.init(args=args)
    node = RotationPublisher()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
