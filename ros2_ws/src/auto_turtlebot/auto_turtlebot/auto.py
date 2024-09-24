import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt, pi
import tf_transformations

class TurtleBotMover(Node):

    def __init__(self):
        super().__init__('turtlebot3_move_to_goal')
        
        # Configura o publisher para enviar comandos de velocidade
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Configura o subscriber para receber dados de odometria (posição atual do robô)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.update_odom, 10)
        
        # Inicializa a odometria
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Variáveis para controle do objetivo
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.moving = False

    def update_odom(self, msg):
        """Atualiza a posição atual do robô a partir da odometria."""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Converte a orientação quaternária em ângulo theta (yaw)
        orientation_q = msg.pose.pose.orientation
        self.theta = self.quaternion_to_euler(orientation_q)
        print('Posição em x: ', self.x)
        print('Posição em y: ', self.y)


    def quaternion_to_euler(self, orientation_q):
        """Converte quaternions para ângulo em radianos (yaw)."""
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = tf_transformations.euler_from_quaternion(orientation_list)
        return yaw

    def normalize_angle(self, angle):
        """Normaliza o ângulo para estar entre -pi e pi."""
        if angle > pi:
            angle -= 2 * pi
        elif angle < -pi:
            angle += 2 * pi
        return angle

    def go_to_goal(self, goal_x, goal_y, distance_error_tolerance=0.005):
        """Move o TurtleBot para o ponto objetivo (goal_x, goal_y)."""
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.moving = True
        vel_msg = Twist()

        while rclpy.ok():
            # Calcular a distância euclidiana até o ponto objetivo
            distance = sqrt(pow((self.goal_x - self.x), 2) + pow((self.goal_y - self.y), 2))

            # Proporcional ao ângulo em que o robô deve se alinhar
            angle_to_goal = atan2(self.goal_y - self.y, self.goal_x - self.x)

            # Calcular o erro angular e normalizar o ângulo
            angle_error = self.normalize_angle(angle_to_goal - self.theta)

            # Tolerância angular para evitar ajustes contínuos
            angular_tolerance = 0.1  # Aproximadamente 5.7 graus

            # Controle angular (alinhar com o ponto objetivo)
            if abs(angle_error) > angular_tolerance:
                vel_msg.angular.z = 0.3 * angle_error  # Ajuste do ganho angular
            else:
                vel_msg.angular.z = 0.0

            # Controle linear (avançar em linha reta)
            if abs(angle_error) < angular_tolerance:  # Só se move se estiver alinhado
                vel_msg.linear.x = 0.3 * distance  # Velocidade linear reduzida para mais controle
            else:
                vel_msg.linear.x = 0.0

            # Verifica se o robô chegou perto do ponto objetivo
            if distance < distance_error_tolerance:
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = 0.0
                self.velocity_publisher.publish(vel_msg)
                self.get_logger().info("Ponto alcançado.")
                break

            # Publica a mensagem de velocidade
            self.velocity_publisher.publish(vel_msg)
            rclpy.spin_once(self)

def main(args=None):
    rclpy.init(args=args)
    
    # Cria o nó TurtleBotMover
    mover = TurtleBotMover()

    try:
        # Defina o ponto de destino
        goal_x = float(input("Digite a coordenada x do ponto: "))
        goal_y = float(input("Digite a coordenada y do ponto: "))
        
        # Move o TurtleBot até o ponto desejado
        mover.go_to_goal(goal_x, goal_y)
        
    except KeyboardInterrupt:
        pass

    # Desliga o nó ROS2
    mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
