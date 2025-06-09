#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, Imu, Image
from nav_msgs.msg import Odometry, OccupancyGrid 
from geometry_msgs.msg import Twist, PoseStamped 

from scipy.spatial.transform import Rotation as R

from cv_bridge import CvBridge
import cv2
import numpy as np
import math


class ControleRobo(Node):

    def __init__(self):
        super().__init__('controle_robo')

        # Publisher para comando de velocidade
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
 
        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        # self.create_subscription(Image, '/robot_cam/colored_map', self.camera_callback, 10)
        self.create_subscription(OccupancyGrid,'/map',self.map_callback,10)
        self.create_subscription(Odometry, '/odom_gt', self.odometry_callback, 10) 

        self.bridge = CvBridge()

        # Timer para enviar comandos continuamente
        self.timer = self.create_timer(0.1, self.move_robot)

        # Estado interno
        self.obstaculo_a_frente = False
        self.current_map = None
        self.map_info = None
        self.robot_pose = None 
        self.exploration_target = None 

        self.get_logger().info("Robô rodando!")


    def scan_callback(self, msg: LaserScan):
        # Verifica uma faixa estreita ao redor de 0° (frente)
        num_ranges = len(msg.ranges)
        if num_ranges == 0:
            return

        # Índices de -30° a +30° (equivalente a 330 até 30)
        # Usamos uma cópia para evitar modificar a lista original durante a iteração
        front_indices = list(range(330, 360)) + list(range(0, 31))

        # Filtra distancias, ignorando 'inf' (alcance máximo)
        valid_distances = [msg.ranges[i] for i in front_indices if not math.isinf(msg.ranges[i])]

        if valid_distances and min(valid_distances) < 0.5:
            self.obstaculo_a_frente = True
            # self.get_logger().info('Obstáculo detectado a {:.2f}m à frente'.format(min(valid_distances)))
        else:
            self.obstaculo_a_frente = False

    def odometry_callback(self, msg: Odometry):
        # Pegando a pose do odometro
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        
        r = R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w])
        _, _, yaw = r.as_euler('xyz') 
        
        self.robot_pose = (position.x, position.y, yaw)


    def map_callback(self, msg: OccupancyGrid):
        self.current_map = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.map_info = msg.info

        # Ao atualizar o mapa, atualiza o target de exploracao caso ele tenha sido alcancado
        if(self.exploration_target is None):
            self.find_and_set_exploration_target()


    def find_and_set_exploration_target(self):
        if self.current_map is None or self.map_info is None or self.robot_pose is None:
            return

        # Puxando as infos de self
        height, width = self.current_map.shape
        resolution = self.map_info.resolution
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y

        # Convertendo a pose para pixels 
        robot_x_map, robot_y_map, _ = self.robot_pose
        robot_x_pixel = int((robot_x_map - origin_x) / resolution)
        robot_y_pixel = int((robot_y_map - origin_y) / resolution)

        if not (0 <= robot_x_pixel < width and 0 <= robot_y_pixel < height):
            self.get_logger().warn(f"O robô tá fora do mapa")
            return

        # Mapeando fronteiras
        frontiers = []

        offsets = [(-1, -1), (-1, 0), (-1, 1),
                   (0, -1),           (0, 1),
                   (1, -1), (1, 0), (1, 1)]

        for y in range(height):
            for x in range(width):
                if self.current_map[y, x] == -1: 
                    is_frontier = False
                    for dy, dx in offsets:
                        nx, ny = x + dx, y + dy
                        if 0 <= nx < width and 0 <= ny < height and self.current_map[ny, nx] == 0: 
                            is_frontier = True
                            break
                    if is_frontier:
                        frontiers.append((x, y))

        if not frontiers:
            self.get_logger().info("O Mapa inteiro foi explorado.")
            self.exploration_target = None
            return

        # Procurando pela fronteira inexplorada mais próxima
        min_distance = float('inf')
        closest_frontier_pixel = None

        for fx, fy in frontiers:
            dist = math.sqrt((fx - robot_x_pixel)**2 + (fy - robot_y_pixel)**2)
            if dist < min_distance:
                min_distance = dist
                closest_frontier_pixel = (fx, fy)

        if closest_frontier_pixel:
            target_x_map = origin_x + closest_frontier_pixel[0] * resolution
            target_y_map = origin_y + closest_frontier_pixel[1] * resolution
            self.exploration_target = (target_x_map, target_y_map)
            self.get_logger().info(f"indo para: ({target_x_map:.2f}, {target_y_map:.2f})")
        else:
            self.exploration_target = None


    def move_robot(self):
        twist = Twist()

        # Evita obstáculo
        if self.obstaculo_a_frente:
            twist.angular.z = -0.3  
            twist.linear.x = 0.0 
            self.cmd_vel_pub.publish(twist)
            return

        # Vai em direção ao target definido
        if self.exploration_target and self.robot_pose:
            target_x, target_y = self.exploration_target
            robot_x, robot_y, robot_yaw = self.robot_pose

            # Calcula a direcao
            dx = target_x - robot_x
            dy = target_y - robot_y
            distance_to_target = math.sqrt(dx**2 + dy**2)
            angle_to_target = math.atan2(dy, dx)

            if distance_to_target < 0.1: # Chegou no target
                self.get_logger().info("Target alcancando!")
                self.exploration_target = None
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                return

            angle_error = angle_to_target - robot_yaw
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

            # Ajusta a direcao e velocidade conforme o necessario
            angular_speed = 0.8 * angle_error 
            linear_speed = min(1.0, distance_to_target / 2.0) 

            twist.linear.x = linear_speed
            twist.angular.z = angular_speed
            self.cmd_vel_pub.publish(twist)

        elif self.exploration_target is None:
            twist.linear.x =  0.5 if self.obstaculo_a_frente else 0.0
            twist.angular.z = 0.2 
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info("Sem target. Vagando por aí")


def main(args=None):
    rclpy.init(args=args)
    node = ControleRobo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()