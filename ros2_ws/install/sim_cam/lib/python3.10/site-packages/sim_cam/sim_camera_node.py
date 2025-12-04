#!/usr/bin/env python3
import os
import glob
import time

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class SimulatedCameraNode(Node):
    def __init__(self):
        super().__init__('simulated_camera')

        # Parámetros
        self.declare_parameter('image_dir', '/home/ros/data/cam1/orbbec_rgb')
        self.declare_parameter('frame_rate', 5.0)  # fps
        self.declare_parameter('loop', True)

        self.image_dir = self.get_parameter('image_dir').get_parameter_value().string_value
        self.frame_rate = self.get_parameter('frame_rate').get_parameter_value().double_value
        self.loop = self.get_parameter('loop').get_parameter_value().bool_value

        self.get_logger().info(f'Usando carpeta de imágenes: {self.image_dir}')
        self.get_logger().info(f'Frame rate: {self.frame_rate} fps')
        self.get_logger().info(f'Loop: {self.loop}')

        # Cargar lista de imágenes
        pattern = os.path.join(self.image_dir, '*')
        self.image_paths = sorted(
            [p for p in glob.glob(pattern) if p.lower().endswith(('.jpg', '.jpeg', '.png'))]
        )

        if not self.image_paths:
            self.get_logger().error(f'No se encontraron imágenes en {self.image_dir}')
        else:
            self.get_logger().info(f'Se encontraron {len(self.image_paths)} imágenes.')

        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)

        # Índice del frame actual
        self.current_index = 0

        # Timer según frame_rate
        if self.frame_rate <= 0:
            self.get_logger().warn('frame_rate <= 0, usando 1.0 fps por defecto')
            self.frame_rate = 1.0

        timer_period = 1.0 / self.frame_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if not self.image_paths:
            # Nada que publicar
            return

        if self.current_index >= len(self.image_paths):
            if self.loop:
                self.current_index = 0
            else:
                self.get_logger().info('Se llegó al final de las imágenes y loop=False, deteniendo nodo.')
                # Detenemos el timer y no publicamos más
                self.timer.cancel()
                return

        image_path = self.image_paths[self.current_index]
        self.current_index += 1

        # Leer imagen con OpenCV
        cv_image = cv2.imread(image_path, cv2.IMREAD_COLOR)
        if cv_image is None:
            self.get_logger().warn(f'No se pudo leer la imagen: {image_path}')
            return

        # Convertir a mensaje ROS
        img_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = 'sim_cam_frame'

        # Publicar
        self.publisher_.publish(img_msg)
        self.get_logger().info(f'Publicado frame desde: {os.path.basename(image_path)}')


def main(args=None):
    rclpy.init(args=args)
    node = SimulatedCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
