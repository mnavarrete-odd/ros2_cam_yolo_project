#!/usr/bin/env python3
import os
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class DummyYoloDetector(Node):
    def __init__(self):
        super().__init__('dummy_yolo_detector')

        # Parámetros
        self.declare_parameter('output_dir', '/home/ros/output/detections')
        self.declare_parameter('draw_box', True)

        self.output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        self.draw_box = self.get_parameter('draw_box').get_parameter_value().bool_value

        os.makedirs(self.output_dir, exist_ok=True)
        self.get_logger().info(f'Guardando detecciones en: {self.output_dir}')

        self.bridge = CvBridge()

        # Suscriptor a la cámara simulada
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.subscription  # evitar warning

        # Publicador de imagen con "detecciones"
        self.publisher_ = self.create_publisher(Image, '/detections/image', 10)

        self.frame_counter = 0

    def image_callback(self, msg: Image):
        # Convertir mensaje ROS a imagen OpenCV (BGR)
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Dibujar una caja dummy
        if self.draw_box:
            h, w, _ = cv_image.shape
            # Caja en el centro
            x1 = int(w * 0.3)
            y1 = int(h * 0.3)
            x2 = int(w * 0.7)
            y2 = int(h * 0.7)
            cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                cv_image,
                'dummy_object',
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

        # Publicar imagen modificada en /detections/image
        out_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        out_msg.header = msg.header  # conservar timestamp y frame_id
        self.publisher_.publish(out_msg)

        # Guardar en disco
        filename = f'det_{self.frame_counter:06d}.jpg'
        save_path = os.path.join(self.output_dir, filename)
        cv2.imwrite(save_path, cv_image)
        self.get_logger().info(f'Guardada imagen con detección: {save_path}')

        self.frame_counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = DummyYoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
