import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import numpy as np
import cv2
from cv_bridge import CvBridge

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')

        # Create separate publishers for each image topic
        self.pub_mono = self.create_publisher(Image, '/img_mono', 10)
        self.pub_red = self.create_publisher(Image, '/img_red', 10)
        self.pub_green = self.create_publisher(Image, '/img_green', 10)
        self.pub_blue = self.create_publisher(Image, '/img_blue', 10)

        self.bridge = CvBridge()

        # Timer to periodically publish images (2 Hz)
        self.timer = self.create_timer(0.5, self.publish_img)
        
        # Image size
        self.height = 20
        self.width = 20

        self.get_logger().info('Publishing 4 images (monochrome static, red static, blue static, green static)')

    def gen_noise(self):
        return np.random.randint(0, 256, (self.height, self.width), dtype=np.uint8)
    
    def gen_colored_msg(self, channel: int = 0):
        c_img = np.zeros((self.height, self.width, 3), dtype=np.uint8)  # RGB image
        c_img[:, :, channel] = self.gen_noise()
        return self.bridge.cv2_to_imgmsg(c_img, encoding="rgb8")

    def add_header(self):
        return Header(stamp=self.get_clock().now().to_msg(), frame_id="camera")

    def publish_img(self):
        msg_list = []
        mono_image = self.gen_noise()
        mono_msg = self.bridge.cv2_to_imgmsg(mono_image, encoding="mono8")
        msg_list.append(mono_msg)
        
        for i in range(0, 3):
            msg_list.append(self.gen_colored_msg(i))

        for msg in msg_list:
            msg.header = self.add_header()

        # Publish the images to their respective topics
        self.pub_mono.publish(msg_list[0])
        self.pub_red.publish(msg_list[1])
        self.pub_green.publish(msg_list[2])
        self.pub_blue.publish(msg_list[3])
        
def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()