import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import cv2
import os
import time
from cv_bridge import CvBridge

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, 'image_topic', 10)
        self.confirmation_subscriber = self.create_subscription(Bool, 'image_processed', self.on_image_processed, 10)
        
        self.bridge = CvBridge()
        self.directory = '/home/vinicius/ros2_ws/src/image_processor/images_source' 
        self.image_files = [f for f in os.listdir(self.directory) if f.endswith('.jpg') or f.endswith('.png')]
        self.image_index = 0
        self.image_processed = True  
        
        time.sleep(3)
        self.timer = self.create_timer(0.05, self.timer_callback)
        
    def timer_callback(self):
        if self.image_index < len(self.image_files) and self.image_processed:
            image_path = os.path.join(self.directory, self.image_files[self.image_index])
            cv_image = cv2.imread(image_path)
            if cv_image is not None:
                ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
                self.publisher_.publish(ros_image)
                self.image_processed = False 
                self.get_logger().info(f'Published image: {self.image_files[self.image_index]}')
                self.image_index += 1
            else:
                self.get_logger().warn(f'Failed to read image {self.image_files[self.image_index]}')
    
    def on_image_processed(self, msg):
        if msg.data:
            self.image_processed = True
            self.get_logger().info('Received confirmation that image was processed')

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
