import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import os
import yaml
from mmseg.apis import MMSegInferencer
from mmseg.evaluation.metrics import CityscapesMetric
import time

global resultado

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(Image, 'image_topic', self.listener_callback, 10)
        self.get_logger().info(f'Initialized')
        self.publisher_ = self.create_publisher(Bool, 'image_processed', 10)

        self.bridge = CvBridge()
        self.output_dir = '/home/vinicius/ros2_ws/src/image_processor/processed_images/' 
        os.makedirs(self.output_dir, exist_ok=True)
        self.image_index = 0 

        #Load the configuration file (yaml)
        config_path = '/home/vinicius/ros2_ws/src/image_processor/config/define_net.yaml'
        self.config = self.load_config(config_path)

        #Initialize MMSeg
        model_config = self.config['model']['config_file']
        model_weights = self.config['model']['weights_file']
        self.inferencer = MMSegInferencer(model=model_config, weights=model_weights) #WE are having a problem here with the mmseg scope

    def load_config(self, config_path):
        with open(config_path, 'r') as file:
            return yaml.safe_load(file)

    def listener_callback(self, msg):
        global resultado
        self.get_logger().info("Received an image, processing...")

        #Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        #Save the received image 
        input_image_path = os.path.join(self.output_dir, f'received_image_{self.image_index}.jpg')
        # cv2.imwrite(input_image_path, cv_image)
        # self.get_logger().info(f"Saved received image: {input_image_path}") -> #Taking it off can gain us some time

        #Run inference using the defined model and congig
        output_dir = self.config['output']['out_dir']
        vis_dir = self.config['output']['vis_dir']
        pred_dir = self.config['output']['pred_dir']

        os.makedirs(output_dir, exist_ok=True)
        os.makedirs(vis_dir, exist_ok=True)
        os.makedirs(pred_dir, exist_ok=True)
        
        try:
            resultado = self.inferencer(
                cv_image,  
                out_dir=output_dir,
                img_out_dir=vis_dir,
                pred_out_dir=pred_dir, 
                return_datasamples=True, 
            )
            self.get_logger().info(f"Inference completed for image: {input_image_path}")

            #publish confirmation
            confirmation_msg = Bool()
            confirmation_msg.data = True
            self.publisher_.publish(confirmation_msg)
            self.get_logger().info(f"Published processing confirmation for image: {self.image_index}")

        except Exception as e:
            # metrics = CityscapesMetric(output_dir=output_dir, keep_results=True)
            # metrics.compute_metrics(resultado)
            self.get_logger().error(f"Error during inference: {str(e)}")


        self.image_index += 1

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
