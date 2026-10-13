import os
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO
import torch
from std_msgs.msg import Header
from sensor_msgs.msg import Image
import message_filters
from ament_index_python.packages import get_package_share_directory
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from datetime import datetime

from .utils.load_config import load_config
from .utils.PerceptionLogger import PerceptionLogger

class YOLONode(Node):
    """
    A ROS2 node that uses YOLO for object detection on images from the ZED cameras.

    For each received image, it:
        - Converts the ROS Image message to an OpenCV image using CvBridge.
        - Runs YOLO inference to detect objects.
        - Extracts detection data including the object class, confidence, and bounding box coordinates.
        - Computes the bounding box center (x, y) and size (width as size_x and height as size_y) from the
        minimum and maximum coordinates.

    Class IDs:
        0: yellow_cone
        1: blue_cone
        2: orange_cone
        3: large_orange_cone
        4: unknown_cone

    The detection results are packaged into vision_msgs/Detection2DArray messages and published separately
    for the left and right cameras on the following topics:
        - '/zed/left/detections'
        - '/zed/right/detections'
    """
    def __init__(self):
        super().__init__('yolo_node')
        config = load_config('yolo_node')
        self.declare_parameter('bristol_fsai_hardware_mode', False)
        self.declare_parameter('bristol_fsai_debug', False)
        self.bristol_fsai_debug = self.get_parameter('bristol_fsai_debug').get_parameter_value().bool_value
        self.bristol_fsai_hardware_mode = self.get_parameter('bristol_fsai_hardware_mode').get_parameter_value().bool_value
        self.model = YOLO(os.path.join(get_package_share_directory('perception'), config['model_path']))
        self.model_conf_threshold = config.get('model_conf_threshold', 0.5)
        self.model.eval()  # Set the model to evaluation mode
        self.bridge = CvBridge()

        self.device = "cuda:0" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"YOLO is running on {self.device}.")

        # Publishers for left and right detections
        self.left_detections_publisher = self.create_publisher(
            Detection2DArray, '/zed/left/detections', 10
        )
        # self.right_detections_publisher = self.create_publisher(
        #     Detection2DArray, '/zed/right/detections', 10
        # )

        # Define the QoS profile with BEST_EFFORT reliability
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Set reliability to BEST_EFFORT
            history=QoSHistoryPolicy.KEEP_LAST,            # Keep only the last messages
            depth=10                                       # Queue size of 10
        )

        # Subscriptions for left, right and depth (from left) camera images
        left_camera_topic = '/zed/' + ('zed_node/' if self.bristol_fsai_hardware_mode else '') + 'left/image_rect_color'
        self.left_sub = message_filters.Subscriber(
            self, Image, left_camera_topic, qos_profile=qos_profile
        )
        # self.right_sub = message_filters.Subscriber(
        #     self, Image, '/zed/right/image_rect_color', qos_profile=qos_profile
        # )

        # Create the approximate time synchroniser to sync the subscribers
        self.ats = message_filters.ApproximateTimeSynchronizer(
            # [self.left_sub, self.right_sub], 100, 0.1
            [self.left_sub], 100, 0.1
        )
        self.ats.registerCallback(self.images_callback)

        # Variables to store left and right images
        self.left_image = None
        self.right_image = None

        # VISUALIZATION
        if self.bristol_fsai_debug:
            # Create windows for both cameras
            cv2.namedWindow('Left Camera Detections', cv2.WINDOW_NORMAL)
            # cv2.namedWindow('Right Camera Detections', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('Left Camera Detections', 640, 480)
            # cv2.resizeWindow('Right Camera Detections', 640, 480)
            # Position windows side by side
            cv2.moveWindow('Left Camera Detections', 0, 0)
            # cv2.moveWindow('Right Camera Detections', 650, 0)

        self.detection_logger = PerceptionLogger(config, folder_name='yolo_detections')

    def __del__(self):
        """Destructor to ensure proper cleanup of CV2 windows"""
        try:
            if hasattr(self, 'bristol_fsai_debug') and self.bristol_fsai_debug:
                cv2.destroyAllWindows()
        except:
            pass

    def images_callback(self, left_msg):
        start = datetime.now()
        
        # Convert ROS images to OpenCV
        left_cv = self.bridge.imgmsg_to_cv2(left_msg, 'bgr8')

        # Process images as a batch
        batch = [left_cv]
        results = self.model(batch, device=self.device, verbose=False, conf=self.model_conf_threshold)
        
        # Process results in parallel
        left_detections = self.process_results(results[0])

        # VISUALIZATION
        left_plotted = results[0].plot()
        if self.bristol_fsai_debug:
            cv2.imshow('Left Camera Detections', left_plotted)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info('Quit key pressed, shutting down...')
                rclpy.shutdown()

        # Publish detections
        self.publish_detections(left_detections, 'left', left_msg.header)
        
        if self.bristol_fsai_hardware_mode:
            self.detection_logger.log_cv_image(left_plotted)

    def process_results(self, results):
        detections = []
        for b in results.boxes:
            coord = b.xyxy[0].cpu().detach().numpy()
            x_min, y_min, x_max, y_max = coord.tolist()
            img_class = int(b.cls)
            conf = float(b.conf[0].cpu().detach().numpy())
            detections.append((img_class, conf, x_min, y_min, x_max, y_max))
        return detections

    def publish_detections(self, detections, camera_side, original_header):
        detection_array = Detection2DArray()
        detection_array.header = Header()
        detection_array.header.stamp = original_header.stamp
        detection_array.header.frame_id = f"{camera_side}_camera"

        for class_id, confidence, x_min, y_min, x_max, y_max in detections:
            detection = Detection2D()
            
            # Bounding box center and size
            detection.bbox.center.x = (x_min + x_max) / 2
            detection.bbox.center.y = (y_min + y_max) / 2
            detection.bbox.size_x = x_max - x_min
            detection.bbox.size_y = y_max - y_min

            # Object hypothesis
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(class_id)
            hypothesis.hypothesis.score = confidence
            detection.results.append(hypothesis)

            detection_array.detections.append(detection)

        # Publish based on camera side
        publisher = self.left_detections_publisher # if camera_side == 'left' else self.right_detections_publisher
        publisher.publish(detection_array)
        # self.get_logger().info(f'Published {len(detections)} detections from {camera_side} camera')


def main(args=None):
    rclpy.init(args=args)
    node = YOLONode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()