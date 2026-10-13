#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from collections import deque

from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo
import cv2
from cv_bridge import CvBridge
import numpy as np

from vision_msgs.msg import Detection2DArray
from eufs_msgs.msg import ConeArrayWithCovariance, ConeWithCovariance
from geometry_msgs.msg import Point

from rclpy.qos import qos_profile_sensor_data

from .DepthExtractionStrategy import DepthFromCenter, DepthFromCenterOrMedianRegion
from .utils.load_config import load_config
from .utils.PerceptionLogger import PerceptionLogger

class ConePositionEstimator(Node):
    def __init__(self):
        super().__init__('cone_converter_node')

        config = load_config('cone_position_estimator')
        
        # Declare parameters for topic names
        self.declare_parameter('detection_topic', '/zed/left/detections')
        self.declare_parameter('depth_topic', '/zed/depth/image_raw') 
        self.declare_parameter('camera_info_topic', '/zed/depth/camera_info') 
        self.declare_parameter('cone_topic', '/perception/cones')
        self.declare_parameter('bristol_fsai_debug', False)
        self.declare_parameter('bristol_fsai_hardware_mode', False)
        
        detection_topic = self.get_parameter('detection_topic').get_parameter_value().string_value
        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        cone_topic = self.get_parameter('cone_topic').get_parameter_value().string_value
        self.bristol_fsai_debug = self.get_parameter('bristol_fsai_debug').get_parameter_value().bool_value
        self.bristol_fsai_hardware_mode = self.get_parameter('bristol_fsai_hardware_mode').get_parameter_value().bool_value

        if self.bristol_fsai_hardware_mode:
            # If in hardware mode, use the ZED node topics
            depth_topic = '/zed/zed_node/depth/depth_registered'
            camera_info_topic = '/zed/zed_node/left/camera_info'
        
        # Subscribers
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            detection_topic,
            self.detection_callback,
            qos_profile_sensor_data
        )
        self.depth_sub = self.create_subscription(
            Image,
            depth_topic,
            self.depth_callback,
            qos_profile_sensor_data
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            qos_profile_sensor_data
        )
        
        # Publisher for cone detections
        self.cone_pub = self.create_publisher(
            ConeArrayWithCovariance, 
            cone_topic, 
            10
        )
        
        # CvBridge for converting images
        self.bridge = CvBridge()
        
        # Variables to store the latest depth images and camera info
        self.depth_buffer = deque(maxlen=config.get('depth_buffer_size', 10))  # Size of the depth image buffer
        self.camera_info = None

        # Select depth extraction strategy
        self.depth_estimator = DepthFromCenterOrMedianRegion()

        # Mapping from detection class IDs to ConeArrayWithCovariance field names
        self.class_mapping = {
            0: 'yellow_cones',
            1: 'blue_cones',
            2: 'orange_cones',
            3: 'big_orange_cones',
            4: 'unknown_color_cones'
        }
        self.closest_depth = config.get('closest_depth', 0.2)
        self.furthest_depth = config.get('furthest_depth', 15.0)
        self.fallen_cone_size_ratio = config.get('fallen_cone_size_ratio', 2.0)

        # VISUALIZATION
        if self.bristol_fsai_debug:
            cv2.namedWindow("Depth Image", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Depth Image", 640, 480)
        
        self.get_logger().info("Cone converter node started.")

        self.depth_logger = PerceptionLogger(config, folder_name='depth_logger')

    def depth_callback(self, msg: Image):
        # Store the depth image in the buffer
        self.depth_buffer.append(msg)

    def get_closest_depth_image(self, detection_timestamp):
        # Find the closest depth image to the detection timestamp
        closest_msg = None
        
        for msg in reversed(self.depth_buffer):
            msg_time = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec
            detection_time = detection_timestamp.sec * 1e9 + detection_timestamp.nanosec

            if msg_time <= detection_time:
                closest_msg = msg
                break # Found the closest one before detection
            
        if closest_msg is None:
            self.get_logger().warn("No depth image found for detection.")
            return None

        depth_image = self.bridge.imgmsg_to_cv2(closest_msg, desired_encoding='32FC1')

        '''Visualize the depth image
        # Replace inf and -inf values with 0
        depth_image[np.isinf(depth_image)] = 0
        # Normalize the depth image to the range [0, 255] for visualization
        normalized_depth_image = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
        normalized_depth_image = np.uint8(normalized_depth_image)
        # Save the colored depth image using OpenCV
        cv2.imwrite('normalized_depth_image.png', normalized_depth_image)
        '''
            
        return depth_image

    def camera_info_callback(self, msg: CameraInfo):
        self.camera_info = msg
        self.get_logger().info("Camera info received and stored.")
        self.destroy_subscription(self.camera_info_sub)
    
    def detection_callback(self, msg: Detection2DArray):
        # start_time = self.get_clock().now()

        # Retrieve closest depth image
        depth_image = self.get_closest_depth_image(msg.header.stamp)

        # Ensure that we have both a depth image and camera intrinsics
        if depth_image is None or self.camera_info is None:
            self.get_logger().warn("Depth image or camera info not received; skipping detection.")
            return
        
        # VISUALIZATION
        display_image = depth_image.copy()
        display_image[np.isinf(display_image)] = 0
        display_image[np.isnan(display_image)] = 0
        cv2.normalize(display_image, display_image, 0, 255, cv2.NORM_MINMAX)
        display_image = np.uint8(display_image)
        display_image_bgr = cv2.cvtColor(display_image, cv2.COLOR_GRAY2BGR)
        
        cone_array = ConeArrayWithCovariance()
        cone_array.header = msg.header
        cone_array.header.frame_id = 'base_footprint'
        
        # Initialize lists in the cone array
        cone_array.blue_cones = []
        cone_array.yellow_cones = []
        cone_array.orange_cones = []
        cone_array.big_orange_cones = []
        cone_array.unknown_color_cones = []
        
        # Extract camera intrinsics
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]
        
        # For each detection, back-project the pixel to 3D using the depth image and camera intrinsics
        for detection in msg.detections:
            try:
                class_id = int(detection.results[0].hypothesis.class_id)
            except Exception as e:
                self.get_logger().error(f"Could not parse class_id: {e}")
                continue

            # VISUALIZATION
            u = int(detection.bbox.center.x)
            v = int(detection.bbox.center.y)
            # Draw a small red circle (BGR format) at the center (u, v)
            # Check bounds just in case
            h, w = display_image_bgr.shape[:2]
            if 0 <= u < w and 0 <= v < h:
                cv2.circle(display_image_bgr, (u, v), radius=3, color=(0, 0, 255), thickness=-1)

            # Extract depth value from the depth image
            depth_value = self.depth_estimator.extract_depth(depth_image, detection, self.get_logger())

            # If depth estimation fails or is too close, skip this detection
            if depth_value is None:
                # self.get_logger().warn("Depth estimation failed.")
                continue
            
            # If depth is too close, mark it as "Too close"
            if depth_value < self.closest_depth:
                cv2.putText(display_image_bgr, "Too close", (u, v + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                continue

            # If depth is too far, mark it as "Too far"
            if depth_value > self.furthest_depth:
                cv2.putText(display_image_bgr, "Too far", (u, v + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                continue

            # Skip detections that are too small or have fallen cones
            if detection.bbox.size_x > self.fallen_cone_size_ratio * detection.bbox.size_y:
                cv2.putText(display_image_bgr, "Fallen cone", (u, v + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                continue

            cv2.putText(display_image_bgr, f"{depth_value:.2f} m", (u, v - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            
            # Back-project pixel (u, v) to a 3D point using the pinhole camera model
            x = (detection.bbox.center.x - cx) * depth_value / fx
            y = (detection.bbox.center.y - cy) * depth_value / fy
            z = depth_value
            
            # Convert the 3D point to a 2D map coordinate
            map_x = z
            map_y = -x
            
            cone = ConeWithCovariance()
            cone.point = Point(x=float(map_x), y=float(map_y), z=0.0)
            # Set a default covariance (adjust if needed)
            cone.covariance = [0.05, 0.0, 0.0, 0.05]
            
            if class_id in self.class_mapping:
                field_name = self.class_mapping[class_id]
                getattr(cone_array, field_name).append(cone)
            else:
                self.get_logger().warn(f"Received detection with unknown class id: {class_id}")
                cone_array.unknown_color_cones.append(cone)
        
        self.cone_pub.publish(cone_array)

        # VISUALIZATION
        if self.bristol_fsai_debug and display_image_bgr is not None:
            cv2.imshow("Depth Image", display_image_bgr)
            cv2.waitKey(1) 

        # If in hardware mode, log the depth image
        if self.bristol_fsai_hardware_mode and display_image_bgr is not None:
            # Log the depth image to file
            self.depth_logger.log_cv_image(display_image_bgr)
            
        # self.get_logger().info(cone_array)
        # self.get_logger().info("Published ConeArrayWithCovariance message.")

        # end_time = self.get_clock().now()
        # elapsed_time = (end_time - start_time).nanoseconds / 1e6
        # self.get_logger().info(f"Processing time: {elapsed_time:.2f} ms")


def main(args=None):
    rclpy.init(args=args)
    node = ConePositionEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()