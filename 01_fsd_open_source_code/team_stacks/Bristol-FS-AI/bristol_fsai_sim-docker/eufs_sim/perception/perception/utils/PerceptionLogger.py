import time
import cv2
import os

class PerceptionLogger:
    def __init__(self, config, folder_name="", log_interval=3.0):
        """
        Initializes the PerceptionLogger with the specified configuration.

        Args:
            config (dict): Configuration dictionary containing parameters for detection logging.
        """
        self.config = config
        self.detection_logging_dir = config.get('perception_logging_dir', None)
        self.log_interval = log_interval
        self.last_log_time = 0

        if self.detection_logging_dir:
            self.detection_logging_dir = os.path.join(
                self.detection_logging_dir,
                f"{folder_name}-{time.strftime('%Y%m%d-%H%M%S')}"
            )
            os.makedirs(self.detection_logging_dir, exist_ok=True)
            print(f"Detection logging directory set to: {self.detection_logging_dir}")
        else:
            print("No detection logging directory specified.")

    def log_cv_image(self, image):
        if not self.detection_logging_dir:
            return 
        
        current_time = time.perf_counter()

        if current_time - self.last_log_time >= self.log_interval:
            self.last_log_time = current_time
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            filename = f"{self.detection_logging_dir}/{timestamp}.jpg"
            cv2.imwrite(filename, image)