from abc import ABC, abstractmethod
import numpy as np


class DepthExtractionStrategy(ABC):
    """Abstract base class for different depth extraction strategies."""
    
    @abstractmethod
    def extract_depth(self, depth_image: np.ndarray, bbox, logger, **kwargs):
        """Extracts a depth value given a bounding box in the depth image."""
        pass


class DepthFromCenter(DepthExtractionStrategy):
    """Extracts depth from the center of the bounding box."""
    
    def extract_depth(self, depth_image: np.ndarray, detection, logger, **kwargs):
        center_u = detection.bbox.center.x
        center_v = detection.bbox.center.y

        u_int = int(round(center_u))
        v_int = int(round(center_v))
        
        # Check bounds against the depth image dimensions
        depth_height, depth_width = depth_image.shape
        if u_int < 0 or u_int >= depth_width or v_int < 0 or v_int >= depth_height:
            logger.warn(f"Pixel ({u_int}, {v_int}) is out of bounds in the depth image (w: {depth_width}, h: {depth_height}).")
            return
        
        depth_value = depth_image[v_int, u_int]
        if depth_value == 0 or np.isnan(depth_value) or np.isinf(depth_value):
            # logger.warn(f"No valid depth at pixel ({u_int}, {v_int}).")
            return

        return depth_value
    

class DepthFromCenterOrMedianRegion(DepthExtractionStrategy):
    """Extracts depth from the center of the bounding box,
    or median from a 40x40 region if center is invalid."""
    window_size = 40

    def extract_depth(self, depth_image: np.ndarray, detection, logger, **kwargs):
        center_u = detection.bbox.center.x
        center_v = detection.bbox.center.y

        u_int = int(round(center_u))
        v_int = int(round(center_v))
        
        depth_height, depth_width = depth_image.shape
        if u_int < 0 or u_int >= depth_width or v_int < 0 or v_int >= depth_height:
            logger.warn(f"Pixel ({u_int}, {v_int}) is out of bounds in the depth image (w: {depth_width}, h: {depth_height}).")
            return

        depth_value = depth_image[v_int, u_int]
        if depth_value == 0 or np.isnan(depth_value) or np.isinf(depth_value):
            # Define window size
            window_size = self.window_size // 2  
            u_start = max(0, u_int - window_size)
            u_end = min(depth_width, u_int + window_size)
            v_start = max(0, v_int - window_size)
            v_end = min(depth_height, v_int + window_size)

            region = depth_image[v_start:v_end, u_start:u_end]
            valid_values = region[(region > 0) & np.isfinite(region)]

            if valid_values.size == 0:
                logger.warn(f"No valid depth values in 40x40 region around ({u_int}, {v_int}).")
                return

            median_depth = np.median(valid_values)
            return median_depth

        return depth_value