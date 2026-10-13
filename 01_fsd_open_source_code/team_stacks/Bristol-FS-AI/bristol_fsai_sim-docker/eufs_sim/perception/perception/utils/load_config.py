from typing import Dict, Any
import os
import yaml
from ament_index_python.packages import get_package_share_directory

def load_config(node: str) -> Dict[str, Any]:
    """Load configuration from YAML file"""
    # Get the package directory
    package_dir = get_package_share_directory('perception')
    config_path = os.path.join(package_dir, 'config', 'perception_params.yaml')
    
    # Load YAML file
    with open(config_path, 'r') as file:
        config_data = yaml.safe_load(file)
    
    # Extract the node configuration
    return config_data[node]