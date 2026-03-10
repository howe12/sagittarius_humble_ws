from pathlib import Path
from typing import List, Optional
 
from ament_index_python.packages import get_package_share_directory
#from pydantic import BaseModel, root_validator, validator
from pydantic import BaseModel, model_validator, validator
 
USB_CAM_DIR = get_package_share_directory('usb_cam')
 
 
class CameraConfig(BaseModel):
    name: str = 'camera1'
    param_path: Path = Path(USB_CAM_DIR, 'config', 'params_1.yaml')
    remappings: Optional[List[str]] = []
    namespace: Optional[str] = None
 
    @validator('param_path')
    def validate_param_path(cls, value):
        if value and not value.exists():
            raise FileNotFoundError(f'Could not find parameter file: {value}')
        return value
 
    @validator('name')
    def validate_name(cls, value):
        if not value:
            raise ValueError("Name is required")
        return value
 
    def __init__(self, **data):
        super().__init__(**data)
        if self.name:
            # Automatically set remappings if name is set
            self.remappings.extend([
                ('image_raw', f'{self.name}/image_raw'),
                ('image_raw/compressed', f'{self.name}/image_compressed'),
                ('image_raw/compressedDepth', f'{self.name}/compressedDepth'),
                ('image_raw/theora', f'{self.name}/image_raw/theora'),
                ('camera_info', f'{self.name}/camera_info'),
            ])