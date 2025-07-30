#!/usr/bin/env python3
"""
Setup script for YOLO-World model
Downloads and configures YOLO-World for weapon detection
"""

import os
import sys
import logging
from ultralytics import YOLO
import cv2

try:
    from ultralytics import YOLO
except ImportError:
    print("❌ Ultralytics not installed. Please run: pip install ultralytics")
    sys.exit(1)

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

def download_yolo_world_model():
    """Download YOLO-World model if not present"""
    print("🔄 Setting up YOLO-World model...")
    
    model_path = "yolov8s-world.pt"
    
    try:
        if os.path.exists(model_path):
            print(f"✓ Model already exists at {model_path}")
        else:
            print(f"📥 Downloading YOLO-World model to {model_path}...")
            
        # Initialize YOLO-World model (will download if not present)
        model = YOLO(model_path)
        print("✓ YOLO-World model loaded successfully")
        
        # Test setting custom classes
        weapon_classes = ["knife", "sword", "gun"]
        model.set_classes(weapon_classes)
        print(f"✓ Custom weapon classes set: {weapon_classes}")
        
        return True
        
    except Exception as e:
        print(f"❌ Failed to setup YOLO-World model: {e}")
        return False

def test_model_functionality():
    """Test basic model functionality"""
    print("\n🧪 Testing model functionality...")
    
    try:
        import numpy as np
        import cv2
        
        # Create a test image
        test_image = np.zeros((640, 640, 3), dtype=np.uint8)
        cv2.putText(test_image, "Test Image", (200, 320), 
                   cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 3)
        
        # Initialize model
        model = YOLO("yolov8s-world.pt")
        model.set_classes(["knife", "sword", "gun"])
        
        # Run inference
        results = model(test_image, verbose=False)
        print("✓ Model inference completed successfully")
        
        # Check results structure
        if results and len(results) > 0:
            print(f"✓ Results structure: {type(results[0])}")
            if hasattr(results[0], 'boxes'):
                print("✓ Boxes attribute available")
            else:
                print("⚠️  Boxes attribute not found")
        
        return True
        
    except Exception as e:
        print(f"❌ Model functionality test failed: {e}")
        return False

def update_requirements():
    """Check and suggest requirements updates"""
    print("\n📋 Checking requirements...")
    
    required_packages = [
        "ultralytics",
        "opencv-python", 
        "numpy"
    ]
    
    missing_packages = []
    
    for package in required_packages:
        try:
            __import__(package.replace('-', '_'))
            print(f"✓ {package}")
        except ImportError:
            missing_packages.append(package)
            print(f"❌ {package} (missing)")
    
    if missing_packages:
        print(f"\n📦 Install missing packages:")
        print(f"pip install {' '.join(missing_packages)}")
        return False
    else:
        print("✓ All required packages are installed")
        return True

def create_config_section():
    """Suggest configuration for config.ini"""
    print("\n⚙️  Suggested configuration for config.ini:")
    print("[weapon_detection]")
    print("model_path = yolov8s-world.pt")
    print("confidence_threshold = 0.5")
    print("target_classes = knife,sword,gun")
    print("enable_logging = true")
    print()

def main():
    """Main setup function"""
    print("🚀 YOLO-World Weapon Detection Setup")
    print("=" * 50)
    
    success = True
    
    # Download and setup model
    if success and not download_yolo_world_model():
        success = False
    
    # Test functionality
    if success and not test_model_functionality():
        success = False
    
    # Show configuration suggestion
    if success:
        create_config_section()
    
    print("=" * 50)
    if success:
        print("🎉 YOLO-World setup completed successfully!")
    else:
        print("❌ Setup failed. Please check errors above.")
    
    return success

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)