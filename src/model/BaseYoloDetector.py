from abc import ABC, abstractmethod
import numpy as np
from typing import List, Dict, Any

class BaseYoloDetector(ABC):
    """
    Simple abstract base class for YOLO detectors
    Defines basic interface for interaction with CameraController
    """
    
    def __init__(self, model_path: str, confidence_threshold: float = 0.5):
        self.model_path = model_path
        self.confidence_threshold = confidence_threshold
        self.is_detection_active = False
        self.latest_detections = []
    
    @abstractmethod
    def start_detection(self):
        """Start detection"""
        pass
    
    @abstractmethod
    def stop_detection(self):
        """Stop detection"""
        pass
    
    @abstractmethod
    def detect(self, frame_rgb: np.ndarray) -> np.ndarray:
        """
        Detect objects in RGB frame and return annotated frame
        """
        pass
    
    def get_latest_detections(self) -> List[Dict[str, Any]]:
        """Get latest detection results"""
        return self.latest_detections.copy()
    
    def is_active(self) -> bool:
        """Check if detection is active"""
        return self.is_detection_active
    
    def has_detections(self) -> bool:
        """Check if any objects detected"""
        return len(self.latest_detections) > 0