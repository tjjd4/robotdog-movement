import cv2
import numpy as np
from ultralytics import YOLO
import logging
from .BaseYoloDetector import BaseYoloDetector

logger = logging.getLogger(__name__)

class YoloV11Detector(BaseYoloDetector):
    def __init__(self, model_path: str = 'yolo11n.pt', confidence_threshold: float = 0.5):
        super().__init__(model_path, confidence_threshold)
        try:
            self.model = YOLO(model_path)
            logger.info(f"[YoloV11Detector] Initialized with model: {model_path}")
        except Exception as e:
            logger.error(f"[YoloV11Detector] Failed to load model: {e}")
            raise
    
    def start_detection(self):
        if not self.is_detection_active:
            self.is_detection_active = True
            logger.info("[YoloV11Detector] Object detection started")
    
    def stop_detection(self):
        if self.is_detection_active:
            self.is_detection_active = False
            self.latest_detections = []
            logger.info("[YoloV11Detector] Object detection stopped")
    
    def detect(self, frame_rgb: np.ndarray) -> np.ndarray:
        if not self.is_detection_active:
            return frame_rgb
            
        try:
            results = self.model(frame_rgb, verbose=False)
            
            detections = []
            if results and len(results) > 0:
                boxes = results[0].boxes
                if boxes is not None:
                    for box in boxes:
                        confidence = float(box.conf.cpu().numpy()[0])
                        if confidence >= self.confidence_threshold:
                            x1, y1, x2, y2 = box.xyxy.cpu().numpy()[0].astype(int)
                            class_idx = int(box.cls.cpu().numpy()[0])
                            class_name = self.model.names[class_idx]
                            
                            detection = {
                                'class': class_name,
                                'confidence': confidence,
                                'bbox': [x1, y1, x2, y2],
                                'center': [(x1 + x2) // 2, (y1 + y2) // 2]
                            }
                            detections.append(detection)
                
                self.latest_detections = detections
                annotated_frame = results[0].plot()
                return annotated_frame
            else:
                self.latest_detections = []
                return frame_rgb
                
        except Exception as e:
            logger.error(f"[YoloV11Detector] Detection error: {e}")
            return frame_rgb