import cv2
import numpy as np
from ultralytics import YOLO
import logging
from .BaseYoloDetector import BaseYoloDetector

logger = logging.getLogger(__name__)

class YoloWorldDetector(BaseYoloDetector):
    def __init__(self, model_path: str = 'yolov8s-world.pt', confidence_threshold: float = 0.5):
        super().__init__(model_path, confidence_threshold)
        try:
            self.model = YOLO(model_path)
            self.melee_weapon_classes = ["knife", "sword", "dagger"] 
            self.firearm_classes = ["gun", "handgun", "pistol", "revolver", "rifle", "assault rifle", "battle rifle", "sniper rifle", "carbine", "shotgun", "submachine gun", "machine gun", "bolt-action rifle", "lever-action rifle", "semi-automatic pistol", "automatic shotgun", "combat shotgun"]
            self.model.set_classes(self.melee_weapon_classes + self.firearm_classes)
            logger.info(f"[YoloWorldDetector] Initialized with classes: {self.melee_weapon_classes + self.firearm_classes}")
        except Exception as e:
            logger.error(f"[YoloWorldDetector] Failed to load model: {e}")
            raise
    
    def start_detection(self):
        if not self.is_detection_active:
            self.is_detection_active = True
            logger.info("[YoloWorldDetector] Detection started")
    
    def stop_detection(self):
        if self.is_detection_active:
            self.is_detection_active = False
            self.latest_detections = []
            logger.info("[YoloWorldDetector] Detection stopped")
    
    def detect(self, frame_rgb: np.ndarray) -> np.ndarray:
        if not self.is_detection_active:
            return frame_rgb
            
        try:
            results = self.model(frame_rgb, verbose=False)
            
            detections = []
            annotated_frame = frame_rgb.copy()
            
            if results and len(results) > 0 and results[0].boxes is not None:
                all_classes = self.melee_weapon_classes + self.firearm_classes
                for box in results[0].boxes:
                    confidence = float(box.conf.cpu().numpy()[0])
                    if confidence >= self.confidence_threshold:
                        x1, y1, x2, y2 = box.xyxy.cpu().numpy()[0].astype(int)
                        class_idx = int(box.cls.cpu().numpy()[0])
                        class_name = all_classes[class_idx]
                        
                        detection = {
                            'class': class_name,
                            'confidence': confidence,
                            'bbox': [x1, y1, x2, y2],
                            'center': [(x1 + x2) // 2, (y1 + y2) // 2]
                        }
                        detections.append(detection)
                        
                        # Determine weapon type and color
                        if class_name in self.melee_weapon_classes:
                            color = (0, 0, 255)  # Red for melee weapons
                            weapon_type = "melee"
                        elif class_name in self.firearm_classes:
                            color = (0, 165, 255)  # Orange for firearms
                            weapon_type = "firearm"
                        else:
                            color = (255, 255, 255)  # White for unknown
                            weapon_type = "unknown"
                        
                        cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, 2)
                        
                        label = f"{weapon_type}: {class_name} ({confidence:.2f})"
                        cv2.putText(annotated_frame, label, (x1, y1 - 10), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            
            self.latest_detections = detections
            return annotated_frame
            
        except Exception as e:
            logger.error(f"[YoloWorldDetector] Detection error: {e}")
            return frame_rgb