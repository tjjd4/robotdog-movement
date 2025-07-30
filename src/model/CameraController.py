import cv2
import logging
from picamera2 import Picamera2
from .BaseYoloDetector import BaseYoloDetector
from typing import Optional

logger = logging.getLogger(__name__)

class CameraController:
    def __init__(self, detector: Optional[BaseYoloDetector] = None, resolution=(640, 480)):
        self.camera = Picamera2()
        self.camera.configure(
            self.camera.create_preview_configuration(
                main={"format": 'XRGB8888', "size": resolution}
            )
        )
        self.camera.preview_configuration.align()
        self.camera.configure("preview")
        self.detector = detector

        self.is_camera_active = False

    def start_camera(self):
        if not self.is_camera_active:
            self.camera.start()
            self.is_camera_active = True

    def stop_camera(self):
        if self.is_camera_active:
            self.camera.stop()
            self.is_camera_active = False

    def start_detection(self):
        if self.detector:
            self.detector.start_detection()

    def stop_detection(self):
        if self.detector:
            self.detector.stop_detection()
    
    def set_detector(self, detector: BaseYoloDetector):
        self.detector = detector

    def generate_frames(self):
        """產生處理後的 JPEG 串流畫面 (適用於 MJPEG 串流輸出)"""
        while self.is_camera_active:
            frame = self.camera.capture_array()
            
            if self.detector and self.detector.is_active():
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                annotated_frame = self.detector.detect(frame_rgb)
                annotated_frame = cv2.cvtColor(annotated_frame, cv2.COLOR_RGB2BGR)
            else:
                annotated_frame = frame

            ret, buffer = cv2.imencode('.jpg', annotated_frame)
            frame_jpeg = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_jpeg + b'\r\n')

    def is_active(self) -> bool:
        return self.is_camera_active
