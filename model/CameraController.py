import cv2
from picamera2 import Picamera2
from ultralytics import YOLO

class CameraController:
    def __init__(self, model_path='yolo11n_ncnn_model', resolution=(640, 480)):
        self.camera = Picamera2()
        self.camera.configure(
            self.camera.create_preview_configuration(
                main={"format": 'XRGB8888', "size": resolution}
            )
        )
        self.camera.preview_configuration.align()
        self.camera.configure("preview")
        self.model = YOLO(model_path)

        self.is_camera_active = False
        self.is_detection_active = False

    def start_camera(self):
        if not self.is_camera_active:
            self.camera.start()
            self.is_camera_active = True

    def stop_camera(self):
        if self.is_camera_active:
            self.camera.stop()
            self.is_camera_active = False

    def start_detection(self):
        self.is_detection_active = True

    def stop_detection(self):
        self.is_detection_active = False

    def generate_frames(self):
        """產生處理後的 JPEG 串流畫面 (適用於 MJPEG 串流輸出)"""
        while self.is_camera_active:
            frame = self.camera.capture_array()
            if self.is_detection_active:
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                results = self.model(frame_rgb)
                annotated_frame = results[0].plot()

                # FPS overlay
                inference_time = results[0].speed.get('inference', 0)
                fps = 1000.0 / inference_time if inference_time > 0 else 0
                text = f'FPS: {fps:.1f}'
                font = cv2.FONT_HERSHEY_SIMPLEX
                text_size = cv2.getTextSize(text, font, 1, 2)[0]
                text_x = annotated_frame.shape[1] - text_size[0] - 10
                text_y = text_size[1] + 10
                cv2.putText(annotated_frame, text, (text_x, text_y), font, 1, (255, 255, 255), 2, cv2.LINE_AA)
            else:
                annotated_frame = frame

            ret, buffer = cv2.imencode('.jpg', annotated_frame)
            frame_jpeg = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_jpeg + b'\r\n')

    def is_active(self) -> bool:
        return self.is_camera_active
