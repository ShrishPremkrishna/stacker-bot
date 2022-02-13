import cv2
import time


class VideoCamera(object):
    def __init__(self):
        self.camera = cv2.VideoCapture(0)

    def __del__(self):
        self.camera.release()
        cv2.destroyAllWindows()

    def get_frame(self):
        _, frame = self.camera.read()
        ret, jpeg = cv2.imencode('.jpg', frame)
        return jpeg.tobytes()