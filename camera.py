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
        scale_percent = 60 # percent of original size
        width = int(frame.shape[1] * scale_percent / 100)
        height = int(frame.shape[0] * scale_percent / 100)
        dim = (width, height)
        
        # resize image
        resized_frame = cv2.resize(frame, dim, interpolation = cv2.INTER_AREA)
        ret, jpeg = cv2.imencode('.jpg', resized_frame)
        return jpeg.tobytes()