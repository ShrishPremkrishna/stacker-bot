#Modified by smartbuilds.io
#Date: 27.09.20
#Desc: This scrtipt script..

import cv2
# from imutils.video.pivideostream import PiVideoStream
# import imutils
import time
# import numpy as np

class VideoCamera(object):
    def __init__(self, flip = False):
        self.camera = cv2.VideoCapture(0)
        # self.vs = PiVideoStream().start()
        # self.flip = flip
        # time.sleep(2.0)

    def __del__(self):
        self.camera.release()
        cv2.destroyAllWindows()
        # self.vs.stop()

    # def flip_if_needed(self, frame):
    #     if self.flip:
    #         return np.flip(frame, 0)
    #     return frame

    def get_frame(self):
        _, frame = self.camera.read()
        # frame = self.vs.read()
        ret, jpeg = cv2.imencode('.jpg', frame)
        return jpeg.tobytes()