import cv2
import time
from edge_impulse_linux.image import ImageImpulseRunner
import os
import numpy as np



class VideoCamera(object):
    def __init__(self):
        model = 'obj-det.eim'
        dir_path = os.path.dirname(os.path.realpath(__file__))
        modelfile = os.path.join(dir_path, model)
        print('MODEL: ' + modelfile)
        self.runner = ImageImpulseRunner(modelfile)
        model_info = self.runner.init()
        print('Loaded runner for "' + model_info['project']['owner'] + ' / ' + model_info['project']['name'] + '"')
        self.labels = model_info['model_parameters']['labels']
        self.camera = cv2.VideoCapture(0)

    def __del__(self):
        self.camera.release()
        cv2.destroyAllWindows()
        if (self.runner):
            self.runner.stop()

    def scalein_crop_img(self, img):
        scale_percent = 67 
        width = int(img.shape[1] * scale_percent / 100)
        height = int(img.shape[0] * scale_percent / 100)
        dim = (width, height)
        resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
        return resized[0:320, 51:371]

    def scaleout(self, img):
        scale_percent = 250
        width = int(img.shape[1] * scale_percent / 100)
        height = int(img.shape[0] * scale_percent / 100)
        dim = (width, height)
        return cv2.resize(img, dim, interpolation = cv2.INTER_AREA)

    def get_frame(self):
        time.sleep(0.1)
        font = cv2.FONT_HERSHEY_COMPLEX_SMALL
        ret, img = self.camera.read()
        features, cropped = self.runner.get_features_from_image(img)
        res = self.runner.classify(features)
        print(res)
        cropped = self.scalein_crop_img(img)
        logList = []


        if "bounding_boxes" in res["result"].keys():
            print('Found %d bounding boxes (%d ms.)' % (len(res["result"]["bounding_boxes"]), res['timing']['dsp'] + res['timing']['classification']))
            for bb in res["result"]["bounding_boxes"]:
                if (bb['label'] == 'shoe'):
                    logList.append('%s (%.2f): x=%d y=%d w=%d h=%d' % (bb['label'], bb['value'], bb['x'], bb['y'], bb['width'], bb['height']))
                    cropped = cv2.rectangle(cropped, (bb['x'], bb['y']), (bb['x'] + bb['width'], bb['y'] + bb['height']), (255, 0, 0), 2)
                    cropped = cv2.putText(cropped, bb['label'], (bb['x'], bb['y'] + 25), font, 1, (255, 0, 0), 2, cv2.LINE_AA)
                    break
        
        logs = np.full((800,800,3), 200, dtype=np.uint8)
        for i, log in enumerate(logList):
            cv2.putText(logs, log, (10, (i + 1) * 30), font, 1, (10, 10, 10), 1, cv2.LINE_AA)
        canvas = np.concatenate((self.scaleout(cropped), logs), axis=1)
        ret, jpeg = cv2.imencode('.jpg', canvas)
        time.sleep(1)
        return jpeg.tobytes()