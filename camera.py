import cv2
import time
from edge_impulse_linux.image import ImageImpulseRunner
import os
import numpy as np
from PCA9685 import PCA9685
from roboclaw import Roboclaw



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
        
        self.cam_pulse = 1600
        self.cam_channel = 4
        self.cam_max = 2000
        self.cam_min = 1600
        self.pwm = PCA9685(0x40, debug=False)
        self.pwm.setPWMFreq(50)
        print("camera pulse being initiated at " + str(self.cam_pulse))
        self.pwm.setPWM(self.cam_channel, 0, self.cam_pulse)
        # self.pwm.setPWM(self.cam_channel, 0, 4096)

        self.speed = 20
        self.Lspeed = 35
        self.address = 0x80
        self.roboclaw = Roboclaw("/dev/serial0", 38400)
        result = self.roboclaw.Open()
        if result == 0:
            print('Unable to open port')
        print('Printing connection result - ' + str(result))
        print('Connection - ' + str(self.roboclaw._port.is_open))

        self.next_action = self.now() + 1000

    def now(self):
        return round(time.time() * 1000)

    def lower_camera(self):
        print("lower_camera")
        print("Cam pulse at - " + str(self.cam_pulse))
        if (self.cam_pulse < self.cam_max) :
            for i in range(self.cam_pulse, self.cam_pulse + 100, 10):  
                self.pwm.setServoPulse(self.cam_channel, i)   
                time.sleep(0.02) 
            print("Cam pulse being set at - " + str(self.cam_pulse))
            self.cam_pulse = self.cam_pulse + 100 
            return True
        else:
            return False

    def raise_camera(self):
        print("raise_camera")


    def move_chassis_up(self):
        self.roboclaw.BackwardM1(0x80,self.speed)
        self.roboclaw.BackwardM2(0x80,self.speed)
        self.roboclaw.BackwardM1(0x81,self.speed)
        self.roboclaw.BackwardM2(0x81,self.speed)
        time.sleep(0.25)
        self.roboclaw.ForwardM1(0x80,0)
        self.roboclaw.ForwardM2(0x80,0)
        self.roboclaw.ForwardM1(0x81,0)
        self.roboclaw.ForwardM2(0x81,0)

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

        if (self.next_action == self.now()):
            shoe_found = False
            if "bounding_boxes" in res["result"].keys():
                print('Found %d bounding boxes (%d ms.)' % (len(res["result"]["bounding_boxes"]), res['timing']['dsp'] + res['timing']['classification']))
                for bb in res["result"]["bounding_boxes"]:
                    if (bb['label'] == 'shoe'):
                        shoe_found = True
                        logList.append('%s (%.2f): x=%d y=%d w=%d h=%d' % (bb['label'], bb['value'], bb['x'], bb['y'], bb['width'], bb['height']))
                        cropped = cv2.rectangle(cropped, (bb['x'], bb['y']), (bb['x'] + bb['width'], bb['y'] + bb['height']), (255, 0, 0), 2)
                        cropped = cv2.putText(cropped, bb['label'], (bb['x'], bb['y'] + 25), font, 1, (255, 0, 0), 2, cv2.LINE_AA)
                        
                        if(bb['y'] > 50):
                            if (self.cam_pulse < self.cam_max):
                                self.lower_camera()
                                logList.append("Lower camera angle")
                            else:
                                logList.append("Proximity Reached")
                        else:
                            self.move_chassis_up()
                            logList.append("Moving chassis up")
                        break
            
            if (not shoe_found):
                if (self.cam_pulse < self.cam_max):
                    self.lower_camera()
                    logList.append("Lower camera angle")
                else:
                    logList.append("Proximity Reached")
            self.next_action = self.now() + 2000
        else:
            if "bounding_boxes" in res["result"].keys():
                print('Found %d bounding boxes (%d ms.)' % (len(res["result"]["bounding_boxes"]), res['timing']['dsp'] + res['timing']['classification']))
                for bb in res["result"]["bounding_boxes"]:
                    logList.append('%s (%.2f): x=%d y=%d w=%d h=%d' % (bb['label'], bb['value'], bb['x'], bb['y'], bb['width'], bb['height']))
                    cropped = cv2.rectangle(cropped, (bb['x'], bb['y']), (bb['x'] + bb['width'], bb['y'] + bb['height']), (255, 0, 0), 2)
                    cropped = cv2.putText(cropped, bb['label'], (bb['x'], bb['y'] + 25), font, 1, (255, 0, 0), 2, cv2.LINE_AA)
        logs = np.full((800,800,3), 200, dtype=np.uint8)
        for i, log in enumerate(logList):
            cv2.putText(logs, log, (10, (i + 1) * 30), font, 1, (10, 10, 10), 1, cv2.LINE_AA)
        canvas = np.concatenate((self.scaleout(cropped), logs), axis=1)
        ret, jpeg = cv2.imencode('.jpg', canvas)
        time.sleep(1)
        return jpeg.tobytes()