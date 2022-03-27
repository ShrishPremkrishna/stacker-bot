import cv2
import time
from edge_impulse_linux.image import ImageImpulseRunner
import os
import numpy as np
from PCA9685 import PCA9685
from roboclaw import Roboclaw



class VideoCamera(object):
    def __init__(self):
        model = 'sb-model-1.eim'
        dir_path = os.path.dirname(os.path.realpath(__file__))
        modelfile = os.path.join(dir_path, model)
        print('MODEL: ' + modelfile)
        self.runner = ImageImpulseRunner(modelfile)
        model_info = self.runner.init()
        print('Loaded runner for "' + model_info['project']['owner'] + ' / ' + model_info['project']['name'] + '"')
        self.labels = model_info['model_parameters']['labels']
        self.camera = cv2.VideoCapture(0)
        
        self.speed = 20
        self.Lspeed = 35
        self.address = 0x80
        self.roboclaw = Roboclaw("/dev/serial0", 38400)
        result = self.roboclaw.Open()
        if result == 0:
            print('Unable to open port')
        print('Printing connection result - ' + str(result))
        print('Connection - ' + str(self.roboclaw._port.is_open))

        self.cam_channel = 4
        self.cam_max = 2800
        self.cam_min = 2000
        self.cam_pulse = self.cam_min
        self.pwm = PCA9685(0x40, debug=False)
        self.pwm.setPWMFreq(50)
        print("camera pulse being initiated at " + str(self.cam_pulse))
        self.pwm.setServoPulse(self.cam_channel, self.cam_pulse)

        self.next_action = self.now() 
        self.proximity_reached = False

    def now(self):
        return round(time.time() * 1000)

    def lower_camera(self):
        print("lower_camera")
        print("Cam pulse at - " + str(self.cam_pulse))
        if (self.cam_pulse < self.cam_max) :
            for i in range(self.cam_pulse, self.cam_pulse + 200, 10):  
                self.pwm.setServoPulse(self.cam_channel, i)   
                time.sleep(0.02) 
            print("Cam pulse being set at - " + str(self.cam_pulse))
            self.cam_pulse = self.cam_pulse + 200 
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
        time.sleep(0.35)
        self.roboclaw.ForwardM1(0x80,0)
        self.roboclaw.ForwardM2(0x80,0)
        self.roboclaw.ForwardM1(0x81,0)
        self.roboclaw.ForwardM2(0x81,0)
        time.sleep(0.15)

    def move_chassis_left(self):  
        self.roboclaw.BackwardM1(0x80,self.speed)
        self.roboclaw.ForwardM2(0x80,self.speed)
        self.roboclaw.ForwardM1(0x81,self.speed)
        self.roboclaw.BackwardM2(0x81,self.speed)
        time.sleep(0.35)
        self.roboclaw.ForwardM1(0x80,0)
        self.roboclaw.ForwardM2(0x80,0)
        self.roboclaw.ForwardM1(0x81,0)
        self.roboclaw.ForwardM2(0x81,0)
        time.sleep(0.15)

    def move_chassis_right(self):
        self.roboclaw.ForwardM1(0x80,self.speed)
        self.roboclaw.BackwardM2(0x80,self.speed)
        self.roboclaw.BackwardM1(0x81,self.speed)
        self.roboclaw.ForwardM2(0x81,self.speed)
        time.sleep(0.35)
        self.roboclaw.ForwardM1(0x80,0)
        self.roboclaw.ForwardM2(0x80,0)
        self.roboclaw.ForwardM1(0x81,0)
        self.roboclaw.ForwardM2(0x81,0)
        time.sleep(0.15)

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

        if (self.next_action < self.now()):
            shoe_found = False
            if "bounding_boxes" in res["result"].keys():
                print('Found %d bounding boxes (%d ms.)' % (len(res["result"]["bounding_boxes"]), res['timing']['dsp'] + res['timing']['classification']))
                # for bb in res["result"]["bounding_boxes"]:
                bb = res["result"]["bounding_boxes"][1]
                if (bb['label'] == 'shoe'):
                    shoe_found = True
                    logList.append('%s (%.2f): x=%d y=%d w=%d h=%d' % (bb['label'], bb['value'], bb['x'], bb['y'], bb['width'], bb['height']))
                    cropped = cv2.rectangle(cropped, (bb['x'], bb['y']), (bb['x'] + bb['width'], bb['y'] + bb['height']), (255, 0, 0), 2)
                    cropped = cv2.putText(cropped, bb['label'], (bb['x'], bb['y'] + 25), font, 1, (255, 0, 0), 2, cv2.LINE_AA)
                    
                    if(bb['y'] > 100):
                        if (self.cam_pulse < self.cam_max):
                            self.lower_camera()
                            logList.append("Lower camera angle")
                        else:
                            logList.append("Proximity Reached")
                            self.proximity_reached = True
                    elif(bb['y'] < 100):
                        self.move_chassis_up()
                        logList.append("Moving chassis up")
                        if (bb['x'] > 100):
                            self.move_chassis_right()
                            logList.append("Moving chassis right")
                        elif (bb['x'] < 30):
                            self.move_chassis_left()
                            logList.append("Moving chassis left")
                        
            
            if (not shoe_found):
                if (self.cam_pulse < self.cam_max):
                    self.lower_camera()
                    logList.append("Lower camera angle")
                else:
                    logList.append("Proximity Reached")
                    self.proximity_reached = True
                    self.cam_pulse = self.cam_min
            self.next_action = self.now() + 5
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
        cv2.imshow('camera-feed', canvas)
        time.sleep(1)


if __name__ == "__main__":
    pi_camera = VideoCamera()
    while(pi_camera.next_action < pi_camera.now() and pi_camera.proximity_reached == False):
        frame = pi_camera.get_frame()

