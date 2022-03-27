import cv2
import time
from edge_impulse_linux.image import ImageImpulseRunner
import os
import numpy as np
from PCA9685 import PCA9685
from roboclaw import Roboclaw



class VideoCamera(object):
    def __init__(self):
        dir_path = os.path.dirname(os.path.realpath(__file__))

        modelfile1 = os.path.join(dir_path, 'sb-model-1.eim')
        self.runner1 = ImageImpulseRunner(modelfile1)
        model_info1 = self.runner1.init()
        print('Loaded runner1 for "' + model_info1['project']['owner'] + ' / ' + model_info1['project']['name'] + '"')
        
        modelfile2= os.path.join(dir_path, 'sb-model-2a.eim')
        self.runner2 = ImageImpulseRunner(modelfile2)
        model_info2 = self.runner2.init()
        print('Loaded runner2 for "' + model_info2['project']['owner'] + ' / ' + model_info2['project']['name'] + '"')

        # self.camera = cv2.VideoCapture(0)
        
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
        self.cam_max = 2400
        self.cam_min = 2000
        self.cam_pulse = self.cam_min
        self.pwm = PCA9685(0x40, debug=False)
        self.pwm.setPWMFreq(50)
        print("camera pulse being initiated at " + str(self.cam_pulse))
        self.pwm.setServoPulse(self.cam_channel, self.cam_pulse)

        self.end_model1_probe = False
        self.end_model2_probe = False
        self.frame_count = 0
        self.retrys = 3

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

    def move_chassis_around(self):
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
        self.roboclaw.ForwardM1(0x80,self.speed)
        self.roboclaw.BackwardM2(0x80,self.speed)
        self.roboclaw.ForwardM1(0x81,self.speed)
        self.roboclaw.BackwardM2(0x81,self.speed)
        time.sleep(0.35)
        self.roboclaw.ForwardM1(0x80,0)
        self.roboclaw.ForwardM2(0x80,0)
        self.roboclaw.ForwardM1(0x81,0)
        self.roboclaw.ForwardM2(0x81,0)
        time.sleep(0.15)

    def __del__(self):
        # self.camera.release()
        cv2.destroyAllWindows()
        self.pwm.setServoPulse(self.cam_channel, self.cam_max)
        if (self.runner1):
            self.runner1.stop()
        if (self.runner2):
            self.runner2.stop()

    def scalein_crop_img(self, img):
        # scale_percent = 67 
        # width = int(img.shape[1] * scale_percent / 100)
        # height = int(img.shape[0] * scale_percent / 100)
        # dim = (width, height)
        # resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
        # return resized[0:320, 51:371]
        pts1 = np.float32([[0,0],[640,0],[0,480],[640,480]])
        pts2 = np.float32([[0,0],[320,0],[0,320],[320,320]])
        M = cv2.getPerspectiveTransform(pts1,pts2)
        frame = cv2.warpPerspective(img,M,(320,320))
        return frame

    def scalein_crop_img2(self, img):
        # dim = (96, 96)
        # resized = cv2.resize(img[0:480, 80:560], dim, interpolation = cv2.INTER_AREA)
        # return resized
        pts1 = np.float32([[0,0],[640,0],[0,480],[640,480]])
        pts2 = np.float32([[0,0],[96,0],[0,96],[96,96]])
        M = cv2.getPerspectiveTransform(pts1,pts2)
        frame = cv2.warpPerspective(img,M,(96,96))
        return frame

    def scaleout(self, img):
        scale_percent = 250
        width = int(img.shape[1] * scale_percent / 100)
        height = int(img.shape[0] * scale_percent / 100)
        dim = (width, height)
        return cv2.resize(img, dim, interpolation = cv2.INTER_AREA)

    def move_to_shoe(self):
        print("Move to Shoe")
        camera = cv2.VideoCapture(0)
        font = cv2.FONT_HERSHEY_COMPLEX_SMALL
        ret, img = camera.read()
        cropped = self.scalein_crop_img(img)
        logList = []
        logList.append("frame count" + str(self.frame_count))
        self.frame_count += 1
        features, cropped1 = self.runner1.get_features_from_image(cropped)
        res = self.runner1.classify(features)
        print(res)
        # logList.append("model 1 prediction" + str(res))
        
        if len(res["result"]["bounding_boxes"]) > 0:
            self.retrys = 3
            bb = res["result"]["bounding_boxes"][0]
            if (bb['label'] == 'shoe'):
                cropped = cv2.rectangle(cropped, (bb['x'], bb['y']), (bb['x'] + bb['width'], bb['y'] + bb['height']), (255, 0, 0), 2)
                cropped = cv2.putText(cropped, bb['label'], (bb['x'], bb['y'] + 25), font, 1, (255, 0, 0), 2, cv2.LINE_AA)
                if(bb['y'] > 180):
                    if (self.cam_pulse < self.cam_max):
                        self.lower_camera()
                        logList.append("Lower camera angle")
                        print("Lower camera angle")
                    else:
                        logList.append("Proximity Reached")
                        print("Proximity Reached")
                        self.end_model1_probe = True
                elif(bb['y'] < 180):
                    self.move_chassis_up()
                    logList.append("Moving chassis up")
                    print("Moving chassis up")
                    if (bb['x'] > 100):
                        self.move_chassis_right()
                        logList.append("Moving chassis right")
                        print("Moving chassis right")
                    elif (bb['x'] < 60):
                        self.move_chassis_left()
                        logList.append("Moving chassis left")
                        print("Moving chassis left")
        elif self.retrys > 0:
            self.retrys -= 1
        elif (self.cam_pulse < self.cam_max):
            self.lower_camera()
        else:
            logList.append("Unable to find shoe")
            print("Unable to find shoe")
            self.end_model1_probe = True


        logs = np.full((800,800,3), 200, dtype=np.uint8)
        for i, log in enumerate(logList):
            cv2.putText(logs, log, (10, (i + 1) * 30), font, 1, (10, 10, 10), 1, cv2.LINE_AA)
        canvas = np.concatenate((self.scaleout(cropped), logs), axis=1)
        cv2.imshow('camera-feed', canvas)
        if self.end_model1_probe == True:
            if cv2.waitKey(5000) == 27: 
                print("end wait key")
        else:
            if cv2.waitKey(300) == 27: 
                print("end wait key")
        camera.release()

    def move_around_shoe(self):
        print("Move around Shoe")
        self.pwm.setServoPulse(self.cam_channel, self.cam_max)
        camera = cv2.VideoCapture(0)
        font = cv2.FONT_HERSHEY_COMPLEX_SMALL
        ret, img = camera.read()
        print(img.shape)
        cropped = self.scalein_crop_img2(img)
        print(cropped.shape)
        logList = []
        logList.append("frame count" + str(self.frame_count))
        self.frame_count += 1
        features, cropped1 = self.runner2.get_features_from_image(cropped)
        res = self.runner2.classify(features)
        print(res)
        # logList.append("model 1 prediction" + str(res))
        logList.append("In Position Probalility" + str(res["result"]["classification"]["in-position"]))
        logList.append("Out Position Probalility" + str(res["result"]["classification"]["out-of-position"]))
        
        if res["result"]["classification"]["in-position"] > res["result"]["classification"]["out-of-position"]:
            logList.append("In Position")
            print("In Position")
            self.end_model2_probe = True
        else:
            logList.append("Out of Position")
            print("Out of Position")
            self.move_chassis_around()

        logs = np.full((480,600,3), 200, dtype=np.uint8)
        for i, log in enumerate(logList):
            cv2.putText(logs, log, (10, (i + 1) * 30), font, 1, (10, 10, 10), 1, cv2.LINE_AA)
        canvas = np.concatenate((img, logs), axis=1)
        cv2.imshow('camera-feed', canvas)
        if self.end_model2_probe == True:
            if cv2.waitKey(5000) == 27: 
                print("end wait key")
        else:
            if cv2.waitKey(300) == 27: 
                print("end wait key")
        camera.release()

if __name__ == "__main__":
    pi_camera = VideoCamera()
    while(pi_camera.end_model1_probe == False):
        frame = pi_camera.move_to_shoe()
    # while(pi_camera.end_model2_probe == False):
    #     frame = pi_camera.move_around_shoe()



