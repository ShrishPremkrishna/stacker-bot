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
        
        modelfile2= os.path.join(dir_path, 'sb-model-2b.eim')
        self.runner2 = ImageImpulseRunner(modelfile2)
        model_info2 = self.runner2.init()
        print('Loaded runner2 for "' + model_info2['project']['owner'] + ' / ' + model_info2['project']['name'] + '"')

        modelfile3= os.path.join(dir_path, 'sb-model-3a.eim')
        self.runner3 = ImageImpulseRunner(modelfile3)
        model_info3 = self.runner3.init()
        print('Loaded runner3 for "' + model_info3['project']['owner'] + ' / ' + model_info3['project']['name'] + '"')

        # self.camera = cv2.VideoCapture(0)
        
        self.speed = 20
        self.Lspeed = 36
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
        
        self.gripper_channel = 0
        self.gripper_max = 2300
        self.gripper_min = 1200
        self.gripper_pulse = self.gripper_max
        print("gripper pulse being initiated at " + str(self.gripper_pulse))
        self.pwm.setServoPulse(self.gripper_channel, self.gripper_pulse)
        
        self.barlift_channel = 2
        self.barlift_max = 2200
        self.barlift_min = 600
        self.barlift_pulse = self.barlift_min
        print("barlift pulse being initiated at " + str(self.gripper_pulse))
        self.pwm.setServoPulse(self.barlift_channel, self.barlift_pulse)

        self.end_model1_probe = False
        self.end_model2_probe = False
        self.end_model3_probe = False
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
        self.pwm.setServoPulse(self.cam_channel, 1400)

    def rotate_chassis_right(self):
        print("RotateRight")
        self.roboclaw.BackwardM1(0x80,self.speed)
        self.roboclaw.ForwardM2(0x80,self.speed)
        self.roboclaw.BackwardM1(0x81,self.speed)
        self.roboclaw.ForwardM2(0x81,self.speed)
        time.sleep(0.2)
        self.roboclaw.ForwardM1(0x80,0)
        self.roboclaw.ForwardM2(0x80,0)
        self.roboclaw.ForwardM1(0x81,0)
        self.roboclaw.ForwardM2(0x81,0)
        time.sleep(0.15)

    def rotate_chassis_left(self):
        print("RotateLeft")
        self.roboclaw.ForwardM1(0x80,self.speed)
        self.roboclaw.BackwardM2(0x80,self.speed)
        self.roboclaw.ForwardM1(0x81,self.speed)
        self.roboclaw.BackwardM2(0x81,self.speed)
        time.sleep(0.2)
        self.roboclaw.ForwardM1(0x80,0)
        self.roboclaw.ForwardM2(0x80,0)
        self.roboclaw.ForwardM1(0x81,0)
        self.roboclaw.ForwardM2(0x81,0)
        time.sleep(0.15)

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

    def move_chassis_down(self):
        self.roboclaw.ForwardM1(0x80,self.speed)
        self.roboclaw.ForwardM2(0x80,self.speed)
        self.roboclaw.ForwardM1(0x81,self.speed)
        self.roboclaw.ForwardM2(0x81,self.speed)
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
        time.sleep(0.50)
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

    def stop_chassis(self):
        self.roboclaw.ForwardM1(0x80,0)
        self.roboclaw.ForwardM2(0x80,0)
        self.roboclaw.ForwardM1(0x81,0)
        self.roboclaw.ForwardM2(0x81,0)
        self.roboclaw.ForwardM1(0x82,0)
        self.roboclaw.ForwardM2(0x82,0)
        time.sleep(0.15)

    def linearslide_up(self, duration):
        print("Linear slide Up")
        self.roboclaw.ForwardM1(0x82,self.Lspeed)
        print("#1")
        time.sleep(duration)
        print("#2")
        self.roboclaw.ForwardM1(0x82,0)
        print("#3")
        time.sleep(0.15)
        print("#4")

    def linearslide_down(self, duration):
        print("Linear slide Down")
        self.roboclaw.BackwardM1(0x82,self.Lspeed)
        time.sleep(duration)
        self.roboclaw.ForwardM1(0x82,0)
        time.sleep(0.15)

    def gripper_close(self):
        print("On X Press")
        if (self.gripper_pulse > self.gripper_min) :
            self.gripper_pulse = self.gripper_pulse - 100
            self.pwm.setServoPulse(self.gripper_channel, self.gripper_pulse) 
            time.sleep(0.02)
        
    def gripper_open(self):
        print("On Triangle Press")
        if (self.gripper_pulse < self.gripper_max) :
            self.gripper_pulse = self.gripper_pulse + 200
            self.pwm.setServoPulse(self.gripper_channel, self.gripper_pulse) 
            time.sleep(0.02)

    def barlift_up(self):
        print("Barlift pulse at - " + str(self.barlift_pulse))
        for i in range(self.barlift_pulse, self.barlift_max, 20):  
            self.pwm.setServoPulse(self.barlift_channel, i)   
            time.sleep(0.02) 
        print("Barlift pulse being set at - " + str(self.barlift_pulse))
        self.barlift_pulse = self.barlift_max 

    def barlift_down(self):
        print("Barlift pulse at - " + str(self.barlift_pulse))
        print("Barlift pulse being set at - " + str(self.barlift_pulse))
        for i in range(self.barlift_pulse, self.barlift_min, -20):  
            self.pwm.setServoPulse(self.barlift_channel, i)   
            time.sleep(0.02) 
        self.barlift_pulse = self.barlift_min
        self.pwm.setPWM(self.barlift_channel, 0, 4096)

    def __del__(self):
        # self.camera.release()
        cv2.destroyAllWindows()
        self.stop_chassis()
        self.pwm.setServoPulse(self.cam_channel, self.cam_max)
        self.pwm.setPWM(self.gripper_channel, 0, 4096)
        self.barlift_down()
        if (self.runner1):
            self.runner1.stop()
        if (self.runner2):
            self.runner2.stop()

    def scalein_crop_img(self, img):
        pts1 = np.float32([[0,0],[640,0],[0,480],[640,480]])
        pts2 = np.float32([[0,0],[320,0],[0,320],[320,320]])
        M = cv2.getPerspectiveTransform(pts1,pts2)
        frame = cv2.warpPerspective(img,M,(320,320))
        return frame

    def scalein_crop_img2(self, img):
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
        logList.append("*** MODEL 1 *** ")
        logList.append("frame count --- " + str(self.frame_count))
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
                if(bb['y'] > 150):
                    if (self.cam_pulse < self.cam_max):
                        self.lower_camera()
                        logList.append("Lower camera angle")
                        print("Lower camera angle")
                    else:
                        logList.append("Proximity Reached")
                        print("Proximity Reached")
                        self.end_model1_probe = True
                elif(bb['y'] < 150):
                    self.move_chassis_up()
                    logList.append("Moving chassis up")
                    print("Moving chassis up")
                    if (bb['x'] > 100):
                        self.move_chassis_right()
                        logList.append("Moving chassis right")
                        print("Moving chassis right")
                    elif (bb['x'] < 10):
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
        logList.append("*** MODEL 2 *** ")
        logList.append("frame count --- " + str(self.frame_count))
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
            self.move_chassis_around()
            self.move_chassis_around()
            self.move_chassis_around()
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

    def move_to_rack(self):
        print("Move to Rack")
        self.raise_camera()
        camera = cv2.VideoCapture(0)
        font = cv2.FONT_HERSHEY_COMPLEX_SMALL
        ret, img = camera.read()
        cropped = self.scalein_crop_img(img)
        logList = []
        logList.append("*** MODEL 3 *** ")
        logList.append("frame count --- " + str(self.frame_count))
        self.frame_count += 1
        features, cropped1 = self.runner3.get_features_from_image(cropped)
        res = self.runner3.classify(features)
        print(res)
        # logList.append("model 1 prediction" + str(res))
        
        if len(res["result"]["bounding_boxes"]) > 0:
            self.retrys = 3
            bb = res["result"]["bounding_boxes"][0]
            cropped = cv2.rectangle(cropped, (bb['x'], bb['y']), (bb['x'] + bb['width'], bb['y'] + bb['height']), (255, 0, 0), 2)
            cropped = cv2.putText(cropped, bb['label'], (bb['x'], bb['y'] + 25), font, 1, (255, 0, 0), 2, cv2.LINE_AA)
            if(bb['width'] * bb['height'] > 35000):
                logList.append("Proximity Reached")
                print("Proximity Reached")
                self.end_model3_probe = True
            else:
                self.move_chassis_up()
                logList.append("Moving chassis up")
                print("Moving chassis up")
                if (bb['x'] > 200):
                    self.move_chassis_right()
                    logList.append("Moving chassis right")
                    print("Moving chassis right")
                elif (bb['x'] < 5):
                    self.move_chassis_left()
                    logList.append("Moving chassis left")
                    print("Moving chassis left")
        elif self.retrys > 0:
            self.rotate_chassis_left()
            logList.append("Rotate chassis left")
            print("Rotate chassis left")
        else:
            logList.append("Unable to find rack")
            print("Unable to find rack")
            self.end_model3_probe = True
        logs = np.full((800,800,3), 200, dtype=np.uint8)
        for i, log in enumerate(logList):
            cv2.putText(logs, log, (10, (i + 1) * 30), font, 1, (10, 10, 10), 1, cv2.LINE_AA)
        canvas = np.concatenate((self.scaleout(cropped), logs), axis=1)
        cv2.imshow('camera-feed', canvas)
        if self.end_model3_probe == True:
            if cv2.waitKey(5000) == 27: 
                print("end wait key")
        else:
            if cv2.waitKey(300) == 27: 
                print("end wait key")
        camera.release()

if __name__ == "__main__":

    sbot = VideoCamera()
    sbot.linearslide_up(2.5)    
    while(sbot.end_model1_probe == False):
        sbot.move_to_shoe()
    while(sbot.end_model2_probe == False):
        sbot.move_around_shoe()
    sbot.linearslide_down(2.5)
    sbot.pwm.setServoPulse(sbot.gripper_channel, sbot.gripper_min)
    sbot.linearslide_up(7.5)
    sbot.barlift_up()
    sbot.retrys = 20
    while(sbot.end_model3_probe == False):
        sbot.move_to_rack()
    sbot.pwm.setServoPulse(sbot.gripper_channel, sbot.gripper_max)
    time.sleep(1)
    sbot.move_chassis_down()
    sbot.move_chassis_down()
    sbot.move_chassis_down()
    sbot.move_chassis_down()
    sbot.move_chassis_down()
    time.sleep(5)
    del sbot



