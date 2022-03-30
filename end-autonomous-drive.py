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
        self.gripper_max = 2000
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
        self.frame_count = 0
        self.retrys = 3

    
    def barlift_down(self):
        print("Barlift pulse at - " + str(self.barlift_pulse))
        if (self.barlift_pulse > self.barlift_min) :
            print("Barlift pulse being set at - " + str(self.barlift_pulse))
            for i in range(self.barlift_pulse, self.barlift_pulse - 200, -10):  
                self.pwm.setServoPulse(self.barlift_channel, i)   
                time.sleep(0.02) 
            self.barlift_pulse = self.barlift_pulse - 200
        if (self.barlift_pulse <= self.barlift_min + 100):
            self.pwm.setPWM(self.barlift_channel, 0, 4096)

    def __del__(self):
        # self.camera.release()
        self.roboclaw.ForwardM1(0x80,0)
        self.roboclaw.ForwardM2(0x80,0)
        self.roboclaw.ForwardM1(0x81,0)
        self.roboclaw.ForwardM2(0x81,0)
        time.sleep(0.15)
        
        cv2.destroyAllWindows()
        self.pwm.setServoPulse(self.cam_channel, self.cam_max)
        self.pwm.setPWM(self.gripper_channel, 0, 4096)
        self.barlift_down()
        if (self.runner1):
            self.runner1.stop()
        if (self.runner2):
            self.runner2.stop()

    

if __name__ == "__main__":

    sbot = VideoCamera()
    del sbot



