from pyPS4Controller.controller import Controller
from roboclaw import Roboclaw
from PCA9685 import PCA9685
from time import sleep
import time
import cv2
import calendar 



class MyController(Controller):
    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)

        self.cam_pulse = 2200
        self.cam_channel = 4
        self.cam_max = 2200
        self.cam_min = 1600
        self.pwm = PCA9685(0x40, debug=False)
        self.pwm.setPWMFreq(50)
        print("camera pulse being initiated at " + str(self.cam_pulse))
        self.pwm.setServoPulse(self.cam_channel, self.cam_pulse)

        self.motorSpeed = 20
        self.motorRest = 0

        self.cam = cv2.VideoCapture(0)

    def __del__(self):
        self.cam.release()

    def stopChassis(self):
        sleep(0.5)
        roboclaw.ForwardM1(0x80,self.motorRest)
        roboclaw.ForwardM2(0x80,self.motorRest)
        roboclaw.ForwardM1(0x81,self.motorRest)
        roboclaw.ForwardM2(0x81,self.motorRest)
        self.prepCam()

    def resetCamPosition (self):
        self.cam_pulse = self.cam_max
        self.pwm.setServoPulse(self.cam_channel, self.cam_pulse)
        print("Cam pulse at - " + str(self.cam_pulse))

    def prepCam(self):
        if (self.cam_pulse != self.cam_max):
            self.resetCamPosition()
        self.savePic()
        while (self.cam_pulse > self.cam_min) :
            # Lowers camera to a 100 pulse
            for i in range(self.cam_pulse, self.cam_pulse - 200, -10):  
                self.pwm.setServoPulse(self.cam_channel, i)   
                time.sleep(0.02) 
            print("Cam pulse being set at - " + str(self.cam_pulse))
            self.cam_pulse = self.cam_pulse - 200 
            self.savePic()
        self.resetCamPosition()


    def savePic(self):
        
        ts = str(time.time()).replace(".", "_")
        result, frame = self.cam.read()
        if not result:
            print("failed to grab frame")
        img_name = "images/image_{}.jpg".format(ts)
        print("sleep 1 sec")
        time.sleep(10)
        cv2.imwrite(img_name, frame)
        print("{} written!".format(img_name))
                

    #Up
    def on_triangle_press(self):
        print("Up")
        roboclaw.BackwardM1(0x80,self.motorSpeed)
        roboclaw.BackwardM2(0x80,self.motorSpeed)
        roboclaw.BackwardM1(0x81,self.motorSpeed)
        roboclaw.BackwardM2(0x81,self.motorSpeed)
        self.stopChassis()

    #Down
    def on_x_press(self):
        print("Down")
        roboclaw.ForwardM1(0x80,self.motorSpeed)
        roboclaw.ForwardM2(0x80,self.motorSpeed)
        roboclaw.ForwardM1(0x81,self.motorSpeed)
        roboclaw.ForwardM2(0x81,self.motorSpeed)
        self.stopChassis()


    #Right
    def on_circle_press(self):
        print("Right")
        roboclaw.ForwardM1(0x80,self.motorSpeed)
        roboclaw.BackwardM2(0x80,self.motorSpeed)
        roboclaw.BackwardM1(0x81,self.motorSpeed)
        roboclaw.ForwardM2(0x81,self.motorSpeed)
        self.stopChassis()


    #Left
    def on_square_press(self):
        print("Left")
        roboclaw.BackwardM1(0x80,self.motorSpeed)
        roboclaw.ForwardM2(0x80,self.motorSpeed)
        roboclaw.ForwardM1(0x81,self.motorSpeed)
        roboclaw.BackwardM2(0x81,self.motorSpeed)
        self.stopChassis()


    #RotateRight
    def on_right_arrow_press(self):
        print("RotateRight")
        roboclaw.BackwardM1(0x80,self.motorSpeed)
        roboclaw.ForwardM2(0x80,self.motorSpeed)
        roboclaw.BackwardM1(0x81,self.motorSpeed)
        roboclaw.ForwardM2(0x81,self.motorSpeed)
        self.stopChassis()


    #RotateLeft
    def on_left_arrow_press(self):
        print("RotateLeft")
        roboclaw.ForwardM1(0x80,self.motorSpeed)
        roboclaw.BackwardM2(0x80,self.motorSpeed)
        roboclaw.ForwardM1(0x81,self.motorSpeed)
        roboclaw.BackwardM2(0x81,self.motorSpeed)
        self.stopChassis()



if __name__ == "__main__":
    
    address = 0x80
    roboclaw = Roboclaw("/dev/serial0", 38400)
    result = roboclaw.Open()
    if result == 0:
        print('Unable to open port')
    print('Printing connection result - ' + str(result))
    print('Connection - ' + str(roboclaw._port.is_open))
    
    # pwm = PCA9685(0x40, debug=False)
    # pwm.setPWMFreq(50)
    # pwm.setPWM(0, 0, 4096)
    # pwm.setPWM(2, 0, 4096)
    # pwm.setPWM(4, 0, 4096)

    controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
    controller.listen(timeout=6)


