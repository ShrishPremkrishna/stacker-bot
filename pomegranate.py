from pyPS4Controller.controller import Controller
from roboclaw import Roboclaw
from PCA9685 import PCA9685
from time import sleep

speed = 20
rest = 0

class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)

    #Up
    def on_triangle_press(self):
        print("Up")
        roboclaw.BackwardM1(0x80,speed)
        roboclaw.BackwardM2(0x80,speed)
        roboclaw.BackwardM1(0x81,speed)
        roboclaw.BackwardM2(0x81,speed)
        sleep(0.5)
        roboclaw.ForwardM1(0x80,rest)
        roboclaw.ForwardM2(0x80,rest)
        roboclaw.ForwardM1(0x81,rest)
        roboclaw.ForwardM2(0x81,rest)

    #Down
    def on_x_press(self):
        print("Down")
        roboclaw.ForwardM1(0x80,speed)
        roboclaw.ForwardM2(0x80,speed)
        roboclaw.ForwardM1(0x81,speed)
        roboclaw.ForwardM2(0x81,speed)
        sleep(0.5)
        roboclaw.ForwardM1(0x80,rest)
        roboclaw.ForwardM2(0x80,rest)
        roboclaw.ForwardM1(0x81,rest)
        roboclaw.ForwardM2(0x81,rest)

    #Right
    def on_circle_press(self):
        print("Right")
        roboclaw.ForwardM1(0x80,speed)
        roboclaw.BackwardM2(0x80,speed)
        roboclaw.BackwardM1(0x81,speed)
        roboclaw.ForwardM2(0x81,speed)
        sleep(0.5)
        roboclaw.ForwardM1(0x80,rest)
        roboclaw.ForwardM2(0x80,rest)
        roboclaw.ForwardM1(0x81,rest)
        roboclaw.ForwardM2(0x81,rest)

    #Left
    def on_square_press(self):
        print("Left")
        roboclaw.BackwardM1(0x80,speed)
        roboclaw.ForwardM2(0x80,speed)
        roboclaw.ForwardM1(0x81,speed)
        roboclaw.BackwardM2(0x81,speed)
        sleep(0.5)
        roboclaw.ForwardM1(0x80,rest)
        roboclaw.ForwardM2(0x80,rest)
        roboclaw.ForwardM1(0x81,rest)
        roboclaw.ForwardM2(0x81,rest)

    #RotateRight
    def on_right_arrow_press(self):
        print("RotateRight")
        roboclaw.BackwardM1(0x80,speed)
        roboclaw.ForwardM2(0x80,speed)
        roboclaw.BackwardM1(0x81,speed)
        roboclaw.ForwardM2(0x81,speed)
        sleep(0.5)
        roboclaw.ForwardM1(0x80,rest)
        roboclaw.ForwardM2(0x80,rest)
        roboclaw.ForwardM1(0x81,rest)
        roboclaw.ForwardM2(0x81,rest)

    #RotateLeft
    def on_left_arrow_press(self):
        print("RotateLeft")
        roboclaw.ForwardM1(0x80,speed)
        roboclaw.BackwardM2(0x80,speed)
        roboclaw.ForwardM1(0x81,speed)
        roboclaw.BackwardM2(0x81,speed)
        sleep(0.5)
        roboclaw.ForwardM1(0x80,rest)
        roboclaw.ForwardM2(0x80,rest)
        roboclaw.ForwardM1(0x81,rest)
        roboclaw.ForwardM2(0x81,rest)


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


