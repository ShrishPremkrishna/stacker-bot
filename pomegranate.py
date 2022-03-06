from pyPS4Controller.controller import Controller
from roboclaw import Roboclaw
from PCA9685 import PCA9685
from time import sleep


class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)

    #Up
    def on_triangle_press(self):
        print("Up")

    #Down
    def on_x_press(self):
        print("Down")

    #Right
    def on_circle_press(self):
        print("Right")

    #Left
    def on_square_press(self):
        print("Left")

    #RotateRight
    def on_right_arrow_press(self):
        print("RotateRight")

    #RotateLeft
    def on_left_arrow_press(self):
        print("RotateLeft")


