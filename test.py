import cv2
import calendar;
import time;

cam = cv2.VideoCapture(0)

gmt = time.gmtime()
ts = calendar.timegm(gmt)


result, frame = cam.read()

if not result:
    print("failed to grab frame")


    
 
img_name = "image_{}.jpg".format(ts)
cv2.imwrite(img_name, frame)
print("{} written!".format(img_name))

cam.release()
