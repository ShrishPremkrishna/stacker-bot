# stacker-bot

# Raspberry Pi Setup

## Steps to connect to raspberry pi via ssh

### Shrish pi
- Turn on the raspberry pi
- `ping shrishraspberrypi.local` 
- ctrl+c to stop ping
- `ssh pi@shrishraspberrypi.local` 
- Add to list of known hosts
- `shrishraspberry`

### Quest pi
- `ping questraspberrypi.local`
- `ssh pi@questraspberrypi.local` `questraspberry`

## To clone a github repo
- Create Robotics folder
- `mkdir robotics`
- `cd robotics`
- Clone repo
- `git clone https://github.com/ShrishPremkrishna/pi-roboclaw-motor-controller.git`
- `cd pi-roboclaw-motor-controller`
- pip3 install pyPS4Controller

## Useful links 
- https://singleboardbytes.com/647/install-opencv-raspberry-pi-4.htm
- Install OpenCV in raspberry pi - https://qengineering.eu/install-opencv-4.5-on-raspberry-pi-4.html
- Stream raspberry camera - https://makezine.com/projects/beginner-project-a-remote-viewing-camera-with-raspberry-pi/
- http://10.0.0.204:5000/video_feed
- Connect PS4 Controller - https://salamwaddah.com/blog/connecting-ps4-controller-to-raspberry-pi-via-bluetooth
- ps4 controller library - https://pypi.org/project/pyPS4Controller/
- edge impulse - https://www.youtube.com/watch?v=dY3OSiJyne0
- opencv development over ssh - https://richarthurs.com/2019/01/20/raspberrypi-cv-setup/

## Test files to run
- Switch on roboclaw
- Examin the pins
- On basic studio - check baud rate
- On basic studio - check address
- On basic studio - check packet/simple serial mode
- On basic studio - check multi-unit mode
- `cd robotics/pi-roboclaw-motor-controller/roboclaw_packet_serial`
- `python3 packet_serial.py`
- `cd robotics/pi-roboclaw-motor-controller/servo/maker-focus`
- `python3 servo.py`
- `cd robotics/pi-roboclaw-motor-controller/stacker-bot`
- `python3 main.py`
- `cd robotics/pi-roboclaw-motor-controller/cv2`
- `python3 cv2test.py`
- `edge-impulse-linux --disable-microphone`
- https://studio.edgeimpulse.com/studio/80833/acquisition/training?page=1

## To fetch latest code from Github
1. `cd robotics/pi-roboclaw-motor-controller`
2. `git fetch --prune`
3. `git rebase origin/main`

## Reboot and shutdown
- `sudo reboot`
- `sudo shutdown now`
- `sudo poweroff`





