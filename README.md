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

## To run python files
- `cd roboclaw_packet_serial`
- `python3 testMiniUART.py`

## To fetch latest code from Github
- `git fetch --prune`
- `git rebase origin/main`
13. `cd robotics/pi-roboclaw-motor-controller/roboclaw_packet_serial`
14. `sudo chmod 666 /dev/serial0`
15. `python3 testMiniUART.py`
16. `python packet_serial.py`
13. `cd robotics/pi-roboclaw-motor-controller/ps4PlusRoboclaw`
14. `sudo pigpiod`
14. `python3 ps4_motor.py`


## Reboot and shutdown
- `sudo reboot`
- `sudo shutdown now`
- `sudo poweroff`

## To clone stacker-bot






