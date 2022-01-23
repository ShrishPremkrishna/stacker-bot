# stacker-bot

# Raspberry Pi Setup

## Steps to connect to raspberry pi via ssh

### Shrish pi
- Turn on the raspberry pi
- `ping shrishraspberrypi.local`
- `ssh pi@shrishraspberrypi.local` `shrishraspberry`

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
- `cd robotics/pi-roboclaw-motor-controller/roboclaw_packet_serial`
- `python3 packet_serial.py`
- `python3 testMiniUART.py`

## To fetch latest code from Github
- `git fetch --prune`
- `git rebase origin/main`

## To clone stacker-bot






