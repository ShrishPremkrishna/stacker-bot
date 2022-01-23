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

## To clone stacker-bot






