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

## Test files to run
- Switch on roboclaw
- `cd roboclaw_packet_serial`
- `python3 testMiniUART.py`

## To fetch latest code from Github
1. `cd robotics/pi-roboclaw-motor-controller`
2. `git fetch --prune`
3. `git rebase origin/main`
4. `cd ps4PlusRoboclaw`
5. `sudo chmod 666 /dev/serial0`
9. `sudo pigpiod`
10. `python3 ps4_motor.py`


## Reboot and shutdown
- `sudo reboot`
- `sudo shutdown now`
- `sudo poweroff`

## To clone stacker-bot






