# Setup new raspberry pi
1. [Boot using sd card](https://projects.raspberrypi.org/en/projects/raspberry-pi-setting-up)
2. Mount SD Card on PI
3. Connect pi to monitor, keyboard and mouse.
4. Switch on Pi
5. Follow the prompts on screen to configure locale and wifi
6. Restart
7. Enable SSH - menu -> preferences -> pi configuration -> interfaces
8. Change PI host name -  menu -> preferences -> pi configuration
9. Restart
10. connect and verify via ssh

## miniUART setup
[Reference] - https://www.circuits.dk/setup-raspberry-pi-3-gpio-uart/
1. `sudo nano /boot/config.txt`
2. This command will open a file. Add the line (at the bottom of the file) `enable_uart=1`
3. To save, Press `ctrl+x` Press `y` Press `Enter`
4. Reboot `sudo reboot`
5. In terminal, enter `ls -l /dev`
6. Verify - In the long listing you will find: 'serial0 -> ttyS0' and 'serial1 -> ttyAMA0'
7. In terminal, enter `sudo systemctl stop serial-getty@ttyS0.service`
8. In terminal, enter`sudo systemctl disable serial-getty@ttyS0.service`
9. In terminal, enter `sudo nano /boot/cmdline.txt`
10. This command will open a file. You will see something like this on the file. "dwc_otg.lpm_enable=0 console=serial0,115200 console=tty1 root=/dev/mmcblk0p2 rootfstype=ext4 elevator=deadline fsck.repair=yes root wait"
10. Remove "console=serial0,115200"
11. To save, Press `ctrl+x` Press `y` Press `Enter`
12. Reboot `sudo reboot`
13. `cd robotics/pi-roboclaw-motor-controller/roboclaw_packet_serial`
14. `sudo chmod 666 /dev/serial0`
15. `python3 testMiniUART.py`
16. `python packet_serial.py`

## Open raspberry configuration
- `sudo rasp-config`

## Dependencies for stacker-bot
- `pip3 install pyPS4Controller`
- `sudo pigpiod`
- `sudo chmod 666 /dev/serial0`



