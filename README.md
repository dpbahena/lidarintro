# lidarintro
lidar A1M8 from scratch

1. Plug it USB  then run:
    dmesg | grep tty
2. You should see something like this:  
    [ 1234.567890] usb 1-1: ch341-uart converter now attached to ttyUSB0
    or
    [ 1234.567890] cdc_acm 1-1:1.0: ttyACM0: USB ACM device
3. Once you know the name, confirm it:
    ls -l /dev/ttyUSB*
    or
    ls -l /dev/ttyACM*
4. Get device info (optional):
    udevadm info -q all -n /dev/ttyUSB0
5. If permissions needed:
    sudo chmod 666 /dev/ttyUSB0
6. Compile and Run
   ./build/lidar

