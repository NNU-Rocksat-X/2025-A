import pyrealsense2 as rs

15:19:52 [Warn] /home/rocksat1/librealsense-2.50.0/common/notifications.cpp:511 - RealSense UDEV-Rules are missing!
UDEV-Rules permissions configuration 
 for RealSense devices.`
Missing/outdated UDEV-Rules will cause 'Permissions Denied' errors
unless the application is running under 'sudo' (not recommended)
In case of Debians use: 
sudo apt-get upgrade/install librealsense2-udev-rules
To manually install UDEV-Rules in terminal run:
$ sudo cp ~/.99-realsense-libusb.rules /etc/udev/rules.d/99-realsense-libusb.rules && sudo udevadm control --reload-rules && udevadm trigger

15:27:22 [Error] /home/rocksat1/librealsense-2.50.0/src/libusb/handle-libusb.h:51 - failed to open usb interface: 0, error: RS2_USB_STATUS_ACCESS