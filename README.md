# Cinder-PSVR
Cinder PlaystationVR block. 

Experimental block to control the PSVR headset, and read (fused) gyro sensor data from it.

It uses libusb 1.0 to talk to the device.

On mac, if the initial process fails when claim usb interfaces, try:
```sudo kextunload -b com.apple.driver.usb.IOUSBHostHIDDevice```

On windows, it may fail when PSVR is connected to usb 3.0 port, try switch to a 2.0 one.
