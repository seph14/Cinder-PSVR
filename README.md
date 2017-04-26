# Cinder-PSVR
Cinder PlaystationVR block. 

This block controls the PSVR headset, and read (fused) gyro sensor data from it.  
This block only uses gyro sensors without the PSCamera tracking, so it doesn't support controllers and position tracking. 

##### USB drivers:
It uses [libusb 1.0](http://libusb.info/) to talk to the device.  
Pre-built libraries of libusb 1.0.21 have been included.   
On Windows it was built on Visual Studio 2015 (toolset v140), if you want to use this with older versions, you need to built libusb yourself.

##### Usage:
The block was integrated into my fork of [Cinder-VR](https://github.com/seph14/Cinder-VR),
so if you want the rendering part being taken care of, use that block instead.  
If you want to play with PSVR more, add this block to your project.  
The default cinderblock.xml refers to the prebuilt static library to makes builds faster,
but you could also use the cinderblock_code.xml which addes the source code.

##### Example:  

Initialize PSVR  
```c++
void CinderApp::setup(){    
	mPSVRRef = psvr::PSVR::create(true);    
}
```  

Turn on Headset
```c++
mPSVRRef->psvrContext->turnHeadSetOn();
```

Enable VR mode
```c++
mPSVRRef->psvrContext->enableVR();
```

Turn on all LEDs
```c++
mPSVRRef->psvrContext->setLED(PSVRApi::PSVR_LEDMASK::All, 100);
```  

##### Unstable Sensor Data:
The internal fusion data uses the first 4000 samples to stablize (2 seconds),  
so keep your headset stable when starting up.  

##### Troubleshoots:

On mac, if the initial process fails when claim usb interfaces, try:  
```sudo kextunload -b com.apple.driver.usb.IOUSBHostHIDDevice```  

On windows, it may fail when PSVR is connected to usb 3.0 port, try switch to a 2.0 one.  

If you are using an AMD card, and after enabling VR mode and the screen falls to black,
try setting your graphic card pixel format to RGB 4:4:4.
