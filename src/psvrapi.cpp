/*
 Copyright (c) 2016-2017, Seph Li - All rights reserved.
 This code is intended for use with the Cinder C++ library: http://libcinder.org
 This file is part of Cinder-PSVR.
 Cinder-PSVR is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 Cinder-PSVR is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 You should have received a copy of the GNU General Public License
 along with Cinder-PSVR.  If not, see <http://www.gnu.org/licenses/>.
 */


#define NOMINMAX

#include "psvrapi.h"
#include "cinder/app/App.h"
#include "cinder/Utilities.h"
#include "cinder/Log.h"

#include <errno.h>
//#include <mutex>

using namespace ci::app;
using namespace ci;

#define USB_ENDPOINT_IN			0x80
#define USB_ENDPOINT_OUT		0x00
#define compat_err(e) -(errno=libusb_to_errno(e))

//std::mutex mutex;

int frameCount = 0;
float lastTimestamp = 0.f;

static int libusb_to_errno(int result){
	switch (result) {
	case LIBUSB_SUCCESS:
		return 0;
	case LIBUSB_ERROR_IO:
		return EIO;
	case LIBUSB_ERROR_INVALID_PARAM:
		return EINVAL;
	case LIBUSB_ERROR_ACCESS:
		return EACCES;
	case LIBUSB_ERROR_NO_DEVICE:
		return ENXIO;
	case LIBUSB_ERROR_NOT_FOUND:
		return ENOENT;
	case LIBUSB_ERROR_BUSY:
		return EBUSY;
	case LIBUSB_ERROR_TIMEOUT:
		return ETIMEDOUT;
	case LIBUSB_ERROR_OVERFLOW:
		return EOVERFLOW;
	case LIBUSB_ERROR_PIPE:
		return EPIPE;
	case LIBUSB_ERROR_INTERRUPTED:
		return EINTR;
	case LIBUSB_ERROR_NO_MEM:
		return ENOMEM;
	case LIBUSB_ERROR_NOT_SUPPORTED:
		return ENOSYS;
	default:
		return ERANGE;
	}
}

namespace PSVRApi{
    
    libusb_context *_ctx;
    
    //ported from libusb-0.1
	static int usb_bulk_io(libusb_device_handle *dev, int ep, char *bytes, int size, int timeout){
		int actual_length;
		int r;
		//CI_LOG_V("endpoint " + toString(ep) + " size " + toString(size) + " timeout " + toString(timeout));
		r = libusb_bulk_transfer(dev, ep & 0xff, (unsigned char *)bytes, size, &actual_length, timeout);

		/* if we timed out but did transfer some data, report as successful short
		* read. FIXME: is this how libusb-0.1 works? */
		if (r == 0 || (r == LIBUSB_ERROR_TIMEOUT && actual_length > 0))
			return actual_length;
 
		return compat_err(r);
	}

	int usb_bulk_read(libusb_device_handle *dev, int ep, char *bytes, int size, int timeout){
		if (!(ep & USB_ENDPOINT_IN)) {
			/* libusb-0.1 will strangely fix up a read request from endpoint
			* 0x01 to be from endpoint 0x81. do the same thing here, but
			* warn about this silly behaviour. */
			CI_LOG_V("endpoint %x is missing IN direction bit, fixing");
			ep |= USB_ENDPOINT_IN;
		}
		return usb_bulk_io(dev, ep, bytes, size, timeout);
	}

	int usb_bulk_write(libusb_device_handle *dev, int ep, const char *bytes, int size, int timeout){
		if (ep & USB_ENDPOINT_IN) {
			/* libusb-0.1 on BSD strangely fix up a write request to endpoint
			* 0x81 to be to endpoint 0x01. do the same thing here, but
			* warn about this silly behaviour. */
			CI_LOG_V("endpoint %x has excessive IN direction bit, fixing");
			ep &= ~USB_ENDPOINT_IN;
		}
		return usb_bulk_io(dev, ep, (char *)bytes, size, timeout);
	}

	std::string QByteArray(const char * data, int size) {
		std::string res = "";
        for (int i = 0; i < size; i++)
            if(data[i] != '\0') res += data[i];
		return res;
	}
    
    std::vector<libusb_device*>  PSVRContext::listPSVR(){
        std::vector<libusb_device*> res;
        
        int err;
        
        if ((err = libusb_init(&_ctx)) != LIBUSB_SUCCESS ) {
            CI_LOG_E("Libusb Initialization Failed");
            return res;
        }
        
        libusb_device *dev;
        libusb_device **devs;
        int i = 0;
        
        int cnt = (int)libusb_get_device_list(_ctx, &devs);
        if (cnt < 0) {
            CI_LOG_E("Error Device scan");
            return res;
        }
        
        cnt = 0;
        while ((dev = devs[i++]) != NULL){
            struct libusb_device_descriptor desc = {0};
            libusb_get_device_descriptor(dev, &desc);
            if (desc.idVendor == PSVRApi::PSVR_VID && desc.idProduct == PSVRApi::PSVR_PID){
                res.push_back(dev);
                libusb_ref_device(dev);
            }
        }
        
        libusb_free_device_list(devs, 1);
        
        return res;
    }
    
    std::shared_ptr<PSVRContext> PSVRContext::initPSVR(){
        int err;
        
        if ((err = libusb_init(&_ctx)) != LIBUSB_SUCCESS ) {
            CI_LOG_E("Libusb Initialization Failed");
            return NULL;
        }
        
        auto handle = libusb_open_device_with_vid_pid(_ctx, PSVR_VID, PSVR_PID);
        if (handle == NULL) {
            CI_LOG_E("PSVR is not found");
            return NULL;
        }
        
        auto device = libusb_get_device(handle);
        return std::shared_ptr<PSVRContext>(new PSVRContext(device));
    }
    
	PSVRContext::PSVRContext( libusb_device* device ){
        
        int i;
        int err;
        
        struct libusb_device_descriptor usb_descriptor = {0};
        libusb_get_device_descriptor(device, &usb_descriptor);
        err = libusb_open(device, &usbHdl);
        if (err < 0){ CI_LOG_E("Error opening PSVR usb port"); return; }
        
        err = libusb_reset_device(usbHdl);
        if (err < 0){ CI_LOG_E("Cannot reset handler"); return; }
        
#if defined(DEBUG)
        unsigned char string[256];
        if (usb_descriptor.iManufacturer) {
            err = libusb_get_string_descriptor_ascii(usbHdl, usb_descriptor.iManufacturer, string, sizeof(string));
            CI_LOG_V("manufacturer: " << string);
        }
        if (usb_descriptor.iProduct) {
            err = libusb_get_string_descriptor_ascii(usbHdl, usb_descriptor.iProduct, string, sizeof(string));
            CI_LOG_V("iproduct: " << string);
        }
#endif
        
        err = libusb_get_config_descriptor(device, PSVR_CONFIGURATION, &config);
        if (LIBUSB_SUCCESS != err) {
            CI_LOG_E("Cannot get config descriptor");
            return;
        }
        
        for (i = 0; i < config->bNumInterfaces; i++) {
            if ( i == PSVR_USB_INTERFACE::HID_SENSOR || i == PSVR_USB_INTERFACE::HID_CONTROL ) {
                std::string name = (i == PSVR_USB_INTERFACE::HID_SENSOR) ? "PSVR_SENSOR" : "PSVR_CONTROL";
#if defined(CINDER_LINUX) || defined(CINDER_MAC)
                err = libusb_kernel_driver_active(usbHdl, i);
                if (err < 0) { CI_LOG_E(name << "driver status failed"); return; }
                if (err == 1) {
                    CI_LOG_I("Detach kernel driver on " << name);
                    err = libusb_detach_kernel_driver(usbHdl, i);
                    if (err != LIBUSB_SUCCESS) {
                        CI_LOG_E(name << " detach failed");
                        return;
                    }
                }
#endif
                err = libusb_claim_interface(usbHdl, i);
                if (err != LIBUSB_SUCCESS) {
                    CI_LOG_E(name << " interface claim failed: " << err);
                    return;
                }
                CI_LOG_I( name << " claimed" );
            }
        }
        
        stat    = new PSVRStatus;
        connect.emit(true);
        ReadInfo();
        
        BMI055Integrator::Init(BMI055Integrator::AScale::AFS_2G, BMI055Integrator::Gscale::GFS_2000DPS);
        
        running = true;
        
        sensorThread  = std::shared_ptr<std::thread>(new std::thread( std::bind( &PSVRContext::sensorProcess,     this ) ));
        controlThread = std::shared_ptr<std::thread>(new std::thread( std::bind( &PSVRContext::controllerProcess, this ) ));
	}

	PSVRContext::~PSVRContext(){
        running = false;
        if(sensorThread) sensorThread->join();
        if(controlThread) controlThread->join();
        
        ci::sleep(500);
        
        libusb_release_interface(usbHdl, PSVR_USB_INTERFACE::HID_SENSOR);
        CI_LOG_V("PSVR_CONTROL released");
        
        libusb_release_interface(usbHdl, PSVR_USB_INTERFACE::HID_CONTROL);
        CI_LOG_V("PSVR_SENSOR released");
        
        if(config != NULL){
            libusb_free_config_descriptor(config);
            CI_LOG_V("Descriptor is freed");
        }
        
        if (usbHdl != NULL) {
            libusb_close(usbHdl);
            CI_LOG_V("Device is freed");
        }
        
        if (_ctx != NULL) {
            libusb_exit(_ctx);
            CI_LOG_V("LibUsb is freed");
        }
	}
    
    //we cant put control and sensor in one thread - control read seems to be blocking if no update is available
    void PSVRContext::controllerProcess(){
        ci::ThreadSetup threadSetup;
        
        struct PSVRFrame controlFrame;
        int    controlBytesRead = 0;
        
        while (running){
            if (usbHdl != NULL) {
                // read headset status
                controlBytesRead = usb_bulk_read(usbHdl, PSVR_EP_CMD_READ, (char *)&controlFrame, sizeof(PSVRFrame), 0);
                if(controlBytesRead > 0)
                    processControlFrame(controlFrame);
            }
        }
    }

	void PSVRContext::sensorProcess(){
		ci::ThreadSetup threadSetup;

		struct PSVRSensorFrame sensorFrame;
		int    sensorBytesRead = 0;

        lastTimestamp = (float)getElapsedSeconds() + 1.f;
        
		while (running){
			if (usbHdl != NULL) {
                // read sensor data
                sensorBytesRead = usb_bulk_read(usbHdl, PSVR_EP_SENSOR, (char *)&sensorFrame, sizeof(PSVRSensorFrame), 0);
                if(sensorBytesRead > 0){
                    //mutex.lock();
                    PSVRSensorData data;
                    processSensorFrame(sensorFrame, &data);
                    //parse sensor data to quat
                    auto quat  = fixQuat(BMI055Integrator::Parse((void *)&data));
                    auto euler = BMI055Integrator::ToEuler(&quat);
                    //quat = glm::quat_cast(glm::eulerAngleX(euler.y) * glm::eulerAngleY(euler.x) * glm::eulerAngleZ(euler.z));
                    rotationUpdate.emit(quat, euler);
                    /*frameCount ++;
                    if((float)getElapsedSeconds() >= lastTimestamp){
                        CI_LOG_D("update rate:" << frameCount);
                        frameCount = 0;
                        lastTimestamp = (float)getElapsedSeconds() + 1.f;
                    }*/
                    //mutex.unlock();
                }else{
                    app::console() << sensorBytesRead << std::endl;
                }
            }
		}
	}

	void PSVRContext::processSensorFrame(PSVRApi::PSVRSensorFrame rawFrame, PSVRApi::PSVRSensorData *rawData){
		// buttons
		rawData->volumeUpPressed     = rawFrame.buttons & 0x02;
		rawData->volumeDownPressed   = rawFrame.buttons & 0x04;
		rawData->mutePressed         = rawFrame.buttons & 0x08;
		// volume
		rawData->volume              = rawFrame.volume;
		// status
		rawData->isWorn              = rawFrame.status & 0x01;
		rawData->isDisplayActive     = rawFrame.status & 0x02;
		rawData->isMicMuted          = rawFrame.status & 0x08;
		rawData->earphonesConnected  = rawFrame.status & 0x10;
        //frame 1
        rawData->timeStamp_A         = rawFrame.data[0].timestamp;
        rawData->rawGyroYaw_A        = rawFrame.data[0].gyro.yaw;
        rawData->rawGyroPitch_A      = rawFrame.data[0].gyro.pitch;
		rawData->rawGyroRoll_A       = rawFrame.data[0].gyro.roll;
		rawData->rawMotionX_A        = rawFrame.data[0].accel.x >> 4;
		rawData->rawMotionY_A        = rawFrame.data[0].accel.y >> 4;
		rawData->rawMotionZ_A        = rawFrame.data[0].accel.z >> 4;
        //frame 2
        rawData->timeStamp_B         = rawFrame.data[1].timestamp;
        rawData->rawGyroYaw_B        = rawFrame.data[1].gyro.yaw;
        rawData->rawGyroPitch_B      = rawFrame.data[1].gyro.pitch;
		rawData->rawGyroRoll_B       = rawFrame.data[1].gyro.roll;
		rawData->rawMotionX_B        = rawFrame.data[1].accel.x >> 4;
        rawData->rawMotionY_B        = rawFrame.data[1].accel.y >> 4;
		rawData->rawMotionZ_B        = rawFrame.data[1].accel.z >> 4;
        //other stuff
		rawData->calStatus           = rawFrame.calStatus;
		rawData->ready               = rawFrame.ready;
		rawData->voltageValue        = rawFrame.voltageValue;
		rawData->voltageReference    = rawFrame.voltageReference;
		rawData->frameSequence       = rawFrame.frameSequence;
	}

    void PSVRContext::processControlFrame(PSVRFrame frame){
		switch (frame.id){
		case PSVR_INFO_REPORT:
			emitInfoReport(frame);
			break;
		case PSVR_STATUS_REPORT:
			emitStatusReport(frame);
			break;
		case PSVR_UNSOLICITED_REPORT:
			emitUnsolicitedReport(frame);
			break;
		default:
			break;
		}
	}
    
	bool PSVRContext::turnHeadSetOn(){
        PSVRApi::PSVRFrame OnCmd = { 0x17, 0x00, 0xAA, 4, 1};
    	return SendCommand(&OnCmd);
    }
    
    bool PSVRContext::turnHeadSetOff(){
        PSVRApi::PSVRFrame OffCmd = { 0x17, 0x00, 0xAA, 4, 0};
        return SendCommand(&OffCmd);
    }

	bool PSVRContext::enableVRTracking(){
		PSVRFrame sendCmd = { 0x11, 0x00, 0xAA, 8, 0xFF, 0xFF, 0xFF, 0x00 };
        return SendCommand(&sendCmd);
    }
    
    bool PSVRContext::enableVR(){
        PSVRFrame sendCmd = { 0x23, 0x00, 0xAA, 4, 1 };
        return SendCommand(&sendCmd);
    }
    
	bool PSVRContext::enableCinematicMode(){
		PSVRFrame sendCmd = { 0x23, 0x00, 0xAA, 4, 0 };
		return SendCommand(&sendCmd);
	}

	bool PSVRContext::enableCinematicMode(byte distance, byte size, byte brightness, byte micVolume){
		PSVRFrame Cmd = { 0x21, 0x00, 0xAA, 16, 0xC0, distance, size, 0x14, 0, 0, 0, 0, 0, 0, brightness, micVolume, 0, 0, 0, 0 };
		return SendCommand(&Cmd);
	}

	bool PSVRContext::recenterHeadset(){
        BMI055Integrator::Recenter();
		return true;
	}
    
    bool PSVRContext::recalibrateHeadset(){
        BMI055Integrator::Recalibrate();
        return true;
    }

	bool PSVRContext::turnBreakBoxOff(){
		PSVRFrame Cmd = { 0x13, 0x00, 0xAA, 4, 0 };
		return SendCommand(&Cmd);
	}
    
    bool PSVRContext::turnBreakBoxOn(){
        PSVRFrame Cmd = { 0x13, 0x00, 0xAA, 4, 1 };
        return SendCommand(&Cmd);
    }

	bool PSVRContext::ReadInfo(){
		PSVRFrame Cmd = { 0x81, 0x00, 0xAA, 8, 0x80 };
		return SendCommand(&Cmd);
	}

    bool PSVRContext::setLED(PSVR_LEDMASK mask, byte brightness){
        //forget where I read this but the maximum number for led brightess is 100 = 0x64
        brightness = ci::math<byte>::min(brightness, 0x64);
        PSVRFrame Cmd = { 0x15, 0x00, 0xAA, 16, (byte)(mask & 0xFF), (byte)((mask >> 8) & 0xFF),
            brightness, brightness, brightness, brightness, brightness, brightness, brightness, brightness, brightness, 0, 0, 0, 0, 0 };
        return SendCommand(&Cmd);
    }
    
    bool PSVRContext::setLED(PSVR_LEDMASK mask, byte valueA, byte valueB, byte valueC, byte valueD, byte valueE, byte valueF, byte valueG, byte valueH, byte valueI){
        //somehow ci::math<byte>::min gives build error on visual studio
		valueA = ci::math<byte>::min(valueA, 0x64);
        valueB = ci::math<byte>::min(valueB, 0x64);
        valueC = ci::math<byte>::min(valueC, 0x64);
        valueD = ci::math<byte>::min(valueD, 0x64);
        valueE = ci::math<byte>::min(valueE, 0x64);
        valueF = ci::math<byte>::min(valueF, 0x64);
        valueG = ci::math<byte>::min(valueG, 0x64);
        valueH = ci::math<byte>::min(valueH, 0x64);
        valueI = ci::math<byte>::min(valueI, 0x64);
        PSVRFrame Cmd = { 0x15, 0x00, 0xAA, 16, (byte)(mask & 0xFF), (byte)((mask >> 8) & 0xFF),
            valueA, valueB, valueC, valueD, valueE, valueF, valueG, valueH, valueI, 0, 0, 0, 0, 0 };
        return SendCommand(&Cmd);
    }

	void PSVRContext::emitInfoReport(PSVRFrame frame){
		std::string firmware;
		std::string serial;
		firmware  = toString(frame.data[7] + 0x30) + "." + toString(frame.data[8] + 0x30);
		serial    = QByteArray((char *)&frame.data[12], 16);
		infoReport.emit(firmware, serial);
	}

	void PSVRContext::emitStatusReport(PSVRFrame frame){
		stat->isHeadsetOn         = frame.data[0] & (1 << 0);
		stat->isHeadsetWorn       = frame.data[0] & (1 << 1);
		stat->isCinematic         = frame.data[0] & (1 << 2);
		stat->areHeadphonesUsed   = frame.data[0] & (1 << 4);
		stat->isMuted             = frame.data[0] & (1 << 5);
		stat->isCECUsed           = frame.data[0] & (1 << 7);
		stat->volume              = frame.data[1] | (frame.data[2] << 8) | (frame.data[3] << 16) | (frame.data[4] << 24);
		statusReport.emit((void *)stat);
	}
    
    void PSVRContext::emitUnsolicitedReport(PSVRFrame frame){
        unsolicitedReport.emit(frame.data[0], frame.data[1], QByteArray((char *)&frame.data[2], 58));
    }

	bool PSVRContext::SendCommand(PSVRFrame *sendCmd){
        if(!usbHdl || !running) return false;
		int bytesSent;
		if ((bytesSent = usb_bulk_write(usbHdl, PSVR_EP_CMD_WRITE, (char *)sendCmd, 64, 20)) != sizeof(PSVRFrame))
			return false;
		return true;
	}
    
    const glm::quat PSVRContext::fixQuat(glm::quat quat){
        //cant figure out the math to swap x and y axis in quaternion, doing with euler angles
        auto tmp = glm::eulerAngles(quat);
        return glm::normalize(glm::quat_cast(glm::eulerAngleYXZ(-tmp.x, -tmp.y, -tmp.z)));
    }
};
