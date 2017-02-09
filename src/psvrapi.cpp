#include "psvrapi.h"
#include "cinder/app/App.h"
#include "cinder/Utilities.h"
#include <errno.h>
#include <mutex>

using namespace ci::app;
using namespace ci;

#define USB_ENDPOINT_IN			0x80
#define USB_ENDPOINT_OUT		0x00
#define compat_err(e) -(errno=libusb_to_errno(e))

std::mutex mutex;

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
    
	void debug(std::string content) {
#if 0
		app::console() << content << std::endl;
#endif
	}
    
    //ported from libusb-0.1
	static int usb_bulk_io(libusb_device_handle *dev, int ep, char *bytes, int size, int timeout){
		int actual_length;
		int r;
		debug("endpoint " + toString(ep) + " size " + toString(size) + " timeout " + toString(timeout));
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
			debug("endpoint %x is missing IN direction bit, fixing");
			ep |= USB_ENDPOINT_IN;
		}
		return usb_bulk_io(dev, ep, bytes, size, timeout);
	}

	int usb_bulk_write(libusb_device_handle *dev, int ep, const char *bytes, int size, int timeout){
		if (ep & USB_ENDPOINT_IN) {
			/* libusb-0.1 on BSD strangely fix up a write request to endpoint
			* 0x81 to be to endpoint 0x01. do the same thing here, but
			* warn about this silly behaviour. */
			debug("endpoint %x has excessive IN direction bit, fixing");
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
            app::console() << "Libusb Initialization Failed" << std::endl;
            return res;
        }
        
        libusb_device *dev;
        libusb_device **devs;
        int i = 0;
        
        int cnt = (int)libusb_get_device_list(_ctx, &devs);
        if (cnt < 0) {
            app::console() << "Error Device scan" << std::endl;
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
            app::console() << "Libusb Initialization Failed" << std::endl;
            return NULL;
        }
        
        auto handle = libusb_open_device_with_vid_pid(_ctx, PSVR_VID, PSVR_PID);
        if (handle == NULL) {
            app::console() << "PSVR is not found" << std::endl;
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
        if (err < 0){ app::console() << "Error opening PSVR usb port" << std::endl; return; }
        
        err = libusb_reset_device(usbHdl);
        if (err < 0){ app::console() << "Cannot reset handler" << std::endl; return; }
        
#if defined(DEBUG)
        unsigned char string[256];
        if (usb_descriptor.iManufacturer) {
            err = libusb_get_string_descriptor_ascii(usbHdl, usb_descriptor.iManufacturer, string, sizeof(string));
            console() << "manufacturer: " << string << std::endl;
        }
        if (usb_descriptor.iProduct) {
            err = libusb_get_string_descriptor_ascii(usbHdl, usb_descriptor.iProduct, string, sizeof(string));
            console() << "iproduct: " << string << std::endl;
        }
#endif
        
        err = libusb_get_config_descriptor(device, PSVR_CONFIGURATION, &config);
        if (LIBUSB_SUCCESS != err) {
            app::console() << "Cannot get config descriptor" << std::endl;
            return;
        }
        
        for (i = 0; i < config->bNumInterfaces; i++) {
            if ( i == PSVR_USB_INTERFACE::HID_SENSOR || i == PSVR_USB_INTERFACE::HID_CONTROL ) {
                std::string name = (i == PSVR_USB_INTERFACE::HID_SENSOR) ? "PSVR_SENSOR" : "PSVR_CONTROL";
                err = libusb_kernel_driver_active(usbHdl, i);
                if (err < 0) { app::console() << name << "driver status failed" << std::endl; return; }
                if (err == 1) {
                    app::console() << "Detach kernel driver on " << name << std::endl;
                    err = libusb_detach_kernel_driver(usbHdl, i);
                    if (err != LIBUSB_SUCCESS) {
                        app::console() << name << " detach failed" << std::endl;
                        return;
                    }
                }
                err = libusb_claim_interface(usbHdl, i);
                if (err != LIBUSB_SUCCESS) {
                    app::console() << name << " interface claim failed" << std::endl;
                    return;
                }
                app::console() << name << " claimed" << std::endl;
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
        sensorThread->join();
        controlThread->join();
        
        ci::sleep(500);
        
        libusb_release_interface(usbHdl, PSVR_USB_INTERFACE::HID_SENSOR);
#if defined(DEBUG)
        app::console() << "PSVR_CONTROL released" << std::endl;
#endif
        
        libusb_release_interface(usbHdl, PSVR_USB_INTERFACE::HID_CONTROL);
#if defined(DEBUG)
        app::console() << "PSVR_SENSOR released" << std::endl;
#endif
        
        if(config != NULL){
            libusb_free_config_descriptor(config);
#if defined(DEBUG)
            app::console() << "Descriptor is freed" << std::endl;
#endif
        }
        
        if (usbHdl != NULL) {
            libusb_close(usbHdl);
#if defined(DEBUG)
            app::console() << "Device is freed" << std::endl;
#endif
        }
        
        if (_ctx != NULL) {
            libusb_exit(_ctx);
#if defined(DEBUG)
            app::console() << "LibUsb is freed" << std::endl;
#endif
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

		while (running){
			if (usbHdl != NULL) {
                // read sensor data
                sensorBytesRead = usb_bulk_read(usbHdl, PSVR_EP_SENSOR, (char *)&sensorFrame, sizeof(PSVRSensorFrame), 0);
                if(sensorBytesRead > 0){
                    mutex.lock();
                    PSVRSensorData data;
                    processSensorFrame(sensorFrame, &data);
                    //parse sensor data to quat
                    auto quat  = BMI055Integrator::Parse((void *)&data);
                    auto euler = BMI055Integrator::ToEuler(&quat);
                    rotationUpdate.emit(quat, euler);
                    mutex.unlock();
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
		rawData->rawMotionX_A        = rawFrame.data[0].accel.x;
		rawData->rawMotionY_A        = rawFrame.data[0].accel.y;
		rawData->rawMotionZ_A        = rawFrame.data[0].accel.z;
        //frame 2
        rawData->timeStamp_B         = rawFrame.data[1].timestamp;
        rawData->rawGyroYaw_B        = rawFrame.data[1].gyro.yaw;
        rawData->rawGyroPitch_B      = rawFrame.data[1].gyro.pitch;
		rawData->rawGyroRoll_B       = rawFrame.data[1].gyro.roll;
		rawData->rawMotionX_B        = rawFrame.data[1].accel.x;
        rawData->rawMotionY_B        = rawFrame.data[1].accel.x;
		rawData->rawMotionZ_B        = rawFrame.data[1].accel.x;
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
        debug("not implemented");
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
};
