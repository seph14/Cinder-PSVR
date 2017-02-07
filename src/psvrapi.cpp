#include "psvrapi.h"
#include "cinder/app/App.h"
#include "cinder/Utilities.h"
#include <errno.h>

using namespace ci::app;
using namespace ci;

#define USB_ENDPOINT_IN			0x80
#define USB_ENDPOINT_OUT		0x00
#define compat_err(e) -(errno=libusb_to_errno(e))

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

	libusb_context*	usb_context = NULL;

	void debug(std::string content) {
		app::console() << content << std::endl;
	}

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
			res += data[i];
		return res;
	}

	/// -----------------------------------------------------------------------
	/// \brief PSVRContext::PSVRContext
	///
	PSVRContext::PSVRContext(){
		psvrControl = new PSVRControl();
		psvrSensor = new PSVRSensor();
	}

	/// -----------------------------------------------------------------------
	/// \brief PSVRContext::~PSVRContext
	///
	PSVRContext::~PSVRContext(){
		//psvrControl->exit();
		psvrControl->Close(HID_CONTROL);
		delete psvrControl;

		//psvrSensor->exit();
		psvrSensor->Close(HID_SENSOR);
		delete psvrSensor;
	}

	/// -----------------------------------------------------------------------
	/// \brief Sensor::Sensor
	/// \param sensor
	///
	PSVRSensor::PSVRSensor(){
	}

	/// -----------------------------------------------------------------------
	/// \brief PSVRSensor::PSVRSensor
	///
	PSVRSensor::~PSVRSensor(){
		running = false;
		thread->join();
	}

	void PSVRSensor::run() {
		running = true;
		thread = std::shared_ptr<std::thread>(new std::thread( std::bind( &PSVRSensor::process, this ) ));
	}

	/// -----------------------------------------------------------------------
	/// \brief PSVRSensor::process
	///        Thread implementation for sensor
	///
	void PSVRSensor::process(){
		ci::ThreadSetup threadSetup;

		struct PSVRSensorFrame   frame;

		int     bytesRead = 0;
		bool    connected = false;

	
		Sleep(2000);

		// read reply
		while (running){
			// connect to usb, emit connect signal
			if (!connected){
				if (Open(7, HID_SENSOR)){
					//emit connect(true);
					connect.emit(true);
					connected = true;
				}
			}

			// read from device
			if (usbHdl != NULL) {
				bytesRead += usb_bulk_read(usbHdl, PSVR_EP_SENSOR, (char *)&frame, sizeof(PSVRSensorFrame), 0);

			}

			// we got some error from usb, eg. device is disconnected from usb
			if (bytesRead < 0){
				// close connection
				if (connected){
					Close(HID_CONTROL);
					connected = false;
					//emit connect(false);
					connect.emit(false);
				}
				Sleep(20);
				continue;
			}

			if (connected) {
				PSVRSensorData data;
				ProcessFrame(frame, data);
			}
		}
	}

	void PSVRSensor::ProcessFrame(PSVRApi::PSVRSensorFrame rawFrame, PSVRApi::PSVRSensorData rawData){
		// buttons
		rawData.volumeUpPressed = rawFrame.buttons & 0x02;
		rawData.volumeDownPressed = rawFrame.buttons & 0x04;
		rawData.mutePressed = rawFrame.buttons & 0x08;

		// volume
		rawData.volume = rawFrame.volume;

		// status
		rawData.isWorn = rawFrame.status & 0x01;
		rawData.isDisplayActive = rawFrame.status & 0x02;
		rawData.isMicMuted = rawFrame.status & 0x08;
		rawData.earphonesConnected = rawFrame.status & 0x10;

		rawData.timeStamp_A = rawFrame.timeStampA1 | (rawFrame.timeStampA2 << 8) | (rawFrame.timeStampA3 << 16) | (rawFrame.timeStampA3 << 24);

		rawData.rawGyroYaw_A = rawFrame.rawGyroYaw_AL | (rawFrame.rawGyroYaw_AL << 8);
		rawData.rawGyroPitch_A = rawFrame.rawGyroPitch_AL | (rawFrame.rawGyroPitch_AL << 8);
		rawData.rawGyroRoll_A = rawFrame.rawGyroRoll_AL | (rawFrame.rawGyroRoll_AL << 8);

		rawData.rawMotionX_A = (rawFrame.rawMotionX_AL | (rawFrame.rawMotionX_AH << 8)) >> 4;
		rawData.rawMotionY_A = (rawFrame.rawMotionY_AL | (rawFrame.rawMotionY_AH << 8)) >> 4;
		rawData.rawMotionZ_A = (rawFrame.rawMotionZ_AL | (rawFrame.rawMotionZ_AH << 8)) >> 4;

		rawData.timeStamp_B = rawFrame.timeStamp_B1 | (rawFrame.timeStamp_B2 << 8) | (rawFrame.timeStamp_B3 << 16) | (rawFrame.timeStamp_B4 << 24);

		rawData.rawGyroYaw_B = rawFrame.rawGyroYaw_BL | (rawFrame.rawGyroYaw_BL << 8);
		rawData.rawGyroPitch_B = rawFrame.rawGyroPitch_BL | (rawFrame.rawGyroPitch_BL << 8);
		rawData.rawGyroRoll_B = rawFrame.rawGyroRoll_BL | (rawFrame.rawGyroRoll_BL << 8);

		rawData.rawMotionX_B = (rawFrame.rawMotionX_BL | (rawFrame.rawMotionX_BH << 8)) >> 4;
		rawData.rawMotionY_B = (rawFrame.rawMotionY_BL | (rawFrame.rawMotionY_BH << 8)) >> 4;
		rawData.rawMotionZ_B = (rawFrame.rawMotionZ_BL | (rawFrame.rawMotionZ_BH << 8)) >> 4;

		rawData.calStatus = rawFrame.calStatus;
		rawData.ready = rawFrame.ready;
		rawData.voltageValue = rawFrame.voltageValue;
		rawData.voltageReference = rawFrame.voltageReference;
		rawData.frameSequence = rawFrame.frameSequence;
	}

	/// -----------------------------------------------------------------------
	/// \brief PSVRControl::PSVRControl
	///
	PSVRControl::PSVRControl(){}

	/// -----------------------------------------------------------------------
	/// \brief PSVRControl::~PSVRControl
	///
	PSVRControl::~PSVRControl(){
		running = false;
		thread->join();
	}

	void PSVRControl::run() {
		running = true;
		thread = std::shared_ptr<std::thread>(new std::thread(std::bind(&PSVRControl::process, this)));
	}

	/// -----------------------------------------------------------------------
	/// \brief PSVRControl::run
	///        Thread implementation for control
	///
	void PSVRControl::emitUnsolicitedReport(PSVRFrame frame){
		//emit 
		unsolicitedReport.emit(frame.data[0], frame.data[1], QByteArray((char *)&frame.data[2], 58));
	}

	/// -----------------------------------------------------------------------
	/// \brief PSVRControl::process
	///    
	void PSVRControl::process(){
		ci::ThreadSetup threadSetup;
		PSVRFrame    frame;

		int     bytesRead = 0;
		bool    connected = false;

		while (running){
			// connect to usb, emit connect signal
			if (!connected){
				if (Open(8, HID_CONTROL)){
					//
					ReadInfo();
					HeadSetPower(true);
					EnableCinematic();
					connect.emit(true);
					connected = true;
				}
			}

			if (usbHdl != NULL) {
				bytesRead += usb_bulk_read(usbHdl, 132, (char *)&frame, sizeof(PSVRFrame), 0);
			}
			// we got some error from usb, eg. device is disconnected from usb
			if (bytesRead < 0){
				// close connection
				if (connected){
					Close(HID_CONTROL);
					connected = false;
					connect.emit(false);
				}
				Sleep(20);
				continue;
			}

			if (connected) {
				processFrame(frame);
				bytesRead = 0;
			}
		}
	}

	/// -----------------------------------------------------------------------
	/// \brief PSVRControl::processFrame
	/// \param frame
	///
	void PSVRControl::processFrame(PSVRFrame frame){
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
	/// -----------------------------------------------------------------------
	/// \brief PSVRControl::HeadSetPower
	/// \param OnOff
	/// \return
	///
	bool PSVRControl::HeadSetPower(bool OnOff){
		PSVRFrame sendCmd = { 0x17, 0x00, 0xAA, 4, (byte)OnOff };
		return SendCommand(&sendCmd);
	}

	/// -----------------------------------------------------------------------
	/// \brief PSVRControl::EnableVR
	/// \param WithTracking
	/// \return
	///
	bool PSVRControl::EnableVR(bool WithTracking){
		// generate packet for shutdown
		PSVRFrame sendCmd;

		// clear memory
		memset(&sendCmd, 0x00, sizeof(sendCmd));

		if (WithTracking){
			// without tracking
			sendCmd.id = 0x11;
			sendCmd.start = 0xAA;
			sendCmd.length = 8;

			// payload
			sendCmd.data[1] = 0xFF;
			sendCmd.data[2] = 0xFF;
			sendCmd.data[3] = 0xFF;

		}else{
			// without tracking
			sendCmd.id = 0x23;
			sendCmd.start = 0xAA;
			sendCmd.length = sizeof(int);

			// payload
			sendCmd.data[0] = 1; // VRWithoutTracking = 1 / Cinematic = 0
		}

		// send shutdown command and receive reply
		return SendCommand(&sendCmd);
	}

	/// -----------------------------------------------------------------------
	/// \brief PSVRControl::EnableCinematic
	/// \return
	///
	bool PSVRControl::EnableCinematic(){
		// generate packet for shutdown
		PSVRFrame sendCmd = { 0x23, 0x00, 0xAA, 4, 0 }; // VR = 1 / Cinematic = 0
		return SendCommand(&sendCmd);
	}

	/// -----------------------------------------------------------------------
	/// \brief PSVRControl::SetCinematic
	/// \param distance
	/// \param size
	/// \param brightness
	/// \param micVolume
	/// \return
	///
	bool PSVRControl::SetCinematic(byte distance, byte size, byte brightness, byte micVolume){
		PSVRFrame Cmd = { 0x21, 0x00, 0xAA, 16, 0xC0, distance, size, 0x14, 0, 0, 0, 0, 0, 0, brightness, micVolume, 0, 0, 0, 0 };
		return SendCommand(&Cmd);
	}

	/// -----------------------------------------------------------------------
	/// \brief PSVRControl::Recenter
	/// \return
	///
	bool PSVRControl::Recenter(){
		return true;
	}

	/// -----------------------------------------------------------------------
	/// \brief PSVRControl::Shutdown
	/// \return
	///
	bool PSVRControl::Shutdown(){
		// shutdown command
		PSVRFrame Cmd = { 0x13, 0x00, 0xAA, 4, 1 };
		return SendCommand(&Cmd);
	}

	/// -----------------------------------------------------------------------
	/// \brief PSVRControl::ReadInfo
	/// \return
	///
	bool PSVRControl::ReadInfo(){
		// read info command
		PSVRFrame Cmd = { 0x81, 0x00, 0xAA, 8, 0x80 };
		return SendCommand(&Cmd);
	}

	bool PSVRControl::EnterVRMode() {
		PSVRFrame Cmd = { 0x23, 0x00, 0xAA, 4, 1 };
		return SendCommand(&Cmd);
	}

	bool PSVRControl::ExitVRMode() {
		PSVRFrame Cmd = { 0x23, 0x00, 0xAA, 4, 0 };
		return SendCommand(&Cmd);
	}

	bool PSVRControl::On(byte id) {
		PSVRFrame Cmd = { id, 0x00, 0xAA, 4, 1 };
		return SendCommand(&Cmd);
	}

	bool PSVRControl::Off(byte id) {
		PSVRFrame Cmd = { id, 0x00, 0xAA, 4, 0 };
		return SendCommand(&Cmd);
	}

	bool PSVRControl::SetHMDLed(PSVR_LEDMASK Mask, byte Value) {
		PSVRFrame Cmd = { 0x15, 0x00, 0xAA, 16, (byte)(Mask & 0xFF), (byte)((Mask >> 8) & 0xFF), Value, Value, Value, Value, Value, Value, Value, Value, Value, 0, 0, 0, 0, 0 };
		return SendCommand(&Cmd);
	}

	bool PSVRControl::SetHDMLeds(PSVR_LEDMASK Mask, byte ValueA, byte ValueB, byte ValueC, byte ValueD, byte ValueE, byte ValueF, byte ValueG, byte ValueH, byte ValueI) {
		PSVRFrame Cmd = { 0x15, 0x00, 0xAA, 16,  (byte)(Mask & 0xFF), (byte)((Mask >> 8) & 0xFF), ValueA, ValueB, ValueC, ValueD, ValueE, ValueF, ValueG, ValueH, ValueI, 0, 0, 0, 0, 0 };
		return SendCommand(&Cmd);
	}

	/// -----------------------------------------------------------------------
	/// \brief PSVRControl::emitInfoReport
	/// \param frame
	///
	void PSVRControl::emitInfoReport(PSVRFrame frame){
		std::string firmware;
		std::string serial;
		firmware = (frame.data[7] + 0x30) + "." + (frame.data[8] + 0x30);
		serial = QByteArray((char *)&frame.data[12], 16);
		infoReport.emit(firmware, serial);
	}

	/// -----------------------------------------------------------------------
	/// \brief PSVRControl::emitStatusReport
	/// \param frame
	///
	void PSVRControl::emitStatusReport(PSVRFrame frame){
		PSVRStatus  *status = new PSVRStatus;
		status->isHeadsetOn = frame.data[0] & (1 << 0);
		status->isHeadsetWorn = frame.data[0] & (1 << 1);
		status->isCinematic = frame.data[0] & (1 << 2);
		status->areHeadphonesUsed = frame.data[0] & (1 << 4);
		status->isMuted = frame.data[0] & (1 << 5);
		status->isCECUsed = frame.data[0] & (1 << 7);
		status->volume = frame.data[1] | (frame.data[2] << 8) | (frame.data[3] << 16) | (frame.data[4] << 24);
		statusReport.emit((void *)status);
	}

	/// -----------------------------------------------------------------------
	/// \brief PSVRCommon::Open
	/// \param productNumber
	/// \return
	///
	bool PSVRCommon::Open(int productNumber, PSVR_USB_INTERFACE usbInterface){
		if (usb_context == NULL) {
			libusb_init(&usb_context);
			libusb_set_debug(usb_context, 1);
		}

		libusb_device *dev;
		libusb_device **devs;
		
		int i = 0;
		int cnt;

		cnt = (int)libusb_get_device_list(usb_context, &devs);

		if (cnt < 0) {
			//("Error Device scan\n");
		}

		cnt = 0;
		while ((dev = devs[i++]) != NULL) {
			struct libusb_device_descriptor desc = { 0 };
			libusb_get_device_descriptor(dev, &desc);
			
			app::console() << "bcddevice: " << desc.bcdDevice << std::endl;
			app::console() << "bcdUSB: " << desc.bcdUSB << std::endl;
			app::console() << "bDescriptorType" << desc.bDescriptorType << std::endl;
			app::console() << "bDeviceClass: " << desc.bDeviceClass << std::endl;
			app::console() << "bDeviceProtocol" << desc.bDeviceProtocol << std::endl;
			app::console() << "bDeviceSubClass" << desc.bDeviceSubClass << std::endl;
			app::console() << "bLength" << desc.bLength << std::endl;
			app::console() << "bMaxPacketSize0" << desc.bMaxPacketSize0 << std::endl;
			app::console() << "bNumConfigurations" << desc.bNumConfigurations << std::endl;
			app::console() << "iManufacturer" << desc.iManufacturer << std::endl;
			app::console() << "iProduct" << desc.iProduct << std::endl;
			app::console() << "iSerialNumber" << desc.iSerialNumber << std::endl;
			app::console() << "idVendor" << desc.idVendor << std::endl;
			app::console() << "idProduct" << desc.idProduct << std::endl << std::endl;

			if (desc.idVendor == PSVR_VID && desc.idProduct == PSVR_PID) {
				app::console() << i << std::endl;
				if (desc.iProduct == productNumber) {
					// open PSVR Control Interface
					int err = libusb_open(dev, &usbHdl);
					if (err == 0) {
						int r = libusb_claim_interface(usbHdl, usbInterface);
						if (r == 0) {
							last_claimed_interface = usbInterface;
							break;
						}
					}
				//}
			}
		}

		if (usbHdl != NULL){
			// reset endpoint
			libusb_clear_halt(usbHdl, PSVR_EP_CMD_READ & 0xff);
			return TRUE;
		}else return FALSE;
	}

	/// -----------------------------------------------------------------------
	/// \brief PSVRCommon::Close
	/// \param usbInterface
	///
	void PSVRCommon::Close(PSVR_USB_INTERFACE usbInterface){
		int r = libusb_release_interface(usbHdl, usbInterface);
		if (r == 0) last_claimed_interface = -1;
		libusb_close(usbHdl);
		free(usbHdl);

		//usb_release_interface(usbHdl, usbInterface);
		//usb_close(usbHdl);
		usbHdl = NULL;
	}

	/// -----------------------------------------------------------------------
	/// \brief PSVRCommon::SendCommand
	/// \param Command
	/// \return
	///
	bool PSVRCommon::SendCommand(PSVRFrame *sendCmd){

		int bytesSent;

		// send command
		if ((bytesSent = usb_bulk_write(usbHdl, PSVR_EP_CMD_WRITE, (char *)sendCmd, 64, 20)) != sizeof(PSVRFrame))
			return false;

		return true;
	}
}
