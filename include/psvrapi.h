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

#pragma once

#include "cinder/Cinder.h"
#include "cinder/Thread.h"
#include "cinder/app/App.h"
#include "cinder/Signals.h"

#include "BMI055Integrator.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

#include <memory>

//on mac you need this:
//sudo kextunload -b com.apple.driver.usb.IOUSBHostHIDDevice

// Get rid of annoying zero length structure warnings from libusb.h in MSVC

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4200)
#endif

#include "libusb.h"

//typedef unsigned char byte;
typedef uint8_t byte;

///
/// \brief The PSVRApi namespace
///
namespace PSVRApi{
    
	///
	/// \brief PSVR_VID
	///
	static int const           PSVR_VID(0x054C);
	///
	/// \brief PSVR_PID
	///
	static int const           PSVR_PID(0x09AF);
	///
	/// \brief PSVR_EP_COMMAND
	///
	static int const           PSVR_EP_CMD_WRITE(0x04);
	///
	/// \brief PSVR_EP_SENSOR
	///
	static int const           PSVR_EP_CMD_READ(PSVR_EP_CMD_WRITE | 0x80);
	///
	/// \brief PSVR_EP_SENSOR
	///
	static int const           PSVR_EP_SENSOR(0x03 | 0x80);
	///
	/// \brief PSVR_INFO_REPORT
	///
	static int const           PSVR_INFO_REPORT(0x80);
	///
	/// \brief PSVR_STATUS_REPORT
	///
	static int const           PSVR_STATUS_REPORT(0xF0);
	///
	/// \brief PSVR_UNSOL_REPORT
	///
	static int const           PSVR_UNSOLICITED_REPORT(0xA0);

    static int const           PSVR_CONFIGURATION(0);
    
	/// ----------------------------------
	/// \brief The PSVR_USB_INTERFACE enum
	///
	typedef enum {
		AUDIO_3D      = 0,
		AUDIO_CONTROL = 1,
		AUDIO_MIC     = 2,
		AUDIO_CHAT    = 3,
		HID_SENSOR    = 4, //sensor for polling all data
        HID_CONTROL   = 5, //controller
		VS_H264       = 6,
		VS_BULK_IN    = 7,
		HID_CONTROL2  = 8
	} PSVR_USB_INTERFACE;

    typedef enum : byte{
		VolumeUp   = 2,
		VolumeDown = 4,
		Mute       = 8
	} PSVR_HEADSET_BUTTONS;

	typedef enum : uint16_t {
		None = 0,
		LedA = (1 << 0),
		LedB = (1 << 1),
		LedC = (1 << 2),
		LedD = (1 << 3),
		LedE = (1 << 4),
		LedF = (1 << 5),
		LedG = (1 << 6),
		LedH = (1 << 7),
		LedI = (1 << 8),
		All = LedA | LedB | LedC | LedD | LedE | LedF | LedG | LedH | LedI
	} PSVR_LEDMASK;
	
    /// -----------------------------
	/// \brief The PSVRFrame struct
	///
    
#pragma pack(1)
	struct PSVRFrame {
		byte                   id;
		byte                   status;
		byte                   start;
		byte                   length;
		byte                   data[60];
	};

	struct PSVRSensorFrame {
        PSVR_HEADSET_BUTTONS  buttons;
        byte                  u_b01;
        byte                  volume;
        byte                  u_b02[5];
		byte                  status;
        byte                  u_b03[7];
        //16
        
        struct {
            uint32_t timestamp;
            struct {
                int16_t yaw;
                int16_t pitch;
                int16_t roll;
            } gyro;
            struct {
                int16_t x;
                int16_t y;
                int16_t z;
            } accel;
        } data[2];
        //48
        
        byte                  calStatus;
        byte                  ready;
        byte                  u_b04[3];
        byte                  voltageValue;
		byte                  voltageReference;
		int16_t               irSensor;

        byte                  u_b05[5];
		byte                  frameSequence;
        byte                  u_b06;
	};
    
#pragma pack()
	///
	/// \brief The PSVRSensorData struct
	///
	struct PSVRSensorData {
		bool                      volumeUpPressed;
		bool                      volumeDownPressed;
		bool                      mutePressed;

		byte                      volume;

		bool                      isWorn;
		bool                      isDisplayActive;
		bool                      isMicMuted;
		bool                      earphonesConnected;

		uint32_t                  timeStamp_A;

		int16_t                       rawGyroYaw_A;
		int16_t                       rawGyroPitch_A;
		int16_t                       rawGyroRoll_A;

		int16_t                       rawMotionX_A;
		int16_t                       rawMotionY_A;
		int16_t                       rawMotionZ_A;

		uint32_t                  timeStamp_B;

		int16_t                       rawGyroYaw_B;
		int16_t                       rawGyroPitch_B;
		int16_t                       rawGyroRoll_B;

		int16_t                       rawMotionX_B;
		int16_t                       rawMotionY_B;
		int16_t                       rawMotionZ_B;

		byte                      calStatus;
		byte                      ready;

		byte                      voltageValue;
		byte                      voltageReference;

		short                     irSensor;

		byte                      frameSequence;
	};

	///
	/// \brief The PSVRStatus struct
	///
	struct                     PSVRStatus {
	public:
		bool                   isHeadsetOn;
		bool                   isHeadsetWorn;
		bool                   isCinematic;
		bool                   areHeadphonesUsed;
		bool                   isMuted;
		bool                   isCECUsed;
		int                    volume;
	};

	class PSVRContext {

	public:
        libusb_context *usb = NULL;
        
        PSVRContext( libusb_device* device );
		~PSVRContext();
        
        static std::vector<libusb_device*>   listPSVR();
        static std::shared_ptr<PSVRContext>  initPSVR();
        static std::shared_ptr<PSVRContext>  create( libusb_device* device ) { return std::shared_ptr<PSVRContext>(new PSVRContext(device)); }
        
        bool turnHeadSetOn();
        bool turnHeadSetOff();
        
        bool setLED(PSVR_LEDMASK mask, byte brightess);
        bool setLED(PSVR_LEDMASK mask, byte valueA, byte valueB, byte valueC, byte valueD, byte valueE, byte valueF, byte valueG, byte valueH, byte valueI);

        bool enableVRTracking();
        bool enableVR();
        
        bool enableCinematicMode();
        bool enableCinematicMode(byte distance, byte size, byte brightness, byte micVolume);
        bool recenterHeadset();
        bool recalibrateHeadset();
        
        /*interface for ci-vr*/
        void getRecommendedRenderTargetSize(uint32_t *width, uint32_t *height) { *width = 960; *height = 1080; }
        
        bool turnBreakBoxOff();
        bool turnBreakBoxOn();
        
        bool ReadInfo();
        
        bool isHeadsetOn()	     { return stat->isHeadsetOn; }
        bool isHeadsetWorn()     { return stat->isHeadsetWorn; }
        bool isCinematic()       { return stat->isCinematic; }
        bool areHeadphonesUsed() { return stat->areHeadphonesUsed; }
        bool isMuted()           { return stat->isMuted; }
        bool isCECUsed()         { return stat->isCECUsed; }
        int  getVolume()         { return stat->volume; }
        
        std::string            getSerialNumber();
        std::string            getVersion();
        
        ci::signals::Signal<void(bool)>                                            connect;
        ci::signals::Signal<void(std::string, std::string)>                        infoReport;
        ci::signals::Signal<void(void*)>                                           statusReport;
        ci::signals::Signal<void(byte reportId, byte result, std::string message)> unsolicitedReport;
        ci::signals::Signal<void(glm::quat quat, glm::vec3 euler)>                 rotationUpdate;
        
    protected:
		std::string            serialNumber;
		unsigned char          minorVersion;
		unsigned char          majorVersion;
        
        PSVRApi::PSVRStatus *stat;
        
        libusb_device_handle *usbHdl            = NULL;
        struct libusb_config_descriptor *config = NULL;
        uint32_t claimed_interfaces;
        
        bool SendCommand(PSVRFrame *sendCmd);
        
        std::shared_ptr<std::thread> sensorThread, controlThread;
        
        bool running = false;
        void sensorProcess();
        
        void  controllerProcess();
        const glm::quat fixQuat   (glm::quat quat);
        
        void processSensorFrame   (PSVRSensorFrame rawFrame, PSVRSensorData *rawData);
        void processControlFrame  (PSVRFrame frame);
        
        void emitInfoReport       (PSVRFrame frame);
        void emitStatusReport     (PSVRFrame frame);
        void emitUnsolicitedReport(PSVRFrame frame);
    };
    
    typedef std::shared_ptr<PSVRContext> PSVRContextRef;
}
