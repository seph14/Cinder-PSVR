#pragma once

#include "psvrapi.h"
#include "cinder/Cinder.h"

namespace psvr {
	
	class PSVR {
	public:
        PSVR( libusb_device* device, bool enableLogging = false );
        PSVR( bool enableLogging = false );

		static std::shared_ptr<PSVR> create(bool enableLogging) { return std::shared_ptr<PSVR>(new PSVR(enableLogging)); }
        static std::shared_ptr<PSVR> create(libusb_device* device, bool enableLogging) { return std::shared_ptr<PSVR>(new PSVR(device, enableLogging)); }
        
        PSVRApi::PSVRContextRef psvrContext;

        ci::quat getQuaterion(){ return mQuat; }
        ci::vec3 getViewDirection(){ return mDirection; }
        
	protected:
        ci::quat mQuat;
        ci::vec3 mDirection;
        
		void setInfo(std::string firmware, std::string serial);
		void setStatus(void *status);
		void onConnect(bool isConnected);
		void setUnsolicited(byte reportId, byte result, std::string message);
        void rotationUpdate(ci::quat quat, ci::vec3 dir);
	};

	typedef std::shared_ptr<PSVR> PSVRRef;
}
