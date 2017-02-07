#pragma once

#include "psvrapi.h"
#include "cinder/Cinder.h"

namespace psvr {
	
	class PSVR {
	public:
		PSVR();

		static std::shared_ptr<PSVR> create() { return std::shared_ptr<PSVR>(new PSVR()); }

		void turnHeadSetOn();
		void turnHeadSetOff();
		void enableVRTracking();
		void enableVR();
		void enableCinematicMode();
		void recenterHeadset();
		void shutdown();

		bool isHeadsetOn()	     { return stat->isHeadsetOn; }
		bool isHeadsetWorn()     { return stat->isHeadsetWorn; }
		bool isCinematic()       { return stat->isCinematic; }
		bool areHeadphonesUsed() { return stat->areHeadphonesUsed; }
		bool isMuted()           { return stat->isMuted; }
		bool isCECUsed()         { return stat->isCECUsed; }
		int  getVolume()         { return stat->volume; }

		std::shared_ptr<PSVRApi::PSVRContext> psvrRef;

	protected:
		PSVRApi::PSVRStatus *stat;
		void setInfo(std::string firmware, std::string serial);
		void setStatus(void *status);
		void onConnect(bool isConnected);
		void setUnsolicited(byte reportId, byte result, std::string message);
	};

	typedef std::shared_ptr<PSVR> PSVRRef;
}