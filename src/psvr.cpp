#include "psvr.h"

using namespace std;
using namespace ci;
using namespace PSVRApi;

namespace psvr {
	PSVR::PSVR() {
		psvrRef = shared_ptr<PSVRContext>(new PSVRContext());
		PSVRControl *psvrControl = psvrRef->psvrControl;
		PSVRSensor *psvrSensor = psvrRef->psvrSensor;

		PSVRApi::PSVRStatus *stat = new PSVRStatus();
		stat->isHeadsetOn		= false;
		stat->isHeadsetWorn		= false;
		stat->isCinematic		= true;
		stat->areHeadphonesUsed = false;
		stat->isMuted			= false;
		stat->isCECUsed			= false;
		stat->volume			= 0;

		psvrControl->connect.connect		  (std::bind(&PSVR::onConnect,      this, std::placeholders::_1));
		psvrControl->infoReport.connect		  (std::bind(&PSVR::setInfo,        this, std::placeholders::_1, std::placeholders::_1));
		psvrControl->statusReport.connect     (std::bind(&PSVR::setStatus,      this, std::placeholders::_1));
		//psvrControl->unsolicitedReport.connect(std::bind(&PSVR::setUnsolicited, this, std::placeholders::_1, std::placeholders::_1, std::placeholders::_1));

		psvrControl->run();
		psvrSensor->run();
	}

	void PSVR::turnHeadSetOn() {
		psvrRef->psvrControl->HeadSetPower(true);
	}

	void PSVR::turnHeadSetOff() {
		psvrRef->psvrControl->HeadSetPower(false);
	}

	void PSVR::enableVRTracking() {
		psvrRef->psvrControl->EnableVR(true);
	}

	void PSVR::enableVR() {
		psvrRef->psvrControl->EnableVR(false);
	}

	void PSVR::enableCinematicMode() {
		psvrRef->psvrControl->EnableCinematic();
	}

	void PSVR::recenterHeadset() {
		psvrRef->psvrControl->Recenter();
	}

	void PSVR::shutdown() {
		psvrRef->psvrControl->Shutdown();
	}

	void PSVR::setInfo(std::string firmware, std::string serial){
		app::console() << "Firmware: " + firmware << ", Serial: " << serial << endl;
		return;
	}

	void PSVR::setStatus(void *status) {
		stat = (PSVRApi::PSVRStatus *)status;
		return;
	}

	void PSVR::onConnect(bool isConnected){
		if (!isConnected){
			app::console() << "Device is disconnected" << endl;
		}
	}

	void PSVR::setUnsolicited(byte reportId, byte result, std::string message) {
		app::console() << "Unsolicited: " << message << endl;
	}
}