#include "psvr.h"

using namespace std;
using namespace ci;
using namespace PSVRApi;

namespace psvr {
    PSVR::PSVR( libusb_device* device, bool enableLogging ){
        psvrContext = PSVRContext::create(device);
        if(psvrContext != NULL && enableLogging){
            psvrContext->connect.connect	 (std::bind(&PSVR::onConnect,           this, std::placeholders::_1  ));
            psvrContext->infoReport.connect	 (std::bind(&PSVR::setInfo,             this, std::placeholders::_1, std::placeholders::_2));
            psvrContext->statusReport.connect(std::bind(&PSVR::setStatus,           this, std::placeholders::_1) );
            psvrContext->rotationUpdate.connect(std::bind(&PSVR::rotationUpdate,    this, std::placeholders::_1, std::placeholders::_2));
            psvrContext->unsolicitedReport.connect(std::bind(&PSVR::setUnsolicited, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        }
    }
    
    PSVR::PSVR( bool enableLogging ){
        psvrContext = PSVRContext::initPSVR();
        if(psvrContext != NULL && enableLogging){
            psvrContext->connect.connect	 (std::bind(&PSVR::onConnect,           this, std::placeholders::_1  ));
            psvrContext->infoReport.connect	 (std::bind(&PSVR::setInfo,             this, std::placeholders::_1, std::placeholders::_1));
            psvrContext->statusReport.connect(std::bind(&PSVR::setStatus,           this, std::placeholders::_1  ));
            psvrContext->rotationUpdate.connect(std::bind(&PSVR::rotationUpdate,    this, std::placeholders::_1, std::placeholders::_2));
            psvrContext->unsolicitedReport.connect(std::bind(&PSVR::setUnsolicited, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        }
    }

    void PSVR::setInfo(std::string firmware, std::string serial){
		app::console() << "Firmware: " + firmware << ", Serial: " << serial << endl;
	}

	void PSVR::setStatus(void *status) {
        app::console() << "PSVR status updated" << endl;
        
        PSVRStatus *stat = (PSVRStatus*)status;
        
        app::console() << "Is Headset On: "         << stat->isHeadsetOn        << endl;
        app::console() << "Is Headset Worn: "       << stat->isHeadsetWorn      << endl;
        app::console() << "Is Cinematic Mode: "     << stat->isCinematic        << endl;
        app::console() << "Are Headphone Used: "    << stat->areHeadphonesUsed  << endl;
        app::console() << "Is Muted: "              << stat->isMuted            << endl;
        app::console() << "Is CEC Used: "           << stat->isCECUsed          << endl;
        app::console() << "Volume: "                << stat->volume             << endl;
	}

	void PSVR::onConnect(bool isConnected){
		if (!isConnected){
			app::console() << "PSVR is disconnected" << endl;
        }else{
            app::console() << "PSVR is connected" << endl;
        }
	}

	void PSVR::setUnsolicited(byte reportId, byte result, std::string message) {
		app::console() << "Unsolicited: report:" << (int)reportId << ", result:" << (int)result << ", message:" << message << endl;
	}
    
    void PSVR::rotationUpdate(ci::quat quat, ci::vec3 dir){
        mQuat = quat;
        mDirection = dir;
    }
}
