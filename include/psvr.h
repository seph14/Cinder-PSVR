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

        glm::quat getQuaterion(){ return mQuat; }
        ci::vec3 getViewDirection(){ return mDirection; }
        
	protected:
        glm::quat mQuat;
        ci::vec3 mDirection;
        
		void setInfo(std::string firmware, std::string serial);
		void setStatus(void *status);
		void onConnect(bool isConnected);
		void setUnsolicited(byte reportId, byte result, std::string message);
        void rotationUpdate(glm::quat quat, ci::vec3 dir);
	};

	typedef std::shared_ptr<PSVR> PSVRRef;
}
