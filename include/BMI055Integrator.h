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
#include "Madgwick.h"
#include "psvrapi.h"

namespace PSVRApi{
	class BMI055Integrator {
	public:
		static bool LightLed;
		static bool calibrating;

		enum AScale {
			AFS_2G = 0x03,
			AFS_4G = 0x05,
			AFS_8G = 0x08,
			AFS_16G = 0x0C
		};

		enum Gscale{
			GFS_2000DPS = 0,
			GFS_1000DPS,
			GFS_500DPS,
			GFS_250DPS,
			GFS_125DPS
		};

		static void  Init(AScale AccelerometerScale, Gscale GyroscopeScale);
		static void  Recalibrate();
		static void  Recenter();
        
        static glm::quat Parse(void *data);
        static ci::vec3  ToEuler( glm::quat *Q );
        
		static float GetGres(Gscale Scale);
		static float GetAres(AScale Scale);

	protected:
		static float     aRes;
		static float     gRes;
		static glm::vec3 gravityVector;
		static glm::vec3 accelOffset;
		static glm::vec3 gyroOffset;

		static int       samplesLeft;
		static uint32_t  prevTimestamp;

		static MadgwickAHRS fusion;
		static glm::quat ZeroPose;

		static bool recalibrate;
		static bool recenter;

		static glm::quat Integrate(glm::vec3 linearAcceleration, glm::vec3 angularAcceleration, uint32_t Timestamp);
	};
}
