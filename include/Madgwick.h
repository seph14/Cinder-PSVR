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
#include "cinder/CinderMath.h"

namespace PSVRApi{
	/// MadgwickAHRS class. Implementation of Madgwick's IMU and AHRS algorithms.
	/// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
	class MadgwickAHRS{
	public:
		glm::quat Quaternion;
		MadgwickAHRS(glm::quat InitialPose);
		void Update(float gx, float gy, float gz, float ax, float ay, float az, float Beta, float SamplePeriod);
	};
}
