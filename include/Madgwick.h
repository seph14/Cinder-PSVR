#pragma once

#include "cinder/Cinder.h"
#include "cinder/CinderMath.h"

namespace PSVRApi{
	/// <summary>
	/// MadgwickAHRS class. Implementation of Madgwick's IMU and AHRS algorithms.
	/// </summary>
	/// <remarks>
	/// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
	/// </remarks>
	class MadgwickAHRS{
	public:
		glm::quat Quaternion;
		MadgwickAHRS(glm::quat InitialPose);
		void Update(float gx, float gy, float gz, float ax, float ay, float az, float Beta, float SamplePeriod);
	};
}
