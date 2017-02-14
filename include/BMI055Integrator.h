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
