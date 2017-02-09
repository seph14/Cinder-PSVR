#include "BMI055Integrator.h"
#include "cinder/CinderMath.h"

using namespace ci;

namespace PSVRApi{

    int             BMI055Integrator::samplesLeft   = 2000;
    bool            BMI055Integrator::LightLed      = true;
    bool            BMI055Integrator::recalibrate   = false;
    bool            BMI055Integrator::recenter      = false;
    bool            BMI055Integrator::calibrating   = true;
    float           BMI055Integrator::aRes          = 0.f;
    float           BMI055Integrator::gRes          = 0.f;
    glm::vec3       BMI055Integrator::gravityVector = glm::vec3(0.f);
    glm::vec3       BMI055Integrator::accelOffset   = glm::vec3(0.f);
    glm::vec3       BMI055Integrator::gyroOffset    = glm::vec3(0.f);
    uint64_t        BMI055Integrator::prevTimestamp = 0;
    MadgwickAHRS    BMI055Integrator::fusion        = MadgwickAHRS(glm::quat());
    glm::quat       BMI055Integrator::ZeroPose      = glm::quat();

    void BMI055Integrator::Init(AScale AccelerometerScale, Gscale GyroscopeScale) {
        aRes = GetAres(AccelerometerScale);
        gRes = GetGres(GyroscopeScale);
    }

    void BMI055Integrator::Recalibrate() {
        calibrating = true;
        recalibrate = true;
    }

    void BMI055Integrator::Recenter() {
        recenter = true;
    }

    glm::quat BMI055Integrator::Parse(void *data) {
        PSVRSensorData *stat = (PSVRSensorData*)data;

        if (recalibrate){
            samplesLeft = 2000;
            accelOffset = glm::vec3(0.f);
            gyroOffset  = glm::vec3(0.f);
            recalibrate = false;
        }
        
        auto linearAcceleration1  = ci::vec3(stat->rawMotionX_A * aRes, stat->rawMotionY_A * aRes,   stat->rawMotionZ_A * aRes );
        auto angularAcceleration1 = ci::vec3(stat->rawGyroYaw_A * gRes, stat->rawGyroPitch_A * gRes, stat->rawGyroRoll_A * gRes);
        Integrate(linearAcceleration1, angularAcceleration1, stat->timeStamp_A);
        
        auto linearAcceleration2  = ci::vec3(stat->rawMotionX_B * aRes, stat->rawMotionY_B * aRes,   stat->rawMotionZ_B * aRes );
        auto angularAcceleration2 = ci::vec3(stat->rawGyroYaw_B * gRes, stat->rawGyroPitch_B * gRes, stat->rawGyroRoll_B * gRes);
        return Integrate(linearAcceleration2, angularAcceleration2, stat->timeStamp_B);
}

float BMI055Integrator::GetGres(Gscale Scale) {
	switch (Scale){
		// Possible gyro scales (and their register bit settings) are:
		// 125 DPS (100), 250 DPS (011), 500 DPS (010), 1000 DPS (001), and 2000 DPS (000). 
		case Gscale::GFS_125DPS:
			return (float)(0.00381f * (M_PI / 180)); //return 124.87 / (32768.0 * 4); // per data sheet, not exactly 125!?
		case Gscale::GFS_250DPS:
			return (float)(0.007622f * (M_PI / 180)); //1.0 / 262.4; //return 249.75 / 32768.0;
		case Gscale::GFS_500DPS:
			return (float)(0.01524f * (M_PI / 180));//1.0 / 262.4; //return 499.5 / 32768.0;
		case Gscale::GFS_1000DPS:
			return (float)(0.03048f * (M_PI / 180)); //1.0 / 262.4; //return 999.0 / 32768.0;
		case Gscale::GFS_2000DPS:
			return (float)(0.06097f * (M_PI / 180));//1.0 / 262.4; //return 1998.0 / 32768.0;
	}
	return 0;
}

float BMI055Integrator::GetAres(AScale Scale) {
	switch (Scale){
		case AScale::AFS_2G:
			return 2.0f / 2048.0f;
		case AScale::AFS_4G:
			return 4.0f / 2048.0f;
		case AScale::AFS_8G:
			return 8.0f / 2048.0f;
		case AScale::AFS_16G:
			return 16.0f / 2048.0f;
	}
	return 0;
}

glm::quat BMI055Integrator::Integrate(glm::vec3 linearAcceleration, glm::vec3 angularAcceleration, uint64_t Timestamp) {
	if (samplesLeft > 0){
		samplesLeft --;
		accelOffset += linearAcceleration;
		gyroOffset  += angularAcceleration;
		return glm::quat();
	}else if (samplesLeft == 0){
		samplesLeft		--;
		accelOffset		/= 2000;
		gyroOffset		/= 2000;
		gravityVector	= glm::normalize(accelOffset);
		accelOffset		-= gravityVector;
		prevTimestamp	= Timestamp;
		fusion			= MadgwickAHRS(glm::quat());
		return glm::quat();
	}else if (samplesLeft > -1500){
		samplesLeft--;
		linearAcceleration   = glm::normalize(linearAcceleration - accelOffset);
		angularAcceleration -= gyroOffset;
		float interval = 0;
		if (prevTimestamp > Timestamp)
			interval = (Timestamp + (0xFFFFFF - prevTimestamp)) / 1000000.0f;
		else
			interval = (Timestamp - prevTimestamp) / 1000000.0f;

		fusion.Update( angularAcceleration.x, angularAcceleration.y, angularAcceleration.z, 
			           linearAcceleration.x,  linearAcceleration.y,  linearAcceleration.z, 
			           1.5f, interval );
		prevTimestamp = Timestamp;
		return glm::quat();
	}else if (samplesLeft > -2000){
		samplesLeft--;
		linearAcceleration   = glm::normalize(linearAcceleration - accelOffset);
		angularAcceleration -= gyroOffset;
		float interval       = 0;

		if (prevTimestamp > Timestamp)
			interval = (Timestamp + (0xFFFFFF - prevTimestamp)) / 1000000.0f;
		else
			interval = (Timestamp - prevTimestamp) / 1000000.0f;

		fusion.Update(angularAcceleration.x, angularAcceleration.y, angularAcceleration.z, 
			          linearAcceleration.x,  linearAcceleration.y,  linearAcceleration.z, 0.05f, interval);
		prevTimestamp = Timestamp;
		return glm::quat();

	}else if (samplesLeft == -2000){
		samplesLeft --;
		linearAcceleration   = glm::normalize(linearAcceleration - accelOffset);
		angularAcceleration -= gyroOffset;
		float interval = 0;
		if (prevTimestamp > Timestamp)
			interval = (Timestamp + (0xFFFFFF - prevTimestamp)) / 1000000.0f;
		else
			interval = (Timestamp - prevTimestamp) / 1000000.0f;

		fusion.Update( angularAcceleration.x, angularAcceleration.y, angularAcceleration.z, 
			           linearAcceleration.x,  linearAcceleration.y,  linearAcceleration.z, 
			           0.035f, interval);
		prevTimestamp = Timestamp;
		ZeroPose = glm::quat() * glm::inverse(fusion.Quaternion);

		calibrating = false;
		return glm::quat();
	}else{
		linearAcceleration = glm::normalize(linearAcceleration - accelOffset);
		angularAcceleration -= gyroOffset;

		float interval = 0;

		if (prevTimestamp > Timestamp)
			interval = (Timestamp + (0xFFFFFF - prevTimestamp)) / 1000000.0f;
		else
			interval = (Timestamp - prevTimestamp) / 1000000.0f;

		fusion.Update(angularAcceleration.x, angularAcceleration.y, angularAcceleration.z, 
			          linearAcceleration.x,  linearAcceleration.y,  linearAcceleration.z, 0.035f, interval);
		prevTimestamp = Timestamp;

		if (recenter){
			ZeroPose = glm::quat() * glm::inverse(fusion.Quaternion);
			recenter = false;
		}

		return glm::inverse(ZeroPose * fusion.Quaternion);
	}
}

glm::vec3 BMI055Integrator::ToEuler(glm::quat *Q) {
	glm::vec3 pitchYawRoll;

	double sqw = Q->w * Q->w;
	//double sqx = Q->x * Q->x;
	double sqy = Q->y * Q->y;
	double sqz = Q->z * Q->z;
	
	pitchYawRoll.x = -(float)math<double>::atan2(2.f * Q->x * Q->w + 2.f * Q->y * Q->z, 1 - 2.f * (sqz + sqw)); // Yaw 
	pitchYawRoll.y = -(float)math<double>::asin(2.f * (Q->x * Q->z - Q->w * Q->y));                             // Pitch 
	pitchYawRoll.z = (float)math<double>::atan2(2.f * Q->x * Q->y + 2.f * Q->z * Q->w, 1 - 2.f * (sqy + sqz));  // Roll 

	return pitchYawRoll;
}
}
