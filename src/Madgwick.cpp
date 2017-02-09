#include "Madgwick.h"

using namespace glm;
using namespace ci;

namespace PSVRApi{

    MadgwickAHRS::MadgwickAHRS(glm::quat InitialPose) {
        Quaternion = InitialPose;
    }

    void MadgwickAHRS::Update(float gx, float gy, float gz, float ax, float ay, float az, float Beta, float SamplePeriod) {
        float q1 = Quaternion.w, q2 = Quaternion.x, q3 = Quaternion.y, q4 = Quaternion.z;   // short name local variable for readability
        float norm;
        float s1, s2, s3, s4;
        float qDot1, qDot2, qDot3, qDot4;

        // Auxiliary variables to avoid repeated arithmetic
        float _2q1 = 2.f * q1;
        float _2q2 = 2.f * q2;
        float _2q3 = 2.f * q3;
        float _2q4 = 2.f * q4;
        float _4q1 = 4.f * q1;
        float _4q2 = 4.f * q2;
        float _4q3 = 4.f * q3;
        float _8q2 = 8.f * q2;
        float _8q3 = 8.f * q3;
        float q1q1 = q1 * q1;
        float q2q2 = q2 * q2;
        float q3q3 = q3 * q3;
        float q4q4 = q4 * q4;

        // Normalise accelerometer measurement
        norm = glm::sqrt(ax * ax + ay * ay + az * az);
        if (norm == 0.f) return; // handle NaN
        norm = 1.f / norm;        // use reciprocal for division
        ax *= norm;
        ay *= norm;
        az *= norm;

        // Gradient decent algorithm corrective step
        s1 = _4q1 * q3q3 + _2q3 * ax + _4q1 * q2q2 - _2q2 * ay;
        s2 = _4q2 * q4q4 - _2q4 * ax + 4.f * q1q1 * q2 - _2q1 * ay - _4q2 + _8q2 * q2q2 + _8q2 * q3q3 + _4q2 * az;
        s3 = 4.f * q1q1 * q3 + _2q1 * ax + _4q3 * q4q4 - _2q4 * ay - _4q3 + _8q3 * q2q2 + _8q3 * q3q3 + _4q3 * az;
        s4 = 4.f * q2q2 * q4 - _2q2 * ax + 4.f * q3q3 * q4 - _2q3 * ay;
        norm = 1.f / glm::sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
        s1 *= norm;
        s2 *= norm;
        s3 *= norm;
        s4 *= norm;

        // Compute rate of change of quaternion
        qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - Beta * s1;
        qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - Beta * s2;
        qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - Beta * s3;
        qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - Beta * s4;

        // Integrate to yield quaternion
        q1 += qDot1 * SamplePeriod;
        q2 += qDot2 * SamplePeriod;
        q3 += qDot3 * SamplePeriod;
        q4 += qDot4 * SamplePeriod;
        norm = 1.f / glm::sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
        Quaternion.w = q1 * norm;
        Quaternion.x = q2 * norm;
        Quaternion.y = q3 * norm;
        Quaternion.z = q4 * norm;
    }
};
