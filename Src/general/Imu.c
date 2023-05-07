#include "IMU.h"
#include "Math.h"
#include <math.h>
#include <stdint.h>
#include "Imu.h"
#include "Math.h"


_imu_st imu_data =  {1,0,0,0,
					{0,0,0},
					{0,0,0},
					{0,0,0},
					{0,0,0},
					{0,0,0},
					{0,0,0},
					 0,0,0};
uint32_t imu_data_cnt = 0;

float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	/** quaternion of sensor frame relative to auxiliary frame */
float dq0 = 0.0f, dq1 = 0.0f, dq2 = 0.0f, dq3 = 0.0f;	/** quaternion of sensor frame relative to auxiliary frame */
float gyro_bias[3] = {0.0f, 0.0f, 0.0f}; /** bias estimation */
float q0q0, q0q1, q0q2, q0q3;
float q1q1, q1q2, q1q3;
float q2q2, q2q3;
float q3q3;
int bFilterInit = 0;


float invSqrt(float number)
{
    volatile long i;
    volatile float x, y;
    volatile const float f = 1.5F;

    x = number * 0.5F;
    y = number;
    i = * (( long * ) &y);
    i = 0x5f375a86 - ( i >> 1 );
    y = * (( float * ) &i);
    y = y * ( f - ( x * y * y ) );
    return y;
}

void NonlinearSO3AHRSinit(float ax, float ay, float az, float mx, float my, float mz)
{
    float initialRoll, initialPitch;
    float cosRoll, sinRoll, cosPitch, sinPitch;
    float magX, magY;
    float initialHdg, cosHeading, sinHeading;

    initialRoll = fast_atan2(-ay, -az);
    initialPitch = fast_atan2(ax, -az);

    cosRoll = my_cos(initialRoll);
    sinRoll = my_sin(initialRoll);
    cosPitch = my_cos(initialPitch);
    sinPitch = my_sin(initialPitch);

    magX = mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll * sinPitch;

    magY = my * cosRoll - mz * sinRoll;

    initialHdg = fast_atan2(-magY, magX);

    cosRoll = my_cos(initialRoll * 0.5f);
    sinRoll = my_sin(initialRoll * 0.5f);

    cosPitch = my_cos(initialPitch * 0.5f);
    sinPitch = my_sin(initialPitch * 0.5f);

    cosHeading = my_cos(initialHdg * 0.5f);
    sinHeading = my_sin(initialHdg * 0.5f);

    q0 = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
    q1 = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
    q2 = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
    q3 = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;

    // auxillary variables to reduce number of repeated operations, for 1st pass
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;
}

void NonlinearSO3AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float twoKp, float twoKi, float dt)
{
    float recipNorm;
    float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;

    // Make filter converge to initial solution faster
    // This function assumes you are in static position.
    // WARNING : in case air reboot, this can cause problem. But this is very unlikely happen.
    if(bFilterInit == 0) {
        NonlinearSO3AHRSinit(ax,ay,az,mx,my,mz);
        bFilterInit = 1;
    }

    //! If magnetometer measurement is available, use it.
    if(!((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))) {
        float hx, hy, hz, bx, bz;
        float halfwx, halfwy, halfwz;

        // Normalise magnetometer measurement
        // Will sqrt work better? PX4 system is powerful enough?
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        hz = 2.0f * mx * (q1q3 - q0q2) + 2.0f * my * (q2q3 + q0q1) + 2.0f * mz * (0.5f - q1q1 - q2q2);
        bx = my_sqrt(hx * hx + hy * hy);
        bz = hz;

        // Estimated direction of magnetic field
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

        // Error is sum of cross product between estimated direction and measured direction of field vectors
        halfex += (my * halfwz - mz * halfwy);
        halfey += (mz * halfwx - mx * halfwz);
        halfez += (mx * halfwy - my * halfwx);
    }

    //增加一个条件：  加速度的模量与G相差不远时。 0.75*G < normAcc < 1.25*G
    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        float halfvx, halfvy, halfvz;

        // Normalise accelerometer measurement
        //归一化，得到单位加速度
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);

        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity and magnetic field
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;

        // Error is sum of cross product between estimated direction and measured direction of field vectors
        halfex += ay * halfvz - az * halfvy;
        halfey += az * halfvx - ax * halfvz;
        halfez += ax * halfvy - ay * halfvx;
    }

    // Apply feedback only when valid data has been gathered from the accelerometer or magnetometer
    if(halfex != 0.0f && halfey != 0.0f && halfez != 0.0f) {
        // Compute and apply integral feedback if enabled
        if(twoKi > 0.0f) {
            gyro_bias[0] += twoKi * halfex * dt;	// integral error scaled by Ki
            gyro_bias[1] += twoKi * halfey * dt;
            gyro_bias[2] += twoKi * halfez * dt;

            // apply integral feedback
            gx += gyro_bias[0];
            gy += gyro_bias[1];
            gz += gyro_bias[2];
        }
        else {
            gyro_bias[0] = 0.0f;	// prevent integral windup
            gyro_bias[1] = 0.0f;
            gyro_bias[2] = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

            

    // Time derivative of quaternion. q_dot = 0.5*q\otimes omega.
    //! q_k = q_{k-1} + dt*\dot{q}
    //! \dot{q} = 0.5*q \otimes P(\omega)
    dq0 = 0.5f*(-q1 * gx - q2 * gy - q3 * gz);
    dq1 = 0.5f*(q0 * gx + q2 * gz - q3 * gy);
    dq2 = 0.5f*(q0 * gy - q1 * gz + q3 * gx);
    dq3 = 0.5f*(q0 * gz + q1 * gy - q2 * gx);

    q0 += dt*dq0;
    q1 += dt*dq1;
    q2 += dt*dq2;
    q3 += dt*dq3;

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;
}

#include "Sensor_Basic.h"
float euler[3] = {0.0f, 0.0f, 0.0f};
float gyro_offsets_sum[3]= { 0.0f, 0.0f, 0.0f };
float gyroOffset[3]= { 0.0f, 0.0f, 0.0f };
float accel_offsets_sum[3] = { 0.0f, 0.0f, 0.0f };
float accelOffset[3]= { 0.0f, 0.0f, 0.0f };
float Rot_matrix[9] = {1.f,  0.0f,  0.0f, 0.0f,  1.f,  0.0f, 0.0f,  0.0f,  1.f };
int offset_count = 0;
uint8_t ready = 0;
void ImuUpdate_Task(uint32_t dT_ms){
	if(!ready)
	{
			gyro_offsets_sum[0] += sensor.gyro_rps[0];
			gyro_offsets_sum[1] += sensor.gyro_rps[1];
			gyro_offsets_sum[2] += sensor.gyro_rps[2];
			accel_offsets_sum[0] += sensor.accel_mpss[0];
			accel_offsets_sum[1] += sensor.accel_mpss[1];
			accel_offsets_sum[2] += sensor.accel_mpss[2];
			offset_count++;
			if(offset_count >= 3000)
			{
					gyroOffset[0] = gyro_offsets_sum[0] / offset_count;
					gyroOffset[1] = gyro_offsets_sum[1] / offset_count;
					gyroOffset[2] = gyro_offsets_sum[2] / offset_count;
					accelOffset[0] = accel_offsets_sum[0]/offset_count;
					accelOffset[1] = accel_offsets_sum[1]/offset_count;
					accelOffset[2] = accel_offsets_sum[2]/offset_count;
					accelOffset[2]-= CONSTANTS_ONE_G;
					offset_count=0;
					gyro_offsets_sum[0]=0;
					gyro_offsets_sum[1]=0;
					gyro_offsets_sum[2]=0;
					ready = 1;
			}
			return;
	}
	
	if(flag.motionless)
		return;

	sensor.gyro_rps[0] -= gyroOffset[0];
	sensor.gyro_rps[1] -= gyroOffset[1];
	sensor.gyro_rps[2] -= gyroOffset[2];

	sensor.accel_mpss[0] -= accelOffset[0];
	sensor.accel_mpss[1] -= accelOffset[1];
	sensor.accel_mpss[2] -= accelOffset[2];


	NonlinearSO3AHRSupdate(sensor.gyro_rps[0], sensor.gyro_rps[1], sensor.gyro_rps[2], 
									sensor.accel_mpss[0], sensor.accel_mpss[1], sensor.accel_mpss[2],
									0, 0, 0, 
									so3_comp_params_Kp,
									so3_comp_params_Ki,
									dT_ms * 1e-3);
	
	Rot_matrix[0] = q0q0 + q1q1 - q2q2 - q3q3;// 11
	Rot_matrix[1] = 2.f * (q1*q2 + q0*q3);	// 12
	Rot_matrix[2] = 2.f * (q1*q3 - q0*q2);	// 13
	Rot_matrix[3] = 2.f * (q1*q2 - q0*q3);	// 21
	Rot_matrix[4] = q0q0 - q1q1 + q2q2 - q3q3;// 22
	Rot_matrix[5] = 2.f * (q2*q3 + q0*q1);	// 23
	Rot_matrix[6] = 2.f * (q1*q3 + q0*q2);	// 31
	Rot_matrix[7] = 2.f * (q2*q3 - q0*q1);	// 32
	Rot_matrix[8] = q0q0 - q1q1 - q2q2 + q3q3;// 33
	
	float R2D = 57.2957795f; //180.0f / 3.145926535f;
	imu_data.rol = fast_atan2(Rot_matrix[5], Rot_matrix[8]) * R2D;	//! Roll
	imu_data.pit = -my_sin(Rot_matrix[2]) * R2D;									//! Pitch
	imu_data.yaw = fast_atan2(Rot_matrix[1], Rot_matrix[0]) * R2D;

	if(imu_data.yaw > 0)
		imu_data.yaw = 180 - imu_data.yaw;
	else
		imu_data.yaw = -180 - imu_data.yaw;
	
	imu_data.yaw = -imu_data.yaw;

}