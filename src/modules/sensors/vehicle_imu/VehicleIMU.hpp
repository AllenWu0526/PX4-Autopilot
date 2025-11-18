/****************************************************************************
 *
 *   Copyright (c) 2020-2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <Integrator.hpp>

#include <lib/mathlib/math/Limits.hpp>
#include <lib/mathlib/math/WelfordMean.hpp>
#include <lib/mathlib/math/WelfordMeanVector.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <lib/sensor_calibration/Accelerometer.hpp>
#include <lib/sensor_calibration/Gyroscope.hpp>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/estimator_sensor_bias.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_imu.h>
#include <uORB/topics/vehicle_imu_status.h>

//CW HPF for vibration extraction
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>

using namespace time_literals;

//CW added 2nd HPF/LPF
template<typename T>
class MyFilter2p
{
public:
	MyFilter2p() = default;
	MyFilter2p(bool isHP, float cutoff_freq)
	{
		// set initial parameters
		_cutoff_freq = cutoff_freq;
		_isHP = isHP;
	}

	// Change filter parameters
	void set_cutoff_frequency(float sample_freq)
	{
		// reset delay elements on filter change
		//_delay_element_1 = {};
		//_delay_element_2 = {};

		//_cutoff_freq = cutoff_freq;
		_sample_freq = sample_freq;

		const float fr = _sample_freq / _cutoff_freq;
		const float ohm = tanf(M_PI_F / fr);
		const float c = 1.f + 2.f * cosf(M_PI_F / 4.f) * ohm + ohm * ohm;

		if(!_isHP){
		//original LPF2
		_b0 = ohm * ohm / c;
		_b1 = 2.f * _b0;
		_b2 = _b0;
		}else{
		//change to HPF2
		_b0 = 1.f / c;
		_b1 = (-2.f) * _b0;
		_b2 = _b0;
		}

		_a1 = 2.f * (ohm * ohm - 1.f) / c;
		_a2 = (1.f - 2.f * cosf(M_PI_F / 4.f) * ohm + ohm * ohm) / c;
	}
	//Just update cutoff
	void set_cutoff_only(float cutoff_freq){
		_cutoff_freq = cutoff_freq;
	}

	/**
	 * Add a new raw value to the filter
	 *
	 * @return retrieve the filtered result
	 */
	inline T apply(const T &sample, float dt)
	{
		set_cutoff_frequency(1.0f/dt);
		// Direct Form II implementation
		T delay_element_0{sample - _delay_element_1 *_a1 - _delay_element_2 * _a2};

		const T output{delay_element_0 *_b0 + _delay_element_1 *_b1 + _delay_element_2 * _b2};

		_delay_element_2 = _delay_element_1;
		_delay_element_1 = delay_element_0;

		return output;
	}

	// Filter array of samples in place using the Direct form II.
	inline void applyArray(T samples[], int num_samples)
	{
		for (int n = 0; n < num_samples; n++) {
			samples[n] = apply(samples[n]);
		}
	}

	// Return the cutoff frequency
	float get_cutoff_freq() const { return _cutoff_freq; }

	// Return the sample frequency
	float get_sample_freq() const { return _sample_freq; }

	float getMagnitudeResponse(float frequency) const;

	// Reset the filter state to this value
	T reset(const T &sample)
	{
		const T input = isFinite(sample) ? sample : T{};

		if (fabsf(1 + _a1 + _a2) > FLT_EPSILON) {
			_delay_element_1 = _delay_element_2 = input / (1 + _a1 + _a2);

			if (!isFinite(_delay_element_1) || !isFinite(_delay_element_2)) {
				_delay_element_1 = _delay_element_2 = input;
			}

		} else {
			_delay_element_1 = _delay_element_2 = input;
		}

		return apply(input);
	}

	void disable()
	{
		// no filtering
		_sample_freq = 0.f;
		_cutoff_freq = 0.f;

		_delay_element_1 = {};
		_delay_element_2 = {};

		_b0 = 1.f;
		_b1 = 0.f;
		_b2 = 0.f;

		_a1 = 0.f;
		_a2 = 0.f;
	}

protected:
	T _delay_element_1{}; // buffered sample -1
	T _delay_element_2{}; // buffered sample -2

	// All the coefficients are normalized by a0, so a0 becomes 1 here
	float _a1{0.f};
	float _a2{0.f};

	float _b0{1.f};
	float _b1{0.f};
	float _b2{0.f};

	float _cutoff_freq{0.f};
	float _sample_freq{0.f};

	bool _isHP;
};

namespace sensors
{

class VehicleIMU : public ModuleParams, public px4::ScheduledWorkItem
{
public:
	VehicleIMU() = delete;
	VehicleIMU(int instance, uint8_t accel_index, uint8_t gyro_index, const px4::wq_config_t &config);

	~VehicleIMU() override;

	bool Start();
	void Stop();

	void PrintStatus();

private:
	//CW added HPF for sensor_gyro/accel instance #0, process in #1 vehicle IMU
	//CW added for #0 (vibration)
	bool isVibHIL = false;
	static constexpr float _cutoff_freq_hz = 50;
	static constexpr float _sample_rate_hz = 200; //only use for the first time ! (we used time variant HPF2 !)

	uORB::Subscription* _sensor_accel_sub0;
	uORB::Subscription* _sensor_gyro_sub0;
	uORB::Subscription* _sensor_accel_sub1;
	uORB::Subscription* _sensor_gyro_sub1;

	PX4Accelerometer _px4_accel;
	PX4Gyroscope _px4_gyro;

	MyFilter2p<float> _accel_hpf2_x;
	MyFilter2p<float> _accel_hpf2_y;
	MyFilter2p<float> _accel_hpf2_z;
	MyFilter2p<float> _gyro_hpf2_x;
	MyFilter2p<float> _gyro_hpf2_y;
	MyFilter2p<float> _gyro_hpf2_z;

	bool ParametersUpdate(bool force = false);
	bool Publish();
	void Run() override;

	bool UpdateAccel();
	bool UpdateGyro();

	void UpdateIntegratorConfiguration();

	inline void UpdateAccelVibrationMetrics(const matrix::Vector3f &acceleration);
	inline void UpdateGyroVibrationMetrics(const matrix::Vector3f &angular_velocity);

	void SensorCalibrationUpdate();
	void SensorCalibrationSaveAccel();
	void SensorCalibrationSaveGyro();

	// return the square of two floating point numbers
	static constexpr float sq(float var) { return var * var; }

	uORB::PublicationMulti<vehicle_imu_s> _vehicle_imu_pub{ORB_ID(vehicle_imu)};
	uORB::PublicationMulti<vehicle_imu_status_s> _vehicle_imu_status_pub{ORB_ID(vehicle_imu_status)};

	static constexpr hrt_abstime kIMUStatusPublishingInterval{100_ms};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	// Used to check, save and use learned magnetometer biases
	uORB::SubscriptionMultiArray<estimator_sensor_bias_s> _estimator_sensor_bias_subs{ORB_ID::estimator_sensor_bias};

	uORB::Subscription _sensor_accel_sub;
	uORB::SubscriptionCallbackWorkItem _sensor_gyro_sub;

	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};

	calibration::Accelerometer _accel_calibration{};
	calibration::Gyroscope _gyro_calibration{};

	sensors::Integrator       _accel_integrator{};
	sensors::IntegratorConing _gyro_integrator{};

	uint32_t _imu_integration_interval_us{5000};

	hrt_abstime _accel_timestamp_sample_last{0};
	hrt_abstime _gyro_timestamp_sample_last{0};
	hrt_abstime _gyro_timestamp_last{0};

	math::WelfordMeanVector<float, 3> _raw_accel_mean{};
	math::WelfordMeanVector<float, 3> _raw_gyro_mean{};

	math::WelfordMean<float> _accel_mean_interval_us{};
	math::WelfordMean<float> _accel_fifo_mean_interval_us{};

	math::WelfordMean<float> _gyro_mean_interval_us{};
	math::WelfordMean<float> _gyro_fifo_mean_interval_us{};

	math::WelfordMean<float> _gyro_update_latency_mean_us{};
	math::WelfordMean<float> _gyro_publish_latency_mean_us{};

	float _accel_interval_us{NAN};
	float _gyro_interval_us{NAN};

	unsigned _accel_last_generation{0};
	unsigned _gyro_last_generation{0};

	float _accel_temperature_sum{NAN};
	float _gyro_temperature_sum{NAN};

	int _accel_temperature_sum_count{0};
	int _gyro_temperature_sum_count{0};

	matrix::Vector3f _acceleration_prev{};     // acceleration from the previous IMU measurement for vibration metrics
	matrix::Vector3f _angular_velocity_prev{}; // angular velocity from the previous IMU measurement for vibration metrics

	vehicle_imu_status_s _status{};

	float _coning_norm_accum{0};
	float _coning_norm_accum_total_time_s{0};

	uint8_t     _delta_angle_clipping{0};
	uint8_t     _delta_velocity_clipping{0};

	bool _notify_clipping{true};

	hrt_abstime _last_accel_clipping_notify_time{0};
	hrt_abstime _last_gyro_clipping_notify_time{0};

	uint64_t    _last_accel_clipping_notify_total_count{0};
	uint64_t    _last_gyro_clipping_notify_total_count{0};

	orb_advert_t _mavlink_log_pub{nullptr};

	uint32_t _backup_schedule_timeout_us{20000};

	bool _data_gap{false};
	bool _update_integrator_config{true};
	bool _intervals_configured{false};
	bool _publish_status{true};

	const uint8_t _instance;

	bool _armed{false};

	bool _accel_cal_available{false};
	bool _gyro_cal_available{false};

	struct InFlightCalibration {
		matrix::Vector3f offset{};
		matrix::Vector3f bias_variance{};
		bool valid{false};
	};

	InFlightCalibration _accel_learned_calibration[ORB_MULTI_MAX_INSTANCES] {};
	InFlightCalibration _gyro_learned_calibration[ORB_MULTI_MAX_INSTANCES] {};

	static constexpr hrt_abstime INFLIGHT_CALIBRATION_QUIET_PERIOD_US{30_s};

	hrt_abstime _in_flight_calibration_check_timestamp_last{0};

	perf_counter_t _accel_generation_gap_perf{perf_alloc(PC_COUNT, MODULE_NAME": accel data gap")};
	perf_counter_t _gyro_generation_gap_perf{perf_alloc(PC_COUNT, MODULE_NAME": gyro data gap")};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::IMU_INTEG_RATE>) _param_imu_integ_rate,
		(ParamBool<px4::params::SENS_IMU_AUTOCAL>) _param_sens_imu_autocal,
		(ParamBool<px4::params::SENS_IMU_CLPNOTI>) _param_sens_imu_notify_clipping,
		(ParamFloat<px4::params::HIL_VIB_HP_CUTF>) _param_hil_vib_hp_cutf,
		(ParamBool<px4::params::COM_HIL_VIB_TEST>)  _param_com_hil_vib_test,
		(ParamInt<px4::params::SYS_HITL>) _param_sys_hil
	)
};

} // namespace sensors
