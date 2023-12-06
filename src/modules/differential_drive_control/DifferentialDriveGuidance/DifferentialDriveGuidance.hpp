/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>

#include <matrix/matrix/math.hpp>
#include <lib/geo/geo.h>
#include <math.h>

#include <lib/motion_planning/PositionSmoothing.hpp>
#include <lib/motion_planning/VelocitySmoothing.hpp>

// #include "rover_drive_control_pid.hpp"
#include <lib/pid/pid.h>



class DifferentialDriveGuidance : public ModuleParams
{
public:
	DifferentialDriveGuidance(ModuleParams *parent);
	~DifferentialDriveGuidance() = default;

	matrix::Vector2f 	computeGuidance(const matrix::Vector2f &current_pos, const matrix::Vector2f &waypoint,
						const matrix::Vector2f &previous_waypoint, const matrix::Vector2f &next_waypoint, float vehicle_yaw, float dt);
	float 	computeAdvancedBearing(const matrix::Vector2f &current_pos, const matrix::Vector2f &waypoint,
				       const matrix::Vector2f &previous_waypoint);
	float 	computeBearing(const matrix::Vector2f &current_pos, const matrix::Vector2f &waypoint);
	float 	normalizeAngle(float angle);
	float 	computeAlignment(const matrix::Vector2f &current_pos, const matrix::Vector2f &waypoint,
				 const matrix::Vector2f &previous_waypoint);

private:

	// Input & Output (Don't really need input tbh, but lets see)
	matrix::Vector2f _input{0.0f, 0.0f};  // input_[0] -> Vx [m/s], input_[1] -> Omega [rad/s]
	matrix::Vector2f _output{0.0f, 0.0f}; // _output[0] -> Right Motor [rad/s], _output[1] -> Left Motor [rad/s]

	matrix::Vector2f _global_position{0.0, 0.0};
	matrix::Vector2f _local_position{0.0, 0.0};
	matrix::Vector2f _current_waypoint{0.0, 0.0};
	matrix::Vector2f _previous_waypoint{0.0, 0.0};
	matrix::Vector2f _next_waypoint{0.0, 0.0};

	float _dt{1.0};

	VelocitySmoothing _forwards_velocity_smoothing;
	PositionSmoothing _position_smoothing;

	// rover_drive_control_pid 		_yaw_rate_point_pid;
	// rover_drive_control_pid 		_yaw_rate_align_pid;
	// rover_drive_control_pid 		_speed_control_pid;

	PID_t yaw_rate_pid;

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::RDD_MAX_SPEED>) _param_rdd_max_speed,
		(ParamFloat<px4::params::RDD_MAX_ANG_VEL>) _param_rdd_max_angular_velocity,
		(ParamFloat<px4::params::RDD_P_GAIN_GUIDE>) _param_rdc_p_gain_waypoint_controller,
		(ParamFloat<px4::params::RDD_I_GAIN_GUIDE>) _param_rdc_d_gain_waypoint_controller,
		(ParamFloat<px4::params::RDD_D_GAIN_GUIDE>) _param_rdc_i_gain_waypoint_controller,
		(ParamFloat<px4::params::NAV_ACC_RAD>) _param_rdc_accepted_waypoint_radius,
		(ParamFloat<px4::params::RDD_VEL_ALGN>) _param_rdc_velocity_alignment_subtraction
		// (ParamFloat<px4::params::RDC_MAX_JERK>) _param_rdc_max_jerk,
		// (ParamFloat<px4::params::RDC_MAX_ACCEL>) _param_rdc_max_acceleration,
		// (ParamFloat<px4::params::RDC_WP_VEL>) _param_rdc_waypoing_min_vel
	)

};
