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

#include <lib/pid/pid.h>


enum class GuidanceState {
	TURNING,
	DRIVING,
	GOAL_REACHED
};

class DifferentialDriveGuidance : public ModuleParams
{
public:
	DifferentialDriveGuidance(ModuleParams *parent);
	~DifferentialDriveGuidance() = default;

	matrix::Vector2f computeGuidance(const matrix::Vector2d &current_pos, const matrix::Vector2d &current_waypoint,
					 const matrix::Vector2d &next_waypoint, float vehicle_yaw,
					 float body_velocity, float angular_velocity, float dt);
	float normalizeAngle(float angle);

	float setMaxSpeed(float max_speed) { return _max_speed = max_speed; }
	float setMaxAngularVelocity(float max_angular_velocity) { return _max_angular_velocity = max_angular_velocity; }


protected:
	void updateParams() override;

private:
	GuidanceState currentState;

	float _vel{0.0f};
	float _ang_vel{0.0f};

	float _max_speed{0.0f};
	float _max_angular_velocity{0.0f};

	matrix::Vector2d _next_waypoint{0.0f, 0.0f};

	VelocitySmoothing _forwards_velocity_smoothing;
	PositionSmoothing _position_smoothing;

	PID_t yaw_rate_pid;
	PID_t velocity_pid;

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::RDD_P_YAW_RATE>) _param_rdd_p_gain_yaw_rate,
		(ParamFloat<px4::params::RDD_I_YAW_RATE>) _param_rdd_d_gain_yaw_rate,
		(ParamFloat<px4::params::RDD_D_YAW_RATE>) _param_rdd_i_gain_yaw_rate,
		(ParamFloat<px4::params::RDD_P_SPEED>) _param_rdd_p_gain_speed,
		(ParamFloat<px4::params::RDD_I_SPEED>) _param_rdd_i_gain_speed,
		(ParamFloat<px4::params::RDD_D_SPEED>) _param_rdd_d_gain_speed,
		(ParamFloat<px4::params::NAV_ACC_RAD>) _param_rdd_accepted_waypoint_radius,
		(ParamFloat<px4::params::RDD_VEL_ALGN>) _param_rdd_velocity_alignment_subtraction,
		(ParamFloat<px4::params::RDD_WAYPT_OFST>) _param_rdd_waypoint_offset,
		(ParamFloat<px4::params::RDD_MAX_JERK>) _param_rdd_max_jerk,
		(ParamFloat<px4::params::RDD_MAX_ACCEL>) _param_rdd_max_accel
	)

};
