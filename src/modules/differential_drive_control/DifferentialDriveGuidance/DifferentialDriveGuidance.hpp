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

/**
 * @brief Enum class for the different states of guidance.
 */
enum class GuidanceState {
	TURNING, ///< The vehicle is currently turning.
	DRIVING, ///< The vehicle is currently driving straight.
	GOAL_REACHED ///< The vehicle has reached its goal.
};

/**
 * @brief Class for differential drive guidance.
 */
class DifferentialDriveGuidance : public ModuleParams
{
public:
	/**
	 * @brief Constructor for DifferentialDriveGuidance.
	 * @param parent The parent ModuleParams object.
	 */
	DifferentialDriveGuidance(ModuleParams *parent);

	/**
	 * @brief Default destructor.
	 */
	~DifferentialDriveGuidance() = default;

	/**
	 * @brief Compute guidance for the vehicle.
	 * @param global_pos The global position of the vehicle.
	 * @param current_waypoint The current waypoint the vehicle is heading towards.
	 * @param next_waypoint The next waypoint the vehicle will head towards after reaching the current waypoint.
	 * @param vehicle_yaw The yaw orientation of the vehicle.
	 * @param body_velocity The velocity of the vehicle.
	 * @param angular_velocity The angular velocity of the vehicle.
	 * @param dt The time step.
	 * @return A 2D vector containing the computed guidance.
	 */
	matrix::Vector2f computeGuidance(const matrix::Vector2d &global_pos, const matrix::Vector2d &current_waypoint,
					 const matrix::Vector2d &next_waypoint, float vehicle_yaw,
					 float body_velocity, float angular_velocity, float dt);

	/**
	 * @brief Set the maximum speed for the vehicle.
	 * @param max_speed The maximum speed.
	 * @return The set maximum speed.
	 */
	float setMaxSpeed(float max_speed) { return _max_speed = max_speed; }


	/**
	 * @brief Set the maximum angular velocity for the vehicle.
	 * @param max_angular_velocity The maximum angular velocity.
	 * @return The set maximum angular velocity.
	 */
	float setMaxAngularVelocity(float max_angular_velocity) { return _max_angular_velocity = max_angular_velocity; }

protected:
	/**
	 * @brief Update the parameters of the module.
	 */
	void updateParams() override;

private:
	GuidanceState currentState; ///< The current state of guidance.

	float _vel; ///< The current velocity.
	float _desired_angular_velocity; ///< The desired angular velocity.

	float _max_speed; ///< The maximum speed.
	float _max_angular_velocity; ///< The maximum angular velocity.

	matrix::Vector2d _next_waypoint; ///< The next waypoint.

	VelocitySmoothing _forwards_velocity_smoothing; ///< The velocity smoothing for forward motion.
	PositionSmoothing _position_smoothing; ///< The position smoothing.

	PID_t yaw_rate_pid; ///< The PID controller for yaw rate.
	PID_t velocity_pid; ///< The PID controller for velocity.

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
