#include "DifferentialDriveGuidance.hpp"

DifferentialDriveGuidance::DifferentialDriveGuidance(ModuleParams *parent) : ModuleParams(parent)
{
	updateParams();

	pid_init(&yaw_rate_pid, PID_MODE_DERIVATIV_NONE, 0.001f); // Replace dt with actual minimum timestep
	pid_init(&velocity_pid, PID_MODE_DERIVATIV_NONE, 0.001f); // Replace dt with actual minimum timestep
	// _forwards_velocity_smoothing.setMaxJerk(_param_rdd_max_jerk.get());
	// _forwards_velocity_smoothing.setMaxAccel(_param_rdd_max_accel.get());
	// _forwards_velocity_smoothing.setMaxVel(_max_speed);

	// buffer_index = 0;

	// // Manually initialize the buffers to zero
	// for (int i = 0; i < BUFFER_SIZE; ++i) {
	// 	vel_buffer[i] = 0.0f;
	// 	ang_vel_dot_buffer[i] = 0.0f;
	// }
}


matrix::Vector2d DifferentialDriveGuidance::computeGuidance(const matrix::Vector2d &current_pos,
		const matrix::Vector2d &current_waypoint, const matrix::Vector2d &previous_waypoint,
		const matrix::Vector2d &next_waypoint,
		float vehicle_yaw, float body_velocity, float angular_velocity, float dt)
{

	if (_next_waypoint != next_waypoint) {
		_new_waypoint = true;
	}

	printf("previous_waypoint: %f, %f\n", (double)previous_waypoint(0), (double)previous_waypoint(1));
	printf("waypoint: %f, %f\n", (double)current_waypoint(0), (double)current_waypoint(1));
	printf("next_waypoint: %f, %f\n", (double)next_waypoint(0), (double)next_waypoint(1));

	float desired_linear_velocity = _max_speed;
	float distance_to_next_wp = get_distance_to_next_waypoint(current_pos(0), current_pos(1), current_waypoint(0),
				    current_waypoint(1));
	// float desired_heading = computeAdvancedBearing(current_pos, current_waypoint, previous_waypoint, distance_to_next_wp);
	// float desired_heading = computeBearing(current_pos, current_waypoint);
	float desired_heading = get_bearing_to_next_waypoint(current_pos(0), current_pos(1), current_waypoint(0),
				current_waypoint(1));
	float heading_error = normalizeAngle(desired_heading - vehicle_yaw);

	const float max_velocity = math::trajectory::computeMaxSpeedFromDistance(_param_rdd_max_jerk.get(),
				   _param_rdd_max_accel.get(), distance_to_next_wp, 0.0f);
	printf("max_velocity: %f\n", (double)max_velocity);
	_forwards_velocity_smoothing.updateDurations(max_velocity);
	_forwards_velocity_smoothing.updateTraj(dt);

	printf("dt: %f\n", (double)dt);

	desired_linear_velocity = _forwards_velocity_smoothing.getCurrentVelocity();

	printf("heading_error: %f\n", (double)heading_error);
	printf("desired_linear_velocity: %f\n", (double)desired_linear_velocity);

	desired_linear_velocity = math::interpolate<float>(abs(heading_error), 0.1f, 0.2f, desired_linear_velocity, 0.2f);

	if (_new_waypoint) {
		if (fabsf(heading_error) < 0.1f) {
			desired_linear_velocity = math::interpolate<float>(abs(heading_error), 0.1f, 0.2f, desired_linear_velocity, 0.2f);
			_new_waypoint = false;

		} else {
			desired_linear_velocity = 0.f;
		}
	}

	printf("desired_linear_velocity after: %f\n", (double)desired_linear_velocity);

	if (!PX4_ISFINITE(desired_linear_velocity) || desired_linear_velocity < 0) {
		desired_linear_velocity = 0;
	}

	matrix::Vector2d output;

	// logic to stop at the last waypoint
	if ((current_waypoint == next_waypoint) && distance_to_next_wp < _param_rdd_accepted_waypoint_radius.get()) {
		output(0) = 0;
		output(1) = 0;

	} else {

		printf("desired_linear_velocity !!!: %f\n", (double)desired_linear_velocity);
		printf("body_velocity !!!!!!!!!!!!!: %f\n", (double)body_velocity);

		float vel_dot = pid_calculate(&velocity_pid, desired_linear_velocity, body_velocity, 0, dt);
		float ang_vel_dot = pid_calculate(&yaw_rate_pid, heading_error, angular_velocity, 0, dt);

		printf("integral %f\n", (double)velocity_pid.integral);

		desired_linear_velocity += vel_dot;
		_ang_vel += ang_vel_dot;

		output(0) = desired_linear_velocity;
		output(1) = ang_vel_dot;

		printf("angular velocity: %f\n", (double)angular_velocity);
		printf("ang_vel_dot: %f\n", (double)ang_vel_dot);
		printf("desired_linear_velocity: %f\n", (double)desired_linear_velocity);
		printf("vel_dot: %f\n", (double)vel_dot);
		// printf("vel: %f\n", (double)_vel);

		printf("\n");

		_next_waypoint = next_waypoint;
	}

	return output / _max_speed;
}


float DifferentialDriveGuidance::computeBearing(const matrix::Vector2d &current_pos, const matrix::Vector2d &waypoint)
{
	float delta_x = waypoint(0) - current_pos(0);
	float delta_y = waypoint(1) - current_pos(1);
	return std::atan2(delta_y, delta_x);
}

float DifferentialDriveGuidance::normalizeAngle(float angle)
{
	while (angle > M_PIf) { angle -= 2.0f * M_PIf; }

	while (angle < -M_PIf) { angle += 2.0f * M_PIf; }

	return angle;
}

void DifferentialDriveGuidance::updateParams()
{
	ModuleParams::updateParams();

	pid_set_parameters(&yaw_rate_pid,
			   _param_rdd_p_gain_yaw_rate.get(),  // Proportional gain
			   _param_rdd_i_gain_yaw_rate.get(),  // Integral gain
			   _param_rdd_d_gain_yaw_rate.get(),  // Derivative gain
			   20,  // Integral limit
			   200);  // Output limit

	pid_set_parameters(&velocity_pid,
			   _param_rdd_p_gain_speed.get(),  // Proportional gain
			   _param_rdd_i_gain_speed.get(),  // Integral gain
			   _param_rdd_d_gain_speed.get(),  // Derivative gain
			   2,  // Integral limit
			   200);  // Output limit

	printf("I gain: %f\n", (double)_param_rdd_i_gain_speed.get());

	_forwards_velocity_smoothing.setMaxJerk(_param_rdd_max_jerk.get());
	_forwards_velocity_smoothing.setMaxAccel(_param_rdd_max_accel.get());
	_forwards_velocity_smoothing.setMaxVel(_max_speed);
}
