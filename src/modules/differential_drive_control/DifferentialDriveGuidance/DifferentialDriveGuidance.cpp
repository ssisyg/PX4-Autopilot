#include "DifferentialDriveGuidance.hpp"

DifferentialDriveGuidance::DifferentialDriveGuidance(ModuleParams *parent) : ModuleParams(parent)
{
	pid_init(&yaw_rate_pid, PID_MODE_DERIVATIV_NONE, 0.001f); // Replace dt with actual minimum timestep
	pid_set_parameters(&yaw_rate_pid,
			   _param_rdd_p_gain_yaw_rate.get(),  // Proportional gain
			   _param_rdd_i_gain_yaw_rate.get(),  // Integral gain
			   _param_rdd_d_gain_yaw_rate.get(),  // Derivative gain
			   200,  // Integral limit
			   200);  // Output limit

	pid_init(&velocity_pid, PID_MODE_DERIVATIV_NONE, 0.001f); // Replace dt with actual minimum timestep
	pid_set_parameters(&velocity_pid,
			   _param_rdd_p_gain_speed.get(),  // Proportional gain
			   _param_rdd_i_gain_speed.get(),  // Integral gain
			   _param_rdd_d_gain_speed.get(),  // Derivative gain
			   200,  // Integral limit
			   200);  // Output limit

	_forwards_velocity_smoothing.setMaxJerk(_param_rdd_max_jerk.get());
	_forwards_velocity_smoothing.setMaxAccel(_param_rdd_max_accel.get());
	_forwards_velocity_smoothing.setMaxVel(_param_rdd_max_speed.get());

	buffer_index = 0;

	// Manually initialize the buffers to zero
	for (int i = 0; i < BUFFER_SIZE; ++i) {
		vel_buffer[i] = 0.0f;
		ang_vel_dot_buffer[i] = 0.0f;
	}
}


matrix::Vector2f DifferentialDriveGuidance::computeGuidance(const matrix::Vector2f &current_pos,
		const matrix::Vector2f &current_waypoint, const matrix::Vector2f &previous_waypoint,
		const matrix::Vector2f &next_waypoint,
		float vehicle_yaw, float body_velocity, float angular_velocity, float dt)
{

	pid_set_parameters(&velocity_pid,
			   _param_rdd_p_gain_speed.get(),  // Proportional gain
			   _param_rdd_i_gain_speed.get(),  // Integral gain
			   _param_rdd_d_gain_speed.get(),  // Derivative gain
			   200,  // Integral limit
			   200);  // Output limit

	pid_set_parameters(&yaw_rate_pid,
			   _param_rdd_p_gain_yaw_rate.get(),  // Proportional gain
			   _param_rdd_i_gain_yaw_rate.get(),  // Integral gain
			   _param_rdd_d_gain_yaw_rate.get(),  // Derivative gain
			   200,  // Integral limit
			   200);  // Output limit


	float desired_linear_velocity = _param_rdd_max_speed.get();
	float distance_to_next_wp = get_distance_to_next_waypoint(current_pos(0), current_pos(1), current_waypoint(0),
				    current_waypoint(1));
	float desired_heading = computeAdvancedBearing(current_pos, current_waypoint, previous_waypoint, distance_to_next_wp);
	float heading_error = normalizeAngle(desired_heading - vehicle_yaw);

	const float max_velocity = math::trajectory::computeMaxSpeedFromDistance(_param_rdd_max_jerk.get(),
				   _param_rdd_max_accel.get(), distance_to_next_wp, 1.f);
	_forwards_velocity_smoothing.updateDurations(max_velocity);
	_forwards_velocity_smoothing.updateTraj(dt);

	desired_linear_velocity = _forwards_velocity_smoothing.getCurrentVelocity();

	printf("heading_error: %f\n", (double)heading_error);
	printf("desired_linear_velocity: %f\n", (double)desired_linear_velocity);


	desired_linear_velocity = math::interpolate<float>(abs(heading_error), 0.2f, 0.4f, desired_linear_velocity, 0.0f);

	printf("desired_linear_velocity after: %f\n", (double)desired_linear_velocity);

	if (!PX4_ISFINITE(desired_linear_velocity) || desired_linear_velocity < 0) {
		desired_linear_velocity = 0;
	}

	matrix::Vector2f output;

	// logic to stop at the last waypoint
	if ((current_waypoint == next_waypoint) && distance_to_next_wp < _param_rdd_accepted_waypoint_radius.get()) {
		output(0) = 0;
		output(1) = 0;

	} else {
		float vel_dot = pid_calculate(&velocity_pid, desired_linear_velocity, body_velocity, 0, dt);
		float ang_vel_dot = pid_calculate(&yaw_rate_pid, heading_error, angular_velocity, 0, dt);

		// Update buffers
		vel_buffer[buffer_index] = vel_dot;
		ang_vel_dot_buffer[buffer_index] = ang_vel_dot;
		buffer_index = (buffer_index + 1) % BUFFER_SIZE; // Circular buffer logic

		// Compute moving averages
		float vel_moving_avg = 0;
		float ang_vel_dot_moving_avg = 0;

		for (int i = 0; i < BUFFER_SIZE; ++i) {
			vel_moving_avg += vel_buffer[i];
			ang_vel_dot_moving_avg += ang_vel_dot_buffer[i];
		}

		vel_moving_avg /= BUFFER_SIZE;
		ang_vel_dot_moving_avg /= BUFFER_SIZE;

		// Use moving averages instead of raw values
		vel_dot = vel_moving_avg;
		ang_vel_dot = ang_vel_dot_moving_avg;

		_vel += vel_dot;
		// _ang_vel += ang_vel_dot;

		// if (abs(_vel) > _param_rdd_max_speed.get()) {
		// 	_vel = _param_rdd_max_speed.get() * matrix::sign(_vel);
		// }


		// if (abs(ang_vel_dot) > _param_rdd_max_angular_velocity.get()) {
		// 	ang_vel_dot = _param_rdd_max_angular_velocity.get() * matrix::sign(ang_vel_dot);
		// }

		output(0) = _vel;
		output(1) = ang_vel_dot;

		printf("angular velocity: %f\n", (double)angular_velocity);
		printf("ang_vel_dot: %f\n", (double)ang_vel_dot);

		printf("\n");
	}

	return output;
}


float DifferentialDriveGuidance::computeAdvancedBearing(const matrix::Vector2f &current_pos,
		const matrix::Vector2f &waypoint,
		const matrix::Vector2f &previous_waypoint,
		float distance_to_next_wp)
{
	matrix::Vector2f wanted_path = waypoint - previous_waypoint;
	matrix::Vector2f current_path = current_pos - previous_waypoint;

	// Normalize the vectors
	matrix::Vector2f wanted_path_normalized = wanted_path;
	matrix::Vector2f current_path_normalized = current_path;

	wanted_path_normalized.normalize();
	current_path_normalized.normalize();

	float path_dot_product = wanted_path_normalized.dot(current_path_normalized);
	float theta = acos(path_dot_product);

	float total_distance = get_distance_to_next_waypoint(previous_waypoint(0), previous_waypoint(1), waypoint(0),
			       waypoint(1));

	float sensitivity_scale = (distance_to_next_wp / total_distance) > 0.1f ? (distance_to_next_wp / total_distance) : 0.1f;

	matrix::Vector2f new_waypoint = waypoint + sensitivity_scale * _param_rdd_waypoint_offset.get() * ((
						wanted_path_normalized * cos(
								theta) * current_path.norm() + previous_waypoint) - current_pos);

	return computeBearing(current_pos, new_waypoint);
}


float DifferentialDriveGuidance::computeBearing(const matrix::Vector2f &current_pos, const matrix::Vector2f &waypoint)
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
