#include "DifferentialDriveGuidance.hpp"

DifferentialDriveGuidance::DifferentialDriveGuidance(ModuleParams *parent) : ModuleParams(parent)
{
	pid_init(&yaw_angle_pid, PID_MODE_DERIVATIV_NONE, 0.001f); // Replace dt with actual minimum timestep
	pid_set_parameters(&yaw_angle_pid,
			   _param_rdc_p_gain_waypoint_controller.get(),  // Proportional gain
			   _param_rdc_i_gain_waypoint_controller.get(),  // Integral gain
			   _param_rdc_d_gain_waypoint_controller.get(),  // Derivative gain
			   200,  // Integral limit
			   200);  // Output limit

	pid_init(&yaw_rate_pid, PID_MODE_DERIVATIV_NONE, 0.001f); // Replace dt with actual minimum timestep
	pid_set_parameters(&yaw_rate_pid,
			   4.0f,  // Proportional gain
			   _param_rdc_i_gain_waypoint_controller.get(),  // Integral gain
			   _param_rdc_d_gain_waypoint_controller.get(),  // Derivative gain
			   200,  // Integral limit
			   200);  // Output limit

	pid_init(&velocity_pid, PID_MODE_DERIVATIV_NONE, 0.001f); // Replace dt with actual minimum timestep
	pid_set_parameters(&velocity_pid,
			   0.1f,  // Proportional gain
			   _param_rdc_i_gain_waypoint_controller.get(),  // Integral gain
			   _param_rdc_d_gain_waypoint_controller.get(),  // Derivative gain
			   200,  // Integral limit
			   200);  // Output limit
}


matrix::Vector2f DifferentialDriveGuidance::computeGuidance(const matrix::Vector2f &current_pos,
		const matrix::Vector2f &current_waypoint, const matrix::Vector2f &previous_waypoint,
		const matrix::Vector2f &next_waypoint,
		float vehicle_yaw, float body_velocity, float angular_velocity, float dt)
{
	float max_forwards_velocity = _param_rdd_max_speed.get();
	float desired_heading = computeAdvancedBearing(current_pos, current_waypoint, previous_waypoint);
	float distance_to_next_wp = get_distance_to_next_waypoint(current_pos(0), current_pos(1), current_waypoint(0),
				    current_waypoint(1));
	float heading_error = normalizeAngle(desired_heading - vehicle_yaw);

	float desired_angular_rate = pid_calculate(&yaw_angle_pid, 0, -heading_error, 0, dt);

	float desired_linear_velocity = max_forwards_velocity;

	// initialize this at the start of the function and get parameters
	// not quite sure about this section, subject to change
	_forwards_velocity_smoothing.setMaxJerk(0.2);
	_forwards_velocity_smoothing.setMaxAccel(0.5);
	_forwards_velocity_smoothing.setMaxVel(max_forwards_velocity);
	const float max_velocity = math::trajectory::computeMaxSpeedFromDistance(1,
				   0.5, distance_to_next_wp / 10.f, 0.2);
	_forwards_velocity_smoothing.updateDurations(max_velocity);
	_forwards_velocity_smoothing.updateTraj(dt);

	desired_linear_velocity = _forwards_velocity_smoothing.getCurrentVelocity();

	// if (abs(heading_error) > 0.1f) {
	// 	desired_linear_velocity = 0;
	// }

	if (!PX4_ISFINITE(desired_linear_velocity) || desired_linear_velocity < 0) {
		desired_linear_velocity = 0;
	}

	matrix::Vector2f output;

	// logic to stop at the last waypoint
	if ((current_waypoint == next_waypoint) && distance_to_next_wp < _param_rdc_accepted_waypoint_radius.get()) {
		output(0) = 0;
		output(1) = 0;

	} else {

		printf("body_velocity, desired_linear_velocity: %f, %f\n", (double)body_velocity, (double)desired_linear_velocity);
		printf("angular_velocity, desired_angular_rate: %f, %f\n", (double)angular_velocity, (double)desired_angular_rate);

		float vel_dot = pid_calculate(&velocity_pid, desired_linear_velocity, body_velocity, 0, dt);
		float ang_vel_dot = pid_calculate(&yaw_rate_pid, desired_angular_rate, angular_velocity, 0, dt);

		printf("\n");

		// printf("body_velocity, desired_linear_velocity AFTER: %f, %f\n", (double)body_velocity, (double)vel);
		// printf("angular_velocity, desired_angular_rate AFTER: %f, %f\n", (double)angular_velocity, (double)ang_vel);

		_vel += vel_dot;
		_ang_vel += ang_vel_dot;

		output(0) = _vel;
		output(1) = _ang_vel;
	}

	return output;
}


float DifferentialDriveGuidance::computeAdvancedBearing(const matrix::Vector2f &current_pos,
		const matrix::Vector2f &waypoint, const matrix::Vector2f &previous_waypoint)
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

	matrix::Vector2f new_waypoint = waypoint + ((wanted_path_normalized * cos(
						theta) * current_path.norm() + previous_waypoint) - current_pos);

	printf("new_waypoint: %f, %f\n", (double)new_waypoint(0), (double)new_waypoint(1));
	printf("waypoint: %f, %f\n", (double)waypoint(0), (double)waypoint(1));
	printf("current_pos: %f, %f\n", (double)current_pos(0), (double)current_pos(1));

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
