/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
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

#include <stddef.h>

#include "uORB.h"

static constexpr size_t ORB_TOPICS_COUNT{160};
static constexpr size_t orb_topics_count() { return ORB_TOPICS_COUNT; }

/*
 * Returns array of topics metadata
 */
extern const struct orb_metadata *const *orb_get_topics() __EXPORT;

enum class ORB_ID : uint8_t {
	actuator_armed = 0,
	actuator_controls = 1,
	actuator_controls_0 = 2,
	actuator_controls_1 = 3,
	actuator_controls_2 = 4,
	actuator_controls_3 = 5,
	actuator_controls_virtual_fw = 6,
	actuator_controls_virtual_mc = 7,
	actuator_outputs = 8,
	adc_report = 9,
	airspeed = 10,
	airspeed_validated = 11,
	battery_status = 12,
	camera_capture = 13,
	camera_trigger = 14,
	camera_trigger_secondary = 15,
	cellular_status = 16,
	collision_constraints = 17,
	collision_report = 18,
	commander_state = 19,
	cpuload = 20,
	debug_array = 21,
	debug_key_value = 22,
	debug_value = 23,
	debug_vect = 24,
	differential_pressure = 25,
	distance_sensor = 26,
	ekf2_timestamps = 27,
	ekf_gps_drift = 28,
	ekf_gps_position = 29,
	esc_report = 30,
	esc_status = 31,
	estimator_innovation_test_ratios = 32,
	estimator_innovation_variances = 33,
	estimator_innovations = 34,
	estimator_sensor_bias = 35,
	estimator_states = 36,
	estimator_status = 37,
	follow_target = 38,
	fw_virtual_attitude_setpoint = 39,
	geofence_result = 40,
	gps_dump = 41,
	gps_inject_data = 42,
	home_position = 43,
	hover_thrust_estimate = 44,
	input_rc = 45,
	iridiumsbd_status = 46,
	irlock_report = 47,
	landing_gear = 48,
	landing_target_innovations = 49,
	landing_target_pose = 50,
	led_control = 51,
	log_message = 52,
	logger_status = 53,
	manual_control_setpoint = 54,
	mavlink_log = 55,
	mc_virtual_attitude_setpoint = 56,
	mission = 57,
	mission_result = 58,
	mount_orientation = 59,
	multirotor_motor_limits = 60,
	obstacle_distance = 61,
	obstacle_distance_fused = 62,
	offboard_control_mode = 63,
	onboard_computer_status = 64,
	optical_flow = 65,
	orb_multitest = 66,
	orb_test = 67,
	orb_test_large = 68,
	orb_test_medium = 69,
	orb_test_medium_multi = 70,
	orb_test_medium_queue = 71,
	orb_test_medium_queue_poll = 72,
	orbit_status = 73,
	parameter_update = 74,
	ping = 75,
	position_controller_landing_status = 76,
	position_controller_status = 77,
	position_setpoint = 78,
	position_setpoint_triplet = 79,
	power_button_state = 80,
	power_monitor = 81,
	pwm_input = 82,
	px4io_status = 83,
	qshell_req = 84,
	qshell_retval = 85,
	radio_status = 86,
	rate_ctrl_status = 87,
	rc_channels = 88,
	rc_parameter_map = 89,
	rpm = 90,
	safety = 91,
	satellite_info = 92,
	sensor_accel = 93,
	sensor_accel_fifo = 94,
	sensor_baro = 95,
	sensor_combined = 96,
	sensor_correction = 97,
	sensor_gyro = 98,
	sensor_gyro_fifo = 99,
	sensor_mag = 100,
	sensor_preflight_mag = 101,
	sensor_selection = 102,
	sensors_status_imu = 103,
	subsystem_info = 104,
	system_power = 105,
	task_stack_info = 106,
	tecs_status = 107,
	telemetry_heartbeat = 108,
	telemetry_status = 109,
	test_motor = 110,
	timesync = 111,
	timesync_status = 112,
	trajectory_bezier = 113,
	trajectory_setpoint = 114,
	trajectory_waypoint = 115,
	transponder_report = 116,
	tune_control = 117,
	uavcan_parameter_request = 118,
	uavcan_parameter_value = 119,
	ulog_stream = 120,
	ulog_stream_ack = 121,
	vehicle_acceleration = 122,
	vehicle_air_data = 123,
	vehicle_angular_acceleration = 124,
	vehicle_angular_velocity = 125,
	vehicle_angular_velocity_groundtruth = 126,
	vehicle_attitude = 127,
	vehicle_attitude_groundtruth = 128,
	vehicle_attitude_setpoint = 129,
	vehicle_command = 130,
	vehicle_command_ack = 131,
	vehicle_constraints = 132,
	vehicle_control_mode = 133,
	vehicle_global_position = 134,
	vehicle_global_position_groundtruth = 135,
	vehicle_gps_position = 136,
	vehicle_imu = 137,
	vehicle_imu_status = 138,
	vehicle_land_detected = 139,
	vehicle_local_position = 140,
	vehicle_local_position_groundtruth = 141,
	vehicle_local_position_setpoint = 142,
	vehicle_magnetometer = 143,
	vehicle_mocap_odometry = 144,
	vehicle_odometry = 145,
	vehicle_rates_setpoint = 146,
	vehicle_roi = 147,
	vehicle_status = 148,
	vehicle_status_flags = 149,
	vehicle_trajectory_bezier = 150,
	vehicle_trajectory_waypoint = 151,
	vehicle_trajectory_waypoint_desired = 152,
	vehicle_vision_attitude = 153,
	vehicle_visual_odometry = 154,
	vehicle_visual_odometry_aligned = 155,
	vtol_vehicle_status = 156,
	wheel_encoders = 157,
	wind_estimate = 158,
	yaw_estimator_status = 159,

	INVALID
};

const struct orb_metadata *get_orb_meta(ORB_ID id);
