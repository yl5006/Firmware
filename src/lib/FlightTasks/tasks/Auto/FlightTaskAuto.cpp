/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
/**
 * @file FlightTaskAuto.cpp
 */

#include "FlightTaskAuto.hpp"
#include <mathlib/mathlib.h>
#include <float.h>

using namespace matrix;

static constexpr float SIGMA_NORM	= 0.001f;

bool FlightTaskAuto::initializeSubscriptions(SubscriptionArray &subscription_array)
{
	if (!FlightTask::initializeSubscriptions(subscription_array)) {
		return false;
	}

	if (!subscription_array.get(ORB_ID(position_setpoint_triplet), _sub_triplet_setpoint)) {
		return false;
	}

	if (!subscription_array.get(ORB_ID(home_position), _sub_home_position)) {
		return false;
	}

	return true;
}

bool FlightTaskAuto::activate()
{
	bool ret = FlightTask::activate();
	_position_setpoint = _position;
	_velocity_setpoint = _velocity;
	_yaw_setpoint = _yaw;
	_yawspeed_setpoint = 0.0f;
	_setDefaultConstraints();
	return ret;
}

bool FlightTaskAuto::updateInitialize()
{
	bool ret = FlightTask::updateInitialize();
	// require valid reference and valid target
	ret = ret && _evaluateGlobalReference() && _evaluateTriplets();
	// require valid position
	ret = ret && PX4_ISFINITE(_position(0))
	      && PX4_ISFINITE(_position(1))
	      && PX4_ISFINITE(_position(2))
	      && PX4_ISFINITE(_velocity(0))
	      && PX4_ISFINITE(_velocity(1))
	      && PX4_ISFINITE(_velocity(2));

	return ret;
}

bool FlightTaskAuto::_evaluateTriplets()
{
	// TODO: fix the issues mentioned below
	// We add here some conditions that are only required because:
	// 1. navigator continuously sends triplet during mission due to yaw setpoint. This
	// should be removed in the navigator and only updates if the current setpoint actually has changed.
	//
	// 2. navigator should be responsible to send always three valid setpoints. If there is only one setpoint,
	// then previous will be set to current vehicle position and next will be set equal to setpoint.
	//
	// 3. navigator originally only supports gps guided maneuvers. However, it now also supports some flow-specific features
	// such as land and takeoff. The navigator should use for auto takeoff/land with flow the position in xy at the moment the
	// takeoff/land was initiated. Until then we do this kind of logic here.

	// Check if triplet is valid. There must be at least a valid altitude.

	if (!_sub_triplet_setpoint->get().current.valid || !PX4_ISFINITE(_sub_triplet_setpoint->get().current.alt)) {
		// Best we can do is to just set all waypoints to current state and return false.
		_prev_prev_wp = _triplet_prev_wp = _triplet_target = _triplet_next_wp = _position;
		_type = WaypointType::position;
		return false;
	}

	_type = (WaypointType)_sub_triplet_setpoint->get().current.type;

	// Always update cruise speed since that can change without waypoint changes.
	_mc_cruise_speed = _sub_triplet_setpoint->get().current.cruising_speed;

	if (!PX4_ISFINITE(_mc_cruise_speed) || (_mc_cruise_speed < 0.0f) || (_mc_cruise_speed > _constraints.speed_xy)) {
		// Use default limit.
		_mc_cruise_speed = _constraints.speed_xy;
	}

	// Temporary target variable where we save the local reprojection of the latest navigator current triplet.
	matrix::Vector3f tmp_target;

	if (!PX4_ISFINITE(_sub_triplet_setpoint->get().current.lat)
	    || !PX4_ISFINITE(_sub_triplet_setpoint->get().current.lon)) {
		// No position provided in xy. Lock position
		if (!PX4_ISFINITE(_lock_position_xy(0))) {
			tmp_target(0) = _lock_position_xy(0) = _position(0);
			tmp_target(1) = _lock_position_xy(1) = _position(1);

		} else {
			tmp_target(0) = _lock_position_xy(0);
			tmp_target(1) = _lock_position_xy(1);
			_lock_position_xy *= NAN;
		}

	} else {
		// Convert from global to local frame.
		map_projection_project(&_reference_position,
				       _sub_triplet_setpoint->get().current.lat, _sub_triplet_setpoint->get().current.lon, &tmp_target(0), &tmp_target(1));
	}

	tmp_target(2) = -(_sub_triplet_setpoint->get().current.alt - _reference_altitude);

	// Check if anything has changed. We do that by comparing the temporary target
	// to the internal _triplet_target.
	// TODO This is a hack and it would be much better if the navigator only sends out a waypoints once they have changed.

	bool triplet_update = true;
	float radius =  MPC_CIRCLE_RAD.get();

	if (!(fabsf(_triplet_target(0) - tmp_target(0)) > 0.001f || fabsf(_triplet_target(1) - tmp_target(1)) > 0.001f
	      || fabsf(_triplet_target(2) - tmp_target(2)) > 0.001f)) {
		// Nothing has changed: just keep old waypoints.
		triplet_update = false;

	} else {
		_triplet_target = tmp_target;

		if (!PX4_ISFINITE(_triplet_target(0)) || !PX4_ISFINITE(_triplet_target(1))) {
			// Horizontal target is not finite.
			_triplet_target(0) = _position(0);
			_triplet_target(1) = _position(1);
		}

		if (!PX4_ISFINITE(_triplet_target(2))) {
			_triplet_target(2) = _position(2);
		}

		// If _triplet_target has updated, update also _triplet_prev_wp and _triplet_next_wp.
		_prev_prev_wp = _triplet_prev_wp;

		if (_isFinite(_sub_triplet_setpoint->get().previous) && _sub_triplet_setpoint->get().previous.valid) {
			map_projection_project(&_reference_position, _sub_triplet_setpoint->get().previous.lat,
					       _sub_triplet_setpoint->get().previous.lon, &_triplet_prev_wp(0), &_triplet_prev_wp(1));
			_triplet_prev_wp(2) = -(_sub_triplet_setpoint->get().previous.alt - _reference_altitude);

		} else {
			_triplet_prev_wp = _position;
		}

		if (_type == WaypointType::loiter) {
			_triplet_next_wp = _triplet_target;

		} else if (_isFinite(_sub_triplet_setpoint->get().next) && _sub_triplet_setpoint->get().next.valid) {
			map_projection_project(&_reference_position, _sub_triplet_setpoint->get().next.lat,
					       _sub_triplet_setpoint->get().next.lon, &_triplet_next_wp(0), &_triplet_next_wp(1));
			_triplet_next_wp(2) = -(_sub_triplet_setpoint->get().next.alt - _reference_altitude);

		} else {
			_triplet_next_wp = _triplet_target;
		}
	}

	if(_type == WaypointType::circle)
	{
		float  angle_speed = MPC_CRUISE_90.get() / radius ;
		if(fabs(_triplet_target(0)-_center_target(0)) > 0.01f || fabs(_triplet_target(1)-_center_target(1)) > 0.01f ){
			_circle_angle_init = false;
			_center_target = _triplet_target;
		}

		if(!_circle_angle_init && sqrtf((_triplet_target(0)-_position(0))*(_triplet_target(0)-_position(0)) + (_triplet_target(1)-_position(1))*(_triplet_target(1)-_position(1))) < (radius+ 1.0f))
		{
			matrix::Vector2f v;
			v = Vector2f(_triplet_target(0), _triplet_target(1)) - Vector2f(_position(0), _position(1));
			if (PX4_ISFINITE(v.length()) && v.length() > SIGMA_NORM) {
				v.normalize();
				_angle =  math::sign(v(1)) * wrap_pi(acosf(v(0)));
			}else
			{
				_angle =_yaw ;
			}
			_circle_angle_init = true ;

		}
		if(_circle_angle_init){
			float angle_change = angle_speed * _deltatime;
			_angle = _angle + angle_change;
			_angle = wrap_2pi(_angle);
			_triplet_target(0) = _center_target(0) - radius * cosf(_angle);
			_triplet_target(1) = _center_target(1) - radius * sinf(_angle);
		}

	}else
	{
		if(_circle_angle_init)
			_circle_angle_init = false;
	}


	// set heading
	if (_type == WaypointType::follow_target && _sub_triplet_setpoint->get().current.yawspeed_valid) {
		_yawspeed_setpoint = _sub_triplet_setpoint->get().current.yawspeed;
		_yaw_setpoint = NAN;

	} else {
		if (_sub_triplet_setpoint->get().current.yaw_valid) {
			_yaw_setpoint = _sub_triplet_setpoint->get().current.yaw;
		} else {
			_set_heading_from_mode();
		}

		_yawspeed_setpoint = NAN;
	}
	// Calculate the current vehicle state and check if it has updated.
	State previous_state = _current_state;
	_current_state = _getCurrentState();

	if (triplet_update || (_current_state != previous_state)) {
		_updateInternalWaypoints();
		_updateAvoidanceWaypoints();
	}

	return true;
}

void FlightTaskAuto::_set_heading_from_mode()
{

	matrix::Vector2f v; // Vector that points towards desired location

	switch (MPC_YAW_MODE.get()) {

	case 0: { // Heading points towards the current waypoint.
			v = Vector2f(_target(0), _target(1)) - Vector2f(_position(0), _position(1));
			if(_type == WaypointType::circle)
			{
				v = Vector2f(_center_target(0), _center_target(1)) - Vector2f(_position(0), _position(1));
			}
			break;
		}

	case 1: { // Heading points towards home.
			if (_sub_home_position->get().valid_hpos) {
				v = Vector2f(_sub_home_position->get().x, _sub_home_position->get().y) - Vector2f(&_position(0));
			}

			break;
		}

	case 2: { // Heading point away from home.
			if (_sub_home_position->get().valid_hpos) {
				v = Vector2f(&_position(0)) - Vector2f(_sub_home_position->get().x, _sub_home_position->get().y);
			}

			break;
		}

	case 3: { // Along trajectory.
			// The heading depends on the kind of setpoint generation. This needs to be implemented
			// in the subclasses where the velocity setpoints are generated.
			v *= NAN;
		}
	}

	if (PX4_ISFINITE(v.length())) {
		// We only adjust yaw if vehicle is outside of acceptance radius. Once we enter acceptance
		// radius, lock yaw to current yaw.
		// This prevents excessive yawing.
		if (v.length() > NAV_ACC_RAD.get()) {
			_compute_heading_from_2D_vector(_yaw_setpoint, v);
			_yaw_lock = false;

		} else {
			if (!_yaw_lock) {
				_yaw_setpoint = _yaw;
				_yaw_lock = true;
			}
		}

	} else {
		_yaw_lock = false;
		_yaw_setpoint = NAN;
	}
}

void FlightTaskAuto::_updateAvoidanceWaypoints()
{
	_desired_waypoint.timestamp = hrt_absolute_time();

	_triplet_target.copyTo(_desired_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_1].position);
	Vector3f(NAN, NAN, NAN).copyTo(_desired_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_1].velocity);
	Vector3f(NAN, NAN, NAN).copyTo(_desired_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_1].acceleration);

	_desired_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_1].yaw = _yaw_setpoint;
	_desired_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_1].yaw_speed = _yawspeed_setpoint;
	_desired_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_1].point_valid = true;


	_triplet_next_wp.copyTo(_desired_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_2].position);
	Vector3f(NAN, NAN, NAN).copyTo(_desired_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_2].velocity);
	Vector3f(NAN, NAN, NAN).copyTo(_desired_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_2].acceleration);

	_desired_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_2].yaw = _sub_triplet_setpoint->get().next.yaw;
	_desired_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_2].yaw_speed =
		_sub_triplet_setpoint->get().next.yawspeed_valid ?
		_sub_triplet_setpoint->get().next.yawspeed : NAN;
	_desired_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_2].point_valid = true;
}

bool FlightTaskAuto::_isFinite(const position_setpoint_s &sp)
{
	return (PX4_ISFINITE(sp.lat) && PX4_ISFINITE(sp.lon) && PX4_ISFINITE(sp.alt));
}

bool FlightTaskAuto::_evaluateGlobalReference()
{
	// check if reference has changed and update.
	// Only update if reference timestamp has changed AND no valid reference altitude
	// is available.
	// TODO: this needs to be revisited and needs a more clear implementation
	if (_sub_vehicle_local_position->get().ref_timestamp == _time_stamp_reference && PX4_ISFINITE(_reference_altitude)) {
		// don't need to update anything
		return true;
	}

	double ref_lat =  _sub_vehicle_local_position->get().ref_lat;
	double ref_lon =  _sub_vehicle_local_position->get().ref_lon;
	_reference_altitude = _sub_vehicle_local_position->get().ref_alt;

	if (!_sub_vehicle_local_position->get().z_global) {
		// we have no valid global altitude
		// set global reference to local reference
		_reference_altitude = 0.0f;
	}

	if (!_sub_vehicle_local_position->get().xy_global) {
		// we have no valid global alt/lat
		// set global reference to local reference
		ref_lat = 0.0;
		ref_lon = 0.0;
	}

	// init projection
	map_projection_init(&_reference_position,
			    ref_lat,
			    ref_lon);

	// check if everything is still finite
	if (PX4_ISFINITE(_reference_altitude)
	    && PX4_ISFINITE(_sub_vehicle_local_position->get().ref_lat)
	    && PX4_ISFINITE(_sub_vehicle_local_position->get().ref_lon)) {
		return true;

	} else {
		// no valid reference
		return false;
	}
}

void FlightTaskAuto::_setDefaultConstraints()
{
	FlightTask::_setDefaultConstraints();

	// only adjust limits if the new limit is lower
	if (_constraints.speed_xy >= MPC_XY_CRUISE.get()) {
		_constraints.speed_xy = MPC_XY_CRUISE.get();
	}
}

matrix::Vector2f FlightTaskAuto::_getTargetVelocityXY()
{
	// guard against any bad velocity values
	const float vx = _sub_triplet_setpoint->get().current.vx;
	const float vy = _sub_triplet_setpoint->get().current.vy;
	bool velocity_valid = PX4_ISFINITE(vx) && PX4_ISFINITE(vy) &&
			      _sub_triplet_setpoint->get().current.velocity_valid;

	if (velocity_valid) {
		return matrix::Vector2f(vx, vy);

	} else {
		// just return zero speed
		return matrix::Vector2f{};
	}
}

State FlightTaskAuto::_getCurrentState()
{
	// Calculate the vehicle current state based on the Navigator triplets and the current position.
	Vector2f u_prev_to_target = Vector2f(&(_triplet_target - _triplet_prev_wp)(0)).unit_or_zero();
	Vector2f pos_to_target = Vector2f(&(_triplet_target - _position)(0));
	Vector2f prev_to_pos = Vector2f(&(_position - _triplet_prev_wp)(0));
	// Calculate the closest point to the vehicle position on the line prev_wp - target
	_closest_pt = Vector2f(&_triplet_prev_wp(0)) + u_prev_to_target * (prev_to_pos * u_prev_to_target);

	State return_state = State::none;

	if (u_prev_to_target * pos_to_target < 0.0f) {
		// Target is behind.
		return_state = State::target_behind;

	} else if (u_prev_to_target * prev_to_pos < 0.0f && prev_to_pos.length() > _mc_cruise_speed) {
		// Current position is more than cruise speed in front of previous setpoint.
		return_state = State::previous_infront;

	} else if (Vector2f(Vector2f(&_position(0)) - _closest_pt).length() > _mc_cruise_speed) {
		// Vehicle is more than cruise speed off track.
		return_state = State::offtrack;

	}
	return return_state;
}

void FlightTaskAuto::_updateInternalWaypoints()
{
	// The internal Waypoints might differ from _triplet_prev_wp, _triplet_target and _triplet_next_wp.
	// The cases where it differs:
	// 1. The vehicle already passed the target -> go straight to target
	// 2. The vehicle is more than cruise speed in front of previous waypoint -> go straight to previous waypoint
	// 3. The vehicle is more than cruise speed from track -> go straight to closest point on track
	//
	// If a new target is available, then the speed at the target is computed from the angle previous-target-next.

	switch (_current_state) {

	case State::target_behind: {
			_target = _triplet_target;
			_prev_wp = _position;
			_next_wp = _triplet_next_wp;
			//_current_state = State::target_behind;

			float angle = 2.0f;
			_speed_at_target = 0.0f;

			// angle = cos(x) + 1.0
			// angle goes from 0 to 2 with 0 = large angle, 2 = small angle:   0 = PI ; 2 = PI*0

			if (Vector2f(&(_target - _next_wp)(0)).length() > 0.001f &&
			    (Vector2f(&(_target - _prev_wp)(0)).length() > NAV_ACC_RAD.get())) {

				angle = Vector2f(&(_target - _prev_wp)(0)).unit_or_zero()
					* Vector2f(&(_target - _next_wp)(0)).unit_or_zero()
					+ 1.0f;
				_speed_at_target = _getVelocityFromAngle(angle);
			}
		}
		break;

	case State::previous_infront: {
			_next_wp = _triplet_target;
			_target = _triplet_prev_wp;
			_prev_wp = _position;

			float angle = 2.0f;
			_speed_at_target = 0.0f;

			// angle = cos(x) + 1.0
			// angle goes from 0 to 2 with 0 = large angle, 2 = small angle:   0 = PI ; 2 = PI*0
			if (Vector2f(&(_target - _next_wp)(0)).length() > 0.001f &&
			    (Vector2f(&(_target - _prev_wp)(0)).length() > NAV_ACC_RAD.get())) {

				angle = Vector2f(&(_target - _prev_wp)(0)).unit_or_zero()
					* Vector2f(&(_target - _next_wp)(0)).unit_or_zero()
					+ 1.0f;
				_speed_at_target = _getVelocityFromAngle(angle);
			}
		}
		break;

	case State::offtrack: {
			_next_wp = _triplet_target;
			_target = matrix::Vector3f(_closest_pt(0), _closest_pt(1), _triplet_target(2));
			_prev_wp = _position;

			float angle = 2.0f;
			_speed_at_target = 0.0f;

			// angle = cos(x) + 1.0
			// angle goes from 0 to 2 with 0 = large angle, 2 = small angle:   0 = PI ; 2 = PI*0
			if (Vector2f(&(_target - _next_wp)(0)).length() > 0.001f &&
			    (Vector2f(&(_target - _prev_wp)(0)).length() > NAV_ACC_RAD.get())) {

				angle = Vector2f(&(_target - _prev_wp)(0)).unit_or_zero()
					* Vector2f(&(_target - _next_wp)(0)).unit_or_zero()
					+ 1.0f;
				_speed_at_target = _getVelocityFromAngle(angle);
			}
		}
		break;

	case State::none: {
			_target = _triplet_target;
			_prev_wp = _triplet_prev_wp;
			_next_wp = _triplet_next_wp;

			float angle = 2.0f;
			_speed_at_target = 0.0f;

			// angle = cos(x) + 1.0
			// angle goes from 0 to 2 with 0 = large angle, 2 = small angle:   0 = PI ; 2 = PI*0
			if (Vector2f(&(_target - _next_wp)(0)).length() > 0.001f &&
			    (Vector2f(&(_target - _prev_wp)(0)).length() > NAV_ACC_RAD.get())) {

				angle =
					Vector2f(&(_target - _prev_wp)(0)).unit_or_zero()
					* Vector2f(&(_target - _next_wp)(0)).unit_or_zero()
					+ 1.0f;
				_speed_at_target = _getVelocityFromAngle(angle);
			}

			break;
		}

	default:
		break;
	}
}

bool FlightTaskAuto::_compute_heading_from_2D_vector(float &heading, matrix::Vector2f v)
{
	if (PX4_ISFINITE(v.length()) && v.length() > SIGMA_NORM) {
		v.normalize();
		// To find yaw: take dot product of x = (1,0) and v
		// and multiply by the sign given of cross product of x and v.
		// Dot product: (x(0)*v(0)+(x(1)*v(1)) = v(0)
		// Cross product: x(0)*v(1) - v(0)*x(1) = v(1)
		heading =  math::sign(v(1)) * wrap_pi(acosf(v(0)));
		return true;
	}

	// heading unknown and therefore do not change heading
	return false;
}


float FlightTaskAuto::_getVelocityFromAngle(const float angle)
{
	// minimum cruise speed when passing waypoint
	float min_cruise_speed = 0.0f;

	// make sure that cruise speed is larger than minimum
	if ((_mc_cruise_speed - min_cruise_speed) < SIGMA_NORM) {
		return _mc_cruise_speed;
	}

	// Middle cruise speed is a number between maximum cruising speed and minimum cruising speed and corresponds to speed at angle of 90degrees.
	// It needs to be always larger than minimum cruise speed.
	float middle_cruise_speed = MPC_CRUISE_90.get();

	if ((middle_cruise_speed - min_cruise_speed) < SIGMA_NORM) {
		middle_cruise_speed = min_cruise_speed + SIGMA_NORM;
	}

	if ((_mc_cruise_speed - middle_cruise_speed) < SIGMA_NORM) {
		middle_cruise_speed = (_mc_cruise_speed + min_cruise_speed) * 0.5f;
	}

	// If middle cruise speed is exactly in the middle, then compute speed linearly.
	bool use_linear_approach = false;

	if (((_mc_cruise_speed + min_cruise_speed) * 0.5f) - middle_cruise_speed < SIGMA_NORM) {
		use_linear_approach = true;
	}

	// compute speed sp at target
	float speed_close;

	if (use_linear_approach) {

		// velocity close to target adjusted to angle:
		// vel_close =  m*x+q
		float slope = -(_mc_cruise_speed - min_cruise_speed) / 2.0f;
		speed_close = slope * angle + _mc_cruise_speed;

	} else {

		// Speed close to target adjusted to angle x.
		// speed_close = a *b ^x + c; where at angle x = 0 -> speed_close = cruise; angle x = 1 -> speed_close = middle_cruise_speed (this means that at 90degrees
		// the velocity at target is middle_cruise_speed);
		// angle x = 2 -> speed_close = min_cruising_speed

		// from maximum cruise speed, minimum cruise speed and middle cruise speed compute constants a, b and c
		float a = -((middle_cruise_speed - _mc_cruise_speed) * (middle_cruise_speed - _mc_cruise_speed))
			  / (2.0f * middle_cruise_speed - _mc_cruise_speed - min_cruise_speed);
		float c = _mc_cruise_speed - a;
		float b = (middle_cruise_speed - c) / a;
		speed_close = a * powf(b, angle) + c;
	}

	// speed_close needs to be in between max and min
	return math::constrain(speed_close, min_cruise_speed, _mc_cruise_speed);
}
