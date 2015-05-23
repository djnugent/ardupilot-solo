/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL.h>
#include <AC_PrecLand_Companion.h>

extern const AP_HAL::HAL& hal;

// Constructor
AC_PrecLand_Companion::AC_PrecLand_Companion(const AC_PrecLand& frontend, AC_PrecLand::precland_state& state)
: AC_PrecLand_Backend(frontend, state)
{
}

// init - perform initialisation of this backend
void AC_PrecLand_Companion::init()
{
    // set healthy
    _state.healthy = true;
    _new_estimate = false;
}

// update - give chance to driver to get updates from sensor
//  returns true if new data available
bool AC_PrecLand_Companion::update()
{
    // Mavlink commands are received asynchronous so all new data is processed by handle_msg()
    return _new_estimate;
}

// get_angle_to_target - returns body frame angles (in radians) to target
//  returns true if angles are available, false if not (i.e. no target)
//  x_angle_rad : body-frame roll direction, positive = target is to right (looking down)
//  y_angle_rad : body-frame pitch direction, postiive = target is forward (looking down)
bool AC_PrecLand_Companion::get_angle_to_target(float &x_angle_rad, float &y_angle_rad) const
{
	if(_new_estimate){
		x_angle_rad = _bf_angle_to_target.x;
		y_angle_rad = _bf_angle_to_target.y;

		//reset and wait for new data
		_new_estimate = false;

		return true;
	}
    return false;
}

uint8_t AC_PrecLand_Companion::handle_msg(mavlink_message_t* msg){

	//TODO: parse mavlink message

	_new_estimate  = true;
	return MAV_RESULT_FAILED;
}