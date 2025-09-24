#include "Parametrs.h"

Parametrs::Parametrs()
	: start_velocity(UNITIALISED_VARIABLE), start_acceleration(UNITIALISED_VARIABLE)
	, throwing_angle(UNITIALISED_VARIABLE), max_height(UNITIALISED_VARIABLE)
	, max_distance(UNITIALISED_VARIABLE),	total_time(UNITIALISED_VARIABLE)
	, faling_time(UNITIALISED_VARIABLE),	rising_time(UNITIALISED_VARIABLE)
{
}


std::ostream& operator<<(std::ostream& os, const Parametrs& parametrs)
{
    os << std::fixed << std::setprecision(6);
     os << parametrs.start_velocity << " "
        << parametrs.start_acceleration << " "
        << parametrs.throwing_angle << " "
        << parametrs.max_height << " "
        << parametrs.max_distance << " "
        << parametrs.total_time << " "
        << parametrs.rising_time << " "
        << parametrs.faling_time;
    return os;
}

std::istream& operator>>(std::istream& is, Parametrs& parametrs)
{
     is >> parametrs.start_velocity
        >> parametrs.start_acceleration
        >> parametrs.throwing_angle
        >> parametrs.max_height
        >> parametrs.max_distance
        >> parametrs.total_time
        >> parametrs.rising_time
        >> parametrs.faling_time;
    return is;
}
