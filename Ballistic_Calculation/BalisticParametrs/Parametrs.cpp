#include "Parametrs.h"

Parametrs::Parametrs()
	: start_velocity(UNITIALISED_VARIABLE), start_acceleration(UNITIALISED_VARIABLE)
	, throwing_angle_degrees(UNITIALISED_VARIABLE), max_height(UNITIALISED_VARIABLE)
	, max_distance(UNITIALISED_VARIABLE),	total_time(UNITIALISED_VARIABLE)
	, faling_time(UNITIALISED_VARIABLE),	rising_time(UNITIALISED_VARIABLE)
{
}

Parametrs::Parametrs(const Parametrs& other)
{
    start_velocity = other.start_velocity;
    start_acceleration = other.start_acceleration;
    throwing_angle_degrees = other.throwing_angle_degrees;
    max_height = other.max_height;
    max_distance = other.max_distance;
    total_time = other.total_time;
    faling_time = other.faling_time;
    rising_time = other.rising_time;
}

Parametrs& Parametrs::operator=(const Parametrs& other) {
    if (this != &other) {
        start_velocity = other.start_velocity;
        start_acceleration = other.start_acceleration;
        throwing_angle_degrees = other.throwing_angle_degrees;
        max_height = other.max_height;
        max_distance = other.max_distance;
        total_time = other.total_time;
        faling_time = other.faling_time;
        rising_time = other.rising_time;
    }
    return *this;
}


std::ostream& operator<<(std::ostream& os, const Parametrs& parametrs)
{
    os << std::fixed << std::setprecision(15);
     os << parametrs.start_velocity << "\n"
        << parametrs.start_acceleration << "\n"
        << parametrs.throwing_angle_degrees << "\n"
        << parametrs.max_height << "\n"
        << parametrs.max_distance << "\n"
        << parametrs.total_time << "\n"
        << parametrs.rising_time << "\n"
        << parametrs.faling_time << "\n";
    return os;
}

std::istream& operator>>(std::istream& is, Parametrs& parametrs)
{
     is >> parametrs.start_velocity
        >> parametrs.start_acceleration
        >> parametrs.throwing_angle_degrees
        >> parametrs.max_height
        >> parametrs.max_distance
        >> parametrs.total_time
        >> parametrs.rising_time
        >> parametrs.faling_time;
    return is;
}
