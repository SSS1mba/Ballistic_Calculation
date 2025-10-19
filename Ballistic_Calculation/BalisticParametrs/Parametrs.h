#pragma once
#include <cmath>
#include <math.h>
#include <iostream>
#include <numeric>
#include <iomanip>

constexpr double PI = 3.141592653589793;
constexpr double UNITIALISED_VARIABLE = std::numeric_limits<double>::max();
constexpr size_t NUMBER_OF_PARAMETRS = 8;

#define V_0 start_velocity
#define A_0 start_acceleration
#define ALPHA throwing_angle_degrees
#define H max_height
#define L max_distance
#define T_t total_time
#define T_f faling_time
#define T_r rising_time

constexpr size_t index_V_0		 = 0;
constexpr size_t index_A_0		 = 1;
constexpr size_t index_ALPHA	 = 2;
constexpr size_t index_H		 = 3;
constexpr size_t index_L		 = 4;
constexpr size_t index_T_t		 = 5;
constexpr size_t index_T_f		 = 6;
constexpr size_t index_T_r		 = 7;

struct Parametrs
{
	Parametrs();
	double start_velocity;
	double start_acceleration;
	double throwing_angle_degrees;	
	double max_height;
	double max_distance;
	double total_time;
	double faling_time;
	double rising_time;

	inline static bool is_initialised(double value) noexcept { return value != UNITIALISED_VARIABLE; }
	friend std::ostream& operator<<(std::ostream& os, const Parametrs& parametrs);
	friend std::istream& operator>>(std::istream& is, Parametrs& parametrs);
};


inline double deg2rad(double deg)  { return deg * PI / 180.0; }
inline double rad2deg(double rad)  { return rad * 180.0 / PI; }

