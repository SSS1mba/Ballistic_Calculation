#include "BalisticSolver.h"

bool BalisticSolver::solve(const Parametrs& input, Parametrs* result)
{
	if (result == nullptr) throw std::invalid_argument("BalisticSolver -> solve() -> result == nullptr");

	size_t known_parametrs = number_of_initialised_parametrs(input);
	size_t temp;
	
	while (true)
	{
		temp = known_parametrs;

		//все функции для поиска параметров

		if (known_parametrs == NUMBER_OF_PARAMETRS) return true;
		if (known_parametrs == temp) return false;	// if we couldn't find any values during the loop, we're at a dead end  :(
		temp = known_parametrs;
	}

	return true;
}






size_t BalisticSolver::number_of_initialised_parametrs(const Parametrs& input) noexcept
{
	size_t result = 0;

	if (Parametrs::is_initialised(input.start_velocity)) ++result;
	if (Parametrs::is_initialised(input.start_acceleration)) ++result;
	if (Parametrs::is_initialised(input.throwing_angle)) ++result;
	if (Parametrs::is_initialised(input.max_height)) ++result;
	if (Parametrs::is_initialised(input.max_distance)) ++result;
	if (Parametrs::is_initialised(input.total_time)) ++result;
	if (Parametrs::is_initialised(input.faling_time)) ++result;
	if (Parametrs::is_initialised(input.rising_time)) ++result;

	return result;
}
