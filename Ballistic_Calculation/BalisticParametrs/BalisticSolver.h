#pragma once
#include "Parametrs.h"
#include <vector>
#include <functional>

using BallisticFunction = std::function<double>;


class BalisticSolver
{
public:
	BalisticSolver(const Parametrs& parametrs_to_solve);
	 bool solve(const Parametrs& input, Parametrs* result) const;   //solves "input" and write results of calculations in "results"
																	//throws std::invalid argument 

    size_t number_of_initialised_parametrs(const Parametrs& input) const noexcept;
	

private:
	// a template for functions that find a parameter:
	/*
    * // index of function(in _solve_function) . Parametr to solve
	*  double find_parametr_name ()
	* {
    *   1)the further algorithm implies that the value may already have been found. Check it
	*	2)check variables (they can be == UNITIALISED_VARIABLE)
	*	3)solution...
	*	if (success) return parametr_value;
	*	if (failure) return UNITIALISED_VARIABLE;
	* }
	*/


    // 0. V_0
    double find_start_velocity() const noexcept;

    // 1. A_0
    double find_start_acceleration() const noexcept;

    // 2. ALPHA
    double find_throwing_angle_degrees() const noexcept;

    // 3. H
    double find_max_height() const noexcept;

    // 3. L
    double find_max_distance() const noexcept;

    // 4. T_t
    double find_total_time() const noexcept;

    // 5. T_f
    double find_faling_time() const noexcept;

    // 6. T_r
    double find_rising_time() const noexcept;


private:
    Parametrs p;
    std::vector<std::pair<BallisticFunction, bool>> solve_functions;  //bool - решена ли переменная,за которую отвечает функция
};

