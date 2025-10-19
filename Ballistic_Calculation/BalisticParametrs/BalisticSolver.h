#pragma once
#include "Parametrs.h"
#include <vector>
#include <functional>
#include <future>
#include <atomic>



constexpr size_t RECOMENDED_TREADS_FOR_PARALLEL = 2;

class BalisticSolver
{
public:
	BalisticSolver(const Parametrs& parametrs_to_solve);
	 bool solve(const Parametrs& input, Parametrs* result) ;            //solves "input" and write results of calculations in "results"
																	         //throws std::invalid argument 

     bool solve_parallel(const Parametrs& input, Parametrs* result) ;   //multitread version of "solve"

     bool solve_parallel_smart(const Parametrs& input, Parametrs* result,size_t treads = RECOMENDED_TREADS_FOR_PARALLEL) ;

    size_t number_of_initialised_parametrs(const Parametrs& input) const noexcept;
	

private:
    double calculate_parameter(size_t index, const Parametrs& params) const;
    void set_parameter(Parametrs& params, size_t index, double value) const;



    double find_start_velocity(const Parametrs& params) const noexcept;
    double find_start_acceleration(const Parametrs& params) const noexcept;
    double find_throwing_angle_degrees(const Parametrs& params) const noexcept;
    double find_max_height(const Parametrs& params) const noexcept;
    double find_max_distance(const Parametrs& params) const noexcept;
    double find_total_time(const Parametrs& params) const noexcept;
    double find_faling_time(const Parametrs& params) const noexcept;
    double find_rising_time(const Parametrs& params) const noexcept;

};

