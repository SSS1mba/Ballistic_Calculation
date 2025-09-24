#pragma once
#include "Parametrs.h"


class BalisticSolver
{
public:
	static bool solve(const Parametrs& input, Parametrs* result);  
	//solves "input" and write results of calculations in "results"
	//throws std::invalid argument 

	
private:
	static size_t number_of_initialised_parametrs(const Parametrs& input) noexcept;
};

