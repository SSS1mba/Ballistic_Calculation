#include "Parametrs.h"
#include "BalisticSolver.h"
#include <iostream>
#include <fstream>

int main()
{
    Parametrs params;
    params.faling_time = 1;
    params.max_distance = 2;
    params.max_height = 3;
    params.rising_time = 4;
    params.start_acceleration = 5;
    params.start_velocity = 6;
    params.throwing_angle = 7;
    params.T_t = 8;

    BalisticSolver::solve(params,new Parametrs);
   
    
    /* std::ofstream file("params.txt");
    file << params;
    file.close();

    Parametrs params2;
    std::ifstream file_in("params.txt");
    file_in >> params2;
    std::cout << params2.is_initialised(params2.T_f);
    file_in.close();*/
}


