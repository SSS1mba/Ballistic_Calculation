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
    params.rising_time = 5;
    params.start_acceleration = 6;
    params.start_velocity = 7;
    params.throwing_angle = 8;
    params.T_t = 9;

    std::ofstream file("params.txt");
    file << params;
    file.close();

    Parametrs params2;
    std::ifstream file_in("params.txt");
    file_in >> params2;
    file_in.close();
}


