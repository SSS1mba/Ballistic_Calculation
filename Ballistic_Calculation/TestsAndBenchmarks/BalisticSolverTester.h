#pragma once
#include "BalisticSolver.h"
#include "Benchmark.h"
#include <iostream>
#include <vector>
#include <iomanip>

class SimpleBalisticTester {
private:
    struct TestCase {
        std::string name;
        Parametrs input;
    };

    std::vector<TestCase> test_cases;

    void initialize_test_cases();
    Parametrs create_params(double v0, double a0, double alpha, double h, double l,
        double t_t, double t_f, double t_r);
    void run_sequential_tests();
    void run_parallel_tests();

public:
    SimpleBalisticTester();
    void run_all_tests();
};