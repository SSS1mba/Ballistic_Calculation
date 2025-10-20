#pragma once
#include "BalisticSolver.h"
#include "Benchmark.h"
#include <iostream>
#include <vector>
#include <iomanip>

class BalisticTester {
public:
    BalisticTester();
    void run_all_tests();

private:
    struct TestCase {
        std::string name;
        Parametrs input;
        Parametrs expected;  
    };

    std::vector<TestCase> test_cases;

    void initialize_test_cases();
    Parametrs create_params(double v0, double a0, double alpha, double h, double l,
        double t_t, double t_f, double t_r);
    void run_sequential_tests();
    void run_parallel_tests();
    void run_parallel_smart_tests();

    bool compare_with_expected(const Parametrs& result, const Parametrs& expected, const std::string& test_name);
    bool is_close(double a, double b, double tolerance = 1e-6);
};