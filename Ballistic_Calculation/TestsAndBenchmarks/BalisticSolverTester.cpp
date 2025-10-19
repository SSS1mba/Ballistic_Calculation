#include "BalisticSolverTester.h"

SimpleBalisticTester::SimpleBalisticTester() {
    initialize_test_cases();
}

void SimpleBalisticTester::run_all_tests() {
    std::cout << "=== TESTING BALISTIC SOLVER ===\n\n";

    // Test sequential version
    std::cout << "SEQUENTIAL VERSION:\n";
    std::cout << "===================\n";
    benchmark([this]() { run_sequential_tests(); });

    // Test parallel version  
    std::cout << "\nPARALLEL VERSION:\n";
    std::cout << "================\n";
    benchmark([this]() { run_parallel_tests(); });

    std::cout << "\n=== TESTING COMPLETED ===\n";
}

void SimpleBalisticTester::initialize_test_cases() {
    test_cases = {
        {
            "Full parameters set",
            create_params(25.0, 9.81, 45.0, 31.86, 63.77, 3.61, 1.80, 1.80)
        },
        {
            "From velocity and angle",
            create_params(25.0, 9.81, 45.0, UNITIALISED_VARIABLE, UNITIALISED_VARIABLE,
                         UNITIALISED_VARIABLE, UNITIALISED_VARIABLE, UNITIALISED_VARIABLE)
        },
        {
            "From height and distance",
            create_params(UNITIALISED_VARIABLE, 9.81, UNITIALISED_VARIABLE, 31.86, 63.77,
                         UNITIALISED_VARIABLE, UNITIALISED_VARIABLE, UNITIALISED_VARIABLE)
        },
        {
            "From time and angle",
            create_params(UNITIALISED_VARIABLE, 9.81, 45.0, UNITIALISED_VARIABLE,
                         UNITIALISED_VARIABLE, 3.61, UNITIALISED_VARIABLE, UNITIALISED_VARIABLE)
        },
        {
            "Not enough data",
            create_params(UNITIALISED_VARIABLE, 9.81, UNITIALISED_VARIABLE, UNITIALISED_VARIABLE,
                         UNITIALISED_VARIABLE, UNITIALISED_VARIABLE, UNITIALISED_VARIABLE, UNITIALISED_VARIABLE)
        },
        {
            "Vertical throw",
            create_params(20.0, 9.81, 90.0, UNITIALISED_VARIABLE, UNITIALISED_VARIABLE,
                         UNITIALISED_VARIABLE, UNITIALISED_VARIABLE, UNITIALISED_VARIABLE)
        },
        {
            "Horizontal throw",
            create_params(15.0, 9.81, 0.0, 10.0, UNITIALISED_VARIABLE,
                         UNITIALISED_VARIABLE, UNITIALISED_VARIABLE, UNITIALISED_VARIABLE)
        },
        {
            "Moon acceleration",
            create_params(10.0, 1.62, 30.0, UNITIALISED_VARIABLE, UNITIALISED_VARIABLE,
                         UNITIALISED_VARIABLE, UNITIALISED_VARIABLE, UNITIALISED_VARIABLE)
        }
    };
}

Parametrs SimpleBalisticTester::create_params(double v0, double a0, double alpha, double h, double l,
    double t_t, double t_f, double t_r) {
    Parametrs p;
    p.start_velocity = v0;
    p.start_acceleration = a0;
    p.throwing_angle_degrees = alpha;
    p.max_height = h;
    p.max_distance = l;
    p.total_time = t_t;
    p.faling_time = t_f;
    p.rising_time = t_r;
    return p;
}

void SimpleBalisticTester::run_sequential_tests() {
    for (const auto& test_case : test_cases) {
        std::cout << "Test: " << std::setw(25) << std::left << test_case.name;

        BalisticSolver solver(test_case.input);
        Parametrs result;

        bool success = solver.solve(test_case.input, &result);

        std::cout << (success ? " SUCCESS" : " FAILED") << "\n";
    }
}

void SimpleBalisticTester::run_parallel_tests() {
    for (const auto& test_case : test_cases) {
        std::cout << "Test: " << std::setw(25) << std::left << test_case.name;

        BalisticSolver solver(test_case.input);
        Parametrs result;

        bool success = solver.solve_parallel(test_case.input, &result);

        std::cout << (success ? " SUCCESS" : " FAILED") << "\n";
    }
}