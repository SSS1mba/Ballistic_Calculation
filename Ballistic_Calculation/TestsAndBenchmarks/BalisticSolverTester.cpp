#include "BalisticSolverTester.h"

BalisticTester::BalisticTester() {
    initialize_test_cases();
}

void BalisticTester::run_all_tests() {
    std::cout << "=== TESTING BALISTIC SOLVER ===\n\n";

    std::cout << "SEQUENTIAL VERSION:\n";
    std::cout << "===================\n";
     run_sequential_tests();;

    std::cout << "\nPARALLEL VERSION:\n";
    std::cout << "================\n";
    run_parallel_tests();

    std::cout << "\nPARALLEL SMART VERSION:\n";
    std::cout << "================\n";
    run_parallel_smart_tests();

    std::cout << "\n=== TESTING COMPLETED ===\n";
}

void BalisticTester::initialize_test_cases() {
    test_cases = {
        {
            "Full parameters set",
            create_params(25.0, 9.81, 45.0, 31.86, 63.77, 3.61, 1.80, 1.80),
            create_params(25.0, 9.81, 45.0, 31.86, 63.77, 3.61, 1.80, 1.80)
        },
        {
            "From velocity and angle",
            create_params(25.0, 9.81, 45.0, UNITIALISED_VARIABLE, UNITIALISED_VARIABLE,
                         UNITIALISED_VARIABLE, UNITIALISED_VARIABLE, UNITIALISED_VARIABLE),
            create_params(25.0, 9.81, 45.0, 15.94, 63.77, 3.61, 1.80, 1.80)
        },
        {
            "From height and distance",
            create_params(UNITIALISED_VARIABLE, 9.81, UNITIALISED_VARIABLE, 31.86, 63.77,
                         UNITIALISED_VARIABLE, UNITIALISED_VARIABLE, UNITIALISED_VARIABLE),
            create_params(25.0, 9.81, 45.0, 31.86, 63.77, 3.61, 1.80, 1.80)
        },
        {
            "From time and angle",
            create_params(UNITIALISED_VARIABLE, 9.81, 45.0, UNITIALISED_VARIABLE,
                         UNITIALISED_VARIABLE, 3.61, UNITIALISED_VARIABLE, UNITIALISED_VARIABLE),
            create_params(25.0, 9.81, 45.0, 15.94, 63.77, 3.61, 1.80, 1.80)
        },
        {
            "Not enough data",
            create_params(UNITIALISED_VARIABLE, 9.81, UNITIALISED_VARIABLE, UNITIALISED_VARIABLE,
                         UNITIALISED_VARIABLE, UNITIALISED_VARIABLE, UNITIALISED_VARIABLE, UNITIALISED_VARIABLE),
            create_params(UNITIALISED_VARIABLE, 9.81, UNITIALISED_VARIABLE, UNITIALISED_VARIABLE,
                         UNITIALISED_VARIABLE, UNITIALISED_VARIABLE, UNITIALISED_VARIABLE, UNITIALISED_VARIABLE)
        },
        {
            "Vertical throw",
            create_params(20.0, 9.81, 90.0, UNITIALISED_VARIABLE, UNITIALISED_VARIABLE,
                         UNITIALISED_VARIABLE, UNITIALISED_VARIABLE, UNITIALISED_VARIABLE),
            create_params(20.0, 9.81, 90.0, 20.39, 0.0, 4.08, 2.04, 2.04)
        },
        {
            "Horizontal throw",
            create_params(15.0, UNITIALISED_VARIABLE, 0.0, UNITIALISED_VARIABLE, 21.43,
                         UNITIALISED_VARIABLE, UNITIALISED_VARIABLE, UNITIALISED_VARIABLE),
            create_params(15.0, 0, 0, 0, 21.43, 1.43, 1.43, 1.43)
        },
        {
            "Moon acceleration",
            create_params(10.0, 1.62, 30.0, UNITIALISED_VARIABLE, UNITIALISED_VARIABLE,
                         UNITIALISED_VARIABLE, UNITIALISED_VARIABLE, UNITIALISED_VARIABLE),
            create_params(10.0, 1.62, 30.0, 7.72, 53.73, 6.17, 3.09, 3.09)
        }
    };
}

Parametrs BalisticTester::create_params(double v0, double a0, double alpha, double h, double l,
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

void BalisticTester::run_sequential_tests() {
    auto total_duration = std::chrono::steady_clock::duration::zero();
    for (const auto& test_case : test_cases) {
        std::cout << "Test: " << std::setw(25) << std::left << test_case.name;

        BalisticSolver solver;
        Parametrs result;

        auto start = Clock::now();
        bool success = solver.solve(test_case.input, &result);
        total_duration += Clock::now() - start;
        if (success) {
            bool matches_expected = compare_with_expected(result, test_case.expected);
            std::cout << (matches_expected ? " SUCCESS" : " FAILED - wrong values");
        }
        else {
            std::cout << " FAILED - no solution";
        }
        std::cout << "\n";
    }
    prettyPrintDuration(total_duration);
}

void BalisticTester::run_parallel_tests() {
    auto total_duration = std::chrono::steady_clock::duration::zero();
    for (const auto& test_case : test_cases) {
        std::cout << "Test: " << std::setw(25) << std::left << test_case.name;

        BalisticSolver solver;
        Parametrs result;

        auto start = Clock::now();
        bool success = solver.solve_parallel(test_case.input, &result);
        total_duration += Clock::now() - start;
        if (success) {
            bool matches_expected = compare_with_expected(result, test_case.expected);
            std::cout << (matches_expected ? " SUCCESS" : " FAILED - wrong values");
        }
        else {
            std::cout << " FAILED - no solution";
        }
        std::cout << "\n";
    }
    prettyPrintDuration(total_duration);
}

void BalisticTester::run_parallel_smart_tests() {
    auto total_duration = std::chrono::steady_clock::duration::zero();
    for (const auto& test_case : test_cases) {
        std::cout << "Test: " << std::setw(25) << std::left << test_case.name;

        BalisticSolver solver;
        Parametrs result;

        auto start = Clock::now();
        bool success = solver.solve_parallel_smart(test_case.input, &result);
        total_duration += Clock::now() - start;

        if (success) {
            bool matches_expected = compare_with_expected(result, test_case.expected);
            std::cout << (matches_expected ? " SUCCESS" : " FAILED - wrong values");
        }
        else {
            std::cout << " FAILED - no solution";
        }
        std::cout << "\n";
    }
    prettyPrintDuration(total_duration);
}

bool BalisticTester::compare_with_expected(const Parametrs& result, const Parametrs& expected) {
    const double tolerance = 1e-2; // 1% tolerance 

    auto check_param = [&](double result_val, double expected_val) -> bool {
        if (Parametrs::is_initialised(expected_val)) {
            if (!Parametrs::is_initialised(result_val)) {
                return false; 
            }
            if (!is_close(result_val, expected_val, tolerance)) {
                return false; 
            }
        }
        return true;
        };

    return check_param(result.start_velocity, expected.start_velocity)              &&
        check_param(result.start_acceleration, expected.start_acceleration)         &&
        check_param(result.throwing_angle_degrees, expected.throwing_angle_degrees) &&
        check_param(result.max_height, expected.max_height )                        &&
        check_param(result.max_distance, expected.max_distance )                    &&
        check_param(result.total_time, expected.total_time)                         &&
        check_param(result.faling_time, expected.faling_time)                       &&
        check_param(result.rising_time, expected.rising_time);
}

bool BalisticTester::is_close(double a, double b, double tolerance) {
    if (a == b) return true;
    if (std::abs(a - b) < 1e-10) return true; 

    double relative_error = std::abs(a - b) / std::max(std::abs(a), std::abs(b));
    return relative_error <= tolerance;
}