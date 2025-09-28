#include "BalisticSolver.h"

BalisticSolver::BalisticSolver(const Parametrs& parametrs_to_solve)
    : p(parametrs_to_solve)
{
    bool temp;

    // 0. V_0
    temp = parametrs_to_solve.is_initialised(parametrs_to_solve.start_velocity);
    solve_functions.emplace_back(std::pair(find_start_velocity, temp));

    // 1. A_0
    temp = parametrs_to_solve.is_initialised(parametrs_to_solve.start_acceleration);
    solve_functions.emplace_back(std::pair(find_start_acceleration, temp));

    // 2. ALPHA
    temp = parametrs_to_solve.is_initialised(parametrs_to_solve.throwing_angle_degrees);
    solve_functions.emplace_back(std::pair(find_throwing_angle_degrees, temp));

    // 3. H
    temp = parametrs_to_solve.is_initialised(parametrs_to_solve.max_height);
    solve_functions.emplace_back(std::pair(find_max_height, temp));

    // 4. L
    temp = parametrs_to_solve.is_initialised(parametrs_to_solve.max_distance);
    solve_functions.emplace_back(std::pair(find_max_distance, temp));

    // 5. T_t
    temp = parametrs_to_solve.is_initialised(parametrs_to_solve.total_time);
    solve_functions.emplace_back(std::pair(find_total_time, temp));

    // 6. T_f
    temp = parametrs_to_solve.is_initialised(parametrs_to_solve.faling_time);
    solve_functions.emplace_back(std::pair(find_faling_time, temp));

    // 7. T_r
    temp = parametrs_to_solve.is_initialised(parametrs_to_solve.rising_time);
    solve_functions.emplace_back(std::pair(find_rising_time, temp));
}

bool BalisticSolver::solve(const Parametrs& input, Parametrs* result) const
{
	if (result == nullptr) throw std::invalid_argument("BalisticSolver -> solve() -> result == nullptr");

	size_t known_parametrs = number_of_initialised_parametrs(input);
	size_t temp_known_parametrs;
    double value_of_parametr;;
	
	while (true)
	{
        temp_known_parametrs = known_parametrs;

        for (auto& el : solve_functions)
        {
            if (el.second) continue;
            auto a = el.first;
            a();
        }

		if (known_parametrs == NUMBER_OF_PARAMETRS) return true;
		if (known_parametrs == temp_known_parametrs) return false;	// if we couldn't find any values during the loop, we're at a dead end  :(
        temp_known_parametrs = known_parametrs;
	}

	return true;
}

size_t BalisticSolver::number_of_initialised_parametrs(const Parametrs& input)const noexcept 
{
	size_t result = 0;

	if (Parametrs::is_initialised(input.start_velocity)) ++result;
	if (Parametrs::is_initialised(input.start_acceleration)) ++result;
	if (Parametrs::is_initialised(input.throwing_angle_degrees)) ++result;
	if (Parametrs::is_initialised(input.max_height)) ++result;
	if (Parametrs::is_initialised(input.max_distance)) ++result;
	if (Parametrs::is_initialised(input.total_time)) ++result;
	if (Parametrs::is_initialised(input.faling_time)) ++result;
	if (Parametrs::is_initialised(input.rising_time)) ++result;

	return result;
}



///////////////////////////////////////////////////////////////////////

double BalisticSolver::find_start_velocity()         const noexcept
{

    if (!Parametrs::is_initialised(p.ALPHA)) return UNITIALISED_VARIABLE;
    if (!Parametrs::is_initialised(p.A_0)) return UNITIALISED_VARIABLE;

    if (Parametrs::is_initialised(p.H)) {
        double theta = deg2rad(p.ALPHA);
        double sin_theta = std::sin(theta);
        if (sin_theta == 0) return UNITIALISED_VARIABLE;

        double v0 = std::sqrt(2 * p.A_0 * p.H) / sin_theta;
        if (v0 > 0 && std::isfinite(v0)) return v0;
    }

    // Решение через дальность полета
    if (Parametrs::is_initialised(p.L)) {
        double theta = deg2rad(p.ALPHA);
        double sin_2theta = std::sin(2 * theta);
        if (sin_2theta == 0) return UNITIALISED_VARIABLE;

        double v0 = std::sqrt(p.L * p.A_0 / sin_2theta);
        if (v0 > 0 && std::isfinite(v0)) return v0;
    }

    // Решение через общее время полета
    if (Parametrs::is_initialised(p.T_t)) {
        double theta = deg2rad(p.ALPHA);
        double sin_theta = std::sin(theta);
        if (sin_theta == 0) return UNITIALISED_VARIABLE;

        double v0 = (p.T_t * p.A_0) / (2 * sin_theta);
        if (v0 > 0 && std::isfinite(v0)) return v0;
    }

    return UNITIALISED_VARIABLE;
}

double BalisticSolver::find_start_acceleration()     const noexcept
{

    // Через скорость, угол и высоту
    if (Parametrs::is_initialised(p.V_0) &&
        Parametrs::is_initialised(p.ALPHA) &&
        Parametrs::is_initialised(p.H))
    {
        double theta = deg2rad(p.ALPHA);
        double sin_theta = std::sin(theta);
        if (sin_theta == 0) return UNITIALISED_VARIABLE;

        double a0 = (p.V_0 * p.V_0 * sin_theta * sin_theta) / (2 * p.H);
        if (a0 > 0 && std::isfinite(a0)) return a0;
    }

    // Через скорость, угол и дальность
    if (Parametrs::is_initialised(p.V_0) &&
        Parametrs::is_initialised(p.ALPHA) &&
        Parametrs::is_initialised(p.L))
    {
        double theta = deg2rad(p.ALPHA);
        double sin_2theta = std::sin(2 * theta);
        if (sin_2theta == 0) return UNITIALISED_VARIABLE;

        double a0 = (p.V_0 * p.V_0 * sin_2theta) / p.L;
        if (a0 > 0 && std::isfinite(a0)) return a0;
    }

    // Через скорость, угол и время
    if (Parametrs::is_initialised(p.V_0) &&
        Parametrs::is_initialised(p.ALPHA) &&
        Parametrs::is_initialised(p.T_t))
    {
        double theta = deg2rad(p.ALPHA);
        double sin_theta = std::sin(theta);
        if (sin_theta == 0) return UNITIALISED_VARIABLE;

        double a0 = (2 * p.V_0 * sin_theta) / p.T_t;
        if (a0 > 0 && std::isfinite(a0)) return a0;
    }

    return UNITIALISED_VARIABLE;
}

double BalisticSolver::find_throwing_angle_degrees() const noexcept

{
    if (!Parametrs::is_initialised(p.V_0) || p.V_0 <= 0)
        return UNITIALISED_VARIABLE;
    if (!Parametrs::is_initialised(p.A_0))
        return UNITIALISED_VARIABLE;

    // Решение через максимальную высоту
    if (Parametrs::is_initialised(p.H)) {
        double sin_theta = std::sqrt(2 * p.A_0 * p.H) / p.V_0;
        if (sin_theta > 1.0 || sin_theta < 0) return UNITIALISED_VARIABLE;

        double angle = rad2deg(std::asin(sin_theta));
        if (angle >= 0 && angle <= 90) return angle;
    }

    // Решение через дальность полета
    if (Parametrs::is_initialised(p.L)) {
        double sin_2theta = (p.L * p.A_0) / (p.V_0 * p.V_0);
        if (sin_2theta > 1.0 || sin_2theta < 0) return UNITIALISED_VARIABLE;

        double angle = rad2deg(std::asin(sin_2theta) / 2.0);
        if (angle >= 0 && angle <= 90) return angle;
    }

    // Решение через общее время полета
    if (Parametrs::is_initialised(p.T_t)) {
        double sin_theta = (p.T_t * p.A_0) / (2 * p.V_0);
        if (sin_theta > 1.0 || sin_theta < 0) return UNITIALISED_VARIABLE;

        double angle = rad2deg(std::asin(sin_theta));
        if (angle >= 0 && angle <= 90) return angle;
    }

    return UNITIALISED_VARIABLE;
}

double BalisticSolver::find_max_height()             const noexcept

{
    if (!Parametrs::is_initialised(p.A_0)) return UNITIALISED_VARIABLE;

    // Основная формула через скорость и угол
    if (Parametrs::is_initialised(p.V_0) &&
        Parametrs::is_initialised(p.ALPHA))
    {
        double theta = deg2rad(p.ALPHA);
        double h = (p.V_0 * p.V_0 * std::sin(theta) * std::sin(theta)) / (2 * p.A_0);
        if (h >= 0 && std::isfinite(h)) return h;
    }

    // Решение через дальность полета и угол
    if (Parametrs::is_initialised(p.L) &&
        Parametrs::is_initialised(p.ALPHA))
    {
        double theta = deg2rad(p.ALPHA);
        double h = (p.L * std::tan(theta)) / 4.0;
        if (h >= 0 && std::isfinite(h)) return h;
    }

    // Решение через общее время полета
    if (Parametrs::is_initialised(p.T_t)) {
        double h = (p.A_0 * p.T_t * p.T_t) / 8.0;
        if (h >= 0 && std::isfinite(h)) return h;
    }

    return UNITIALISED_VARIABLE;
}

double BalisticSolver::find_max_distance()           const noexcept

{
    if (!Parametrs::is_initialised(p.A_0)) return UNITIALISED_VARIABLE;


    // Основная формула через скорость и угол
    if (Parametrs::is_initialised(p.V_0) &&
        Parametrs::is_initialised(p.ALPHA))
    {
        double theta = deg2rad(p.ALPHA);
        double l = (p.V_0 * p.V_0 * std::sin(2 * theta)) / p.A_0;
        if (l >= 0 && std::isfinite(l)) return l;
    }

    // Решение через максимальную высоту и угол
    if (Parametrs::is_initialised(p.H) &&
        Parametrs::is_initialised(p.ALPHA))
    {
        double theta = deg2rad(p.ALPHA);
        double l = (4 * p.H) / std::tan(theta);
        if (l >= 0 && std::isfinite(l)) return l;
    }

    return UNITIALISED_VARIABLE;
}

double BalisticSolver::find_total_time()             const noexcept

{
    if (!Parametrs::is_initialised(p.A_0)) return UNITIALISED_VARIABLE;

   
    // Основная формула через скорость и угол
    if (Parametrs::is_initialised(p.V_0) &&
        Parametrs::is_initialised(p.ALPHA))
    {
        double theta = deg2rad(p.ALPHA);
        double t = (2 * p.V_0 * std::sin(theta)) / p.A_0;
        if (t >= 0 && std::isfinite(t)) return t;
    }

    // Решение через максимальную высоту
    if (Parametrs::is_initialised(p.H)) {
        double t = 2 * std::sqrt(2 * p.H / p.A_0);
        if (t >= 0 && std::isfinite(t)) return t;
    }


    return UNITIALISED_VARIABLE;
}

double BalisticSolver::find_faling_time()            const noexcept

{
    if (!Parametrs::is_initialised(p.A_0)) return UNITIALISED_VARIABLE;

    // Время падения = время подъема (в вакууме)
    if (Parametrs::is_initialised(p.T_r)) {
        return p.T_r;
    }

    // Через общее время
    if (Parametrs::is_initialised(p.T_t)) {
        double tf = p.T_t / 2.0;
        if (tf >= 0 && std::isfinite(tf)) return tf;
    }

    // Через скорость и угол
    if (Parametrs::is_initialised(p.V_0) &&
        Parametrs::is_initialised(p.ALPHA))
    {
        double theta = deg2rad(p.ALPHA);
        double tf = (p.V_0 * std::sin(theta)) / p.A_0;
        if (tf >= 0 && std::isfinite(tf)) return tf;
    }

    // Через максимальную высоту
    if (Parametrs::is_initialised(p.H)) {
        double tf = std::sqrt(2 * p.H / p.A_0);
        if (tf >= 0 && std::isfinite(tf)) return tf;
    }

    return UNITIALISED_VARIABLE;
}

double BalisticSolver::find_rising_time()            const noexcept

{
    if (!Parametrs::is_initialised(p.A_0)) return UNITIALISED_VARIABLE;


    // Время подъема = половина общего времени
    if (Parametrs::is_initialised(p.T_t)) {
        double tr = p.T_t / 2.0;
        if (tr >= 0 && std::isfinite(tr)) return tr;
    }

    // Через скорость и угол
    if (Parametrs::is_initialised(p.V_0) &&
        Parametrs::is_initialised(p.ALPHA))
    {
        double theta = deg2rad(p.ALPHA);
        double tr = (p.V_0 * std::sin(theta)) / p.A_0;
        if (tr >= 0 && std::isfinite(tr)) return tr;
    }

    // Через максимальную высоту
    if (Parametrs::is_initialised(p.H)) {
        double tr = std::sqrt(2 * p.H / p.A_0);
        if (tr >= 0 && std::isfinite(tr)) return tr;
    }

    return UNITIALISED_VARIABLE;
}