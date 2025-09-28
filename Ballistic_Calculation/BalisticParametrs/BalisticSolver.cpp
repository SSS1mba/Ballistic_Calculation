#include "BalisticSolver.h"

BalisticSolver::BalisticSolver() 
{
	
}

bool BalisticSolver::solve(const Parametrs& input, Parametrs* result) const
{
	if (result == nullptr) throw std::invalid_argument("BalisticSolver -> solve() -> result == nullptr");

	size_t known_parametrs = number_of_initialised_parametrs(input);
	size_t temp;
	
	while (true)
	{
		temp = known_parametrs;

		//все функции для поиска параметров

		if (known_parametrs == NUMBER_OF_PARAMETRS) return true;
		if (known_parametrs == temp) return false;	// if we couldn't find any values during the loop, we're at a dead end  :(
		temp = known_parametrs;
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


double BalisticSolver::find_start_velocity(const Parametrs& p) const noexcept
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

double BalisticSolver::find_start_acceleration(const Parametrs& p) const noexcept
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

double BalisticSolver::find_throwing_angle_degrees(const Parametrs& p) const noexcept

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

double BalisticSolver::find_max_height(const Parametrs& p) const noexcept

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

double BalisticSolver::find_max_distance(const Parametrs& p) const noexcept

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

    // Решение через скорость и время
    if (Parametrs::is_initialised(p.V_0) &&
        Parametrs::is_initialised(p.T_t))
    {
        Parametrs temp = p;
        temp.ALPHA = find_throwing_angle_degrees(p);
        if (Parametrs::is_initialised(temp.ALPHA)) {
            return find_max_distance(temp);
        }
    }

    return UNITIALISED_VARIABLE;
}

double BalisticSolver::find_total_time(const Parametrs& p) const noexcept

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

    // Решение через дальность и скорость
    if (Parametrs::is_initialised(p.L) &&
        Parametrs::is_initialised(p.V_0))
    {
        Parametrs temp = p;
        temp.ALPHA = find_throwing_angle_degrees(p);
        if (Parametrs::is_initialised(temp.ALPHA)) {
            return find_total_time(temp);
        }
    }

    return UNITIALISED_VARIABLE;
}

double BalisticSolver::find_faling_time(const Parametrs& p) const noexcept

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

double BalisticSolver::find_rising_time(const Parametrs& p) const noexcept

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