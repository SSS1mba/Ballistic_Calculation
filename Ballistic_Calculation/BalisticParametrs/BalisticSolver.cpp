#include "BalisticSolver.h"


bool BalisticSolver::solve(const Parametrs& input, Parametrs* result) 
{
    if (result == nullptr)
        throw std::invalid_argument("BalisticSolver -> solve() -> result == nullptr");

    *result = input;

    size_t known_parameters = number_of_initialised_parametrs(*result);
    size_t temp_known_parameters;


    std::vector<bool> calculated(NUMBER_OF_PARAMETRS, false);
    auto mark_initialised = [&](const Parametrs& params) {
        if (Parametrs::is_initialised(params.start_velocity))            calculated[index_V_0] = true;
        if (Parametrs::is_initialised(params.start_acceleration))        calculated[index_A_0] = true;
        if (Parametrs::is_initialised(params.throwing_angle_degrees))    calculated[index_ALPHA] = true;
        if (Parametrs::is_initialised(params.max_height))                calculated[index_H] = true;
        if (Parametrs::is_initialised(params.max_distance))              calculated[index_L] = true;
        if (Parametrs::is_initialised(params.total_time))                calculated[index_T_t] = true;
        if (Parametrs::is_initialised(params.faling_time))               calculated[index_T_f] = true;
        if (Parametrs::is_initialised(params.rising_time))               calculated[index_T_r] = true;
        };

        mark_initialised(*result);

    while (true)
    {
        temp_known_parameters = known_parameters;

        for (size_t i = 0; i < NUMBER_OF_PARAMETRS; ++i)
        {
            if (calculated[i]) continue; 

            double value = calculate_parameter(i,*result);

            if (Parametrs::is_initialised(value))
            {
                set_parameter(*result,i, value);
                calculated[i] = true;
                ++known_parameters;
            }
        }

        // ЗАЩИТА ОТ БЕСКОНЕЧНОГО ЦИКЛА
        if (known_parameters == NUMBER_OF_PARAMETRS) return true;
        if (known_parameters == temp_known_parameters) return false; // dead end 
    }

    return true;
}

bool BalisticSolver::solve_parallel(const Parametrs& input, Parametrs* result) 
{
    if (result == nullptr)
        throw std::invalid_argument("BalisticSolver -> solve_parallel() -> result == nullptr");

    *result = input;

    std::atomic<size_t> known_parameters = number_of_initialised_parametrs(*result);
    size_t temp_known_parameters;

    std::vector<bool> calculated(NUMBER_OF_PARAMETRS, false);

    auto mark_initialised = [&](const Parametrs& params) {
        if (Parametrs::is_initialised(params.start_velocity)) calculated[index_V_0] = true;
        if (Parametrs::is_initialised(params.start_acceleration)) calculated[index_A_0] = true;
        if (Parametrs::is_initialised(params.throwing_angle_degrees)) calculated[index_ALPHA] = true;
        if (Parametrs::is_initialised(params.max_height)) calculated[index_H] = true;
        if (Parametrs::is_initialised(params.max_distance)) calculated[index_L] = true;
        if (Parametrs::is_initialised(params.total_time)) calculated[index_T_t] = true;
        if (Parametrs::is_initialised(params.faling_time)) calculated[index_T_f] = true;
        if (Parametrs::is_initialised(params.rising_time)) calculated[index_T_r] = true;
        };


    mark_initialised(*result);
    while (true)
    {
        temp_known_parameters = known_parameters;
        std::vector<std::future<std::pair<size_t, double>>> futures;

        for (size_t i = 0; i < NUMBER_OF_PARAMETRS; ++i)
        {
            if (calculated[i]) continue;

            futures.push_back(std::async(std::launch::async,
                [this, i, input_copy = *result]() mutable -> std::pair<size_t, double> {
                    double value = UNITIALISED_VARIABLE;
                    try { value = calculate_parameter(i, input_copy); }
                    catch (...) {
                        value = UNITIALISED_VARIABLE;
                    }
                    return { i, value };
                }));
        }

        for (auto& future : futures)
        {
            auto [index, value] = future.get();  //structured binding

            if (Parametrs::is_initialised(value))
            {
                set_parameter(*result, index, value);
                calculated[index] = true;
                ++known_parameters;
            }
        }

        if (known_parameters == NUMBER_OF_PARAMETRS) return true;
        if (known_parameters == temp_known_parameters) return false; // dead end
    }

    return true;
}

bool BalisticSolver::solve_parallel_smart(const Parametrs& input, Parametrs* result,size_t treads) 
{
    if (result == nullptr)
        throw std::invalid_argument("BalisticSolver -> solve_parallel_smart() -> result == nullptr");

    *result = input;

    std::atomic<size_t> known_parameters = number_of_initialised_parametrs(*result);
    size_t temp_known_parameters;

    std::vector<bool> calculated(NUMBER_OF_PARAMETRS, false);

    auto mark_initialised = [&](const Parametrs& params) {
        if (Parametrs::is_initialised(params.start_velocity)) calculated[index_V_0] = true;
        if (Parametrs::is_initialised(params.start_acceleration)) calculated[index_A_0] = true;
        if (Parametrs::is_initialised(params.throwing_angle_degrees)) calculated[index_ALPHA] = true;
        if (Parametrs::is_initialised(params.max_height)) calculated[index_H] = true;
        if (Parametrs::is_initialised(params.max_distance)) calculated[index_L] = true;
        if (Parametrs::is_initialised(params.total_time)) calculated[index_T_t] = true;
        if (Parametrs::is_initialised(params.faling_time)) calculated[index_T_f] = true;
        if (Parametrs::is_initialised(params.rising_time)) calculated[index_T_r] = true;
        };


    mark_initialised(*result);
    while (true)
    {
        temp_known_parameters = known_parameters;
        const size_t num_threads = treads;
        std::vector<std::future<std::vector<std::pair<size_t, double>>>> futures;


        for (size_t t = 0; t < num_threads; ++t) {
            futures.push_back(std::async(std::launch::async,
                [this, t, num_threads, &calculated, input_copy = *result]() {
                    std::vector<std::pair<size_t, double>> results;
                    for (size_t i = t; i < NUMBER_OF_PARAMETRS; i += num_threads) {
                        if (calculated[i]) continue;

                        double value = UNITIALISED_VARIABLE;
                        try { value = calculate_parameter(i, input_copy); }
                        catch (...) {
                            value = UNITIALISED_VARIABLE;
                        }
                        if (Parametrs::is_initialised(value)) {
                            results.emplace_back(i, value);
                        }
                    }
                    return results;
                }
            ));
        }

        for (auto& future : futures) {
            auto thread_results = future.get();
            for (const auto& [index, value] : thread_results) {
                set_parameter(*result, index, value);
                calculated[index] = true;
                known_parameters++;
            }
        }

        if (known_parameters == NUMBER_OF_PARAMETRS) return true;
        if (known_parameters == temp_known_parameters) return false;

    }

    return true;
}


///////////////////////////////////////////////////////////////////////
BalisticSolver::BalisticSolver(const Parametrs& parametrs_to_solve) {}

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

double BalisticSolver::calculate_parameter(size_t index, const Parametrs& params) const {
    switch (index) {
    case index_V_0:   return find_start_velocity(params);
    case index_A_0:   return find_start_acceleration(params);
    case index_ALPHA: return find_throwing_angle_degrees(params);
    case index_H:     return find_max_height(params);
    case index_L:     return find_max_distance(params);
    case index_T_t:   return find_total_time(params);
    case index_T_f:   return find_faling_time(params);
    case index_T_r:   return find_rising_time(params);
    default:          return UNITIALISED_VARIABLE;
    }
}

void BalisticSolver::set_parameter(Parametrs& params, size_t index, double value) const {
    switch (index) {
    case index_V_0:   params.start_velocity = value; break;
    case index_A_0:   params.start_acceleration = value; break;
    case index_ALPHA: params.throwing_angle_degrees = value; break;
    case index_H:     params.max_height = value; break;
    case index_L:     params.max_distance = value; break;
    case index_T_t:   params.total_time = value; break;
    case index_T_f:   params.faling_time = value; break;
    case index_T_r:   params.rising_time = value; break;
    }
}



// Основные методы - работают с константной ссылкой
double BalisticSolver::find_start_velocity(const Parametrs& params) const noexcept
{
    if (!Parametrs::is_initialised(params.ALPHA)) return UNITIALISED_VARIABLE;

    // 1. через дальность и время 
    if (Parametrs::is_initialised(params.M_L) && Parametrs::is_initialised(params.T_t)) {
        double theta = deg2rad(params.ALPHA);
        double cos_theta = std::cos(theta);
        if (cos_theta == 0) return UNITIALISED_VARIABLE;

        double v0 = params.M_L / (params.T_t * cos_theta);
        if (v0 > 0 && std::isfinite(v0)) return v0;
    }

    // 2. Через максимальную высоту 
    if (Parametrs::is_initialised(params.M_H) && Parametrs::is_initialised(params.A_0)) {
        double theta = deg2rad(params.ALPHA);
        double sin_theta = std::sin(theta);
        if (sin_theta == 0) return UNITIALISED_VARIABLE;

        double v0 = std::sqrt(2 * std::abs(params.A_0) * params.M_H) / sin_theta;
        if (v0 > 0 && std::isfinite(v0)) return v0;
    }

    // 3. Через дальность полета 
    if (Parametrs::is_initialised(params.M_L) && Parametrs::is_initialised(params.A_0)) {
        double theta = deg2rad(params.ALPHA);
        double sin_2theta = std::sin(2 * theta);
        if (sin_2theta == 0) return UNITIALISED_VARIABLE;

        double v0 = std::sqrt(params.M_L * std::abs(params.A_0) / sin_2theta);
        if (v0 > 0 && std::isfinite(v0)) return v0;
    }

    // 4. Через общее время полета и ускорение
    if (Parametrs::is_initialised(params.T_t) && Parametrs::is_initialised(params.A_0)) {
        double theta = deg2rad(params.ALPHA);
        double sin_theta = std::sin(theta);
        if (sin_theta == 0) return UNITIALISED_VARIABLE;

        double v0 = (params.T_t * std::abs(params.A_0)) / (2 * sin_theta);
        if (v0 > 0 && std::isfinite(v0)) return v0;
    }

    return UNITIALISED_VARIABLE;
}

double BalisticSolver::find_start_acceleration(const Parametrs& params) const noexcept
{
    // 1. Через скорость, угол и высоту
    if (Parametrs::is_initialised(params.V_0) &&
        Parametrs::is_initialised(params.ALPHA) &&
        Parametrs::is_initialised(params.M_H))
    {
        double theta = deg2rad(params.ALPHA);
        double sin_theta = std::sin(theta);
        if (sin_theta == 0) return UNITIALISED_VARIABLE;

        double a0 = (params.V_0 * params.V_0 * sin_theta * sin_theta) / (2 * params.M_H);
        if (a0 > 0 && std::isfinite(a0)) return a0;
    }

    // 2. Через скорость, угол и время
    if (Parametrs::is_initialised(params.V_0) &&
        Parametrs::is_initialised(params.ALPHA) &&
        Parametrs::is_initialised(params.T_t))
    {
        double theta = deg2rad(params.ALPHA);
        double sin_theta = std::sin(theta);
        if (sin_theta == 0) return UNITIALISED_VARIABLE;

        double a0 = (2 * params.V_0 * sin_theta) / params.T_t;
        if (a0 > 0 && std::isfinite(a0)) return a0;
    }

    return UNITIALISED_VARIABLE;
}

double BalisticSolver::find_throwing_angle_degrees(const Parametrs& params) const noexcept
{
    if (!Parametrs::is_initialised(params.V_0) || params.V_0 <= 0)
        return UNITIALISED_VARIABLE;

    // 1. Решение через максимальную высоту 
    if (Parametrs::is_initialised(params.M_H) && Parametrs::is_initialised(params.A_0)) {
        double sin_theta = std::sqrt(2 * std::abs(params.A_0) * params.M_H) / params.V_0;
        if (sin_theta > 1.0 || sin_theta < 0) return UNITIALISED_VARIABLE;

        double angle = rad2deg(std::asin(sin_theta));
        if (angle >= 0 && angle <= 90) return angle;
    }

    // 2. Решение через дальность полета 
    if (Parametrs::is_initialised(params.M_L) && Parametrs::is_initialised(params.A_0)) {
        double sin_2theta = (params.M_L * std::abs(params.A_0)) / (params.V_0 * params.V_0);
        if (sin_2theta > 1.0 || sin_2theta < 0) return UNITIALISED_VARIABLE;

        double angle = rad2deg(std::asin(sin_2theta) / 2.0);
        if (angle >= 0 && angle <= 90) return angle;
    }

    // 3. Решение через общее время полета 
    if (Parametrs::is_initialised(params.T_t) && Parametrs::is_initialised(params.A_0)) {
        double sin_theta = (params.T_t * std::abs(params.A_0)) / (2 * params.V_0);
        if (sin_theta > 1.0 || sin_theta < 0) return UNITIALISED_VARIABLE;

        double angle = rad2deg(std::asin(sin_theta));
        if (angle >= 0 && angle <= 90) return angle;
    }

    // 4. через соотношение высоты и дальности 
    if (Parametrs::is_initialised(params.M_H) && Parametrs::is_initialised(params.M_L)) {
        double tan_theta = (4 * params.M_H) / params.M_L;
        if (tan_theta > 0 && std::isfinite(tan_theta)) {
            double angle = rad2deg(std::atan(tan_theta));
            if (angle >= 0 && angle <= 90) return angle;
        }
    }

    return UNITIALISED_VARIABLE;
}

double BalisticSolver::find_max_height(const Parametrs& params) const noexcept
{
    // 1.Основная формула через скорость и угол (требует ускорение)
    if (Parametrs::is_initialised(params.V_0) &&
        Parametrs::is_initialised(params.ALPHA) &&
        Parametrs::is_initialised(params.A_0))
    {
        double theta = deg2rad(params.ALPHA);
        double h = (params.V_0 * params.V_0 * std::sin(theta) * std::sin(theta)) / (2 * std::abs(params.A_0));
        if (h >= 0 && std::isfinite(h)) return h;
    }

    // 2.Решение через дальность полета и угол 
    if (Parametrs::is_initialised(params.M_L) &&
        Parametrs::is_initialised(params.ALPHA))
    {
        double theta = deg2rad(params.ALPHA);
        double h = (params.M_L * std::tan(theta)) / 4.0;
        if (h >= 0 && std::isfinite(h)) return h;
    }

    // 3.Решение через общее время полета (требует ускорение)
    if (Parametrs::is_initialised(params.T_t) && Parametrs::is_initialised(params.A_0)) {
        double h = (std::abs(params.A_0) * params.T_t * params.T_t) / 8.0;
        if (h >= 0 && std::isfinite(h)) return h;
    }

    return UNITIALISED_VARIABLE;
}

double BalisticSolver::find_max_distance(const Parametrs& params) const noexcept
{
    // 1. через скорость и время 
    if (Parametrs::is_initialised(params.V_0) &&
        Parametrs::is_initialised(params.ALPHA) &&
        Parametrs::is_initialised(params.T_t))
    {
        double theta = deg2rad(params.ALPHA);
        double l = params.V_0 * std::cos(theta) * params.T_t;
        if (l >= 0 && std::isfinite(l)) return l;
    }

    // 2. формула через скорость и угол 
    if (Parametrs::is_initialised(params.V_0) &&
        Parametrs::is_initialised(params.ALPHA) &&
        Parametrs::is_initialised(params.A_0))
    {
        double theta = deg2rad(params.ALPHA);
        double l = (params.V_0 * params.V_0 * std::sin(2 * theta)) / std::abs(params.A_0);
        if (l >= 0 && std::isfinite(l)) return l;
    }

    // 3.Решение через максимальную высоту и угол
    if (Parametrs::is_initialised(params.M_H) &&
        Parametrs::is_initialised(params.ALPHA))
    {
        double theta = deg2rad(params.ALPHA);
        double l = (4 * params.M_H) / std::tan(theta);
        if (l >= 0 && std::isfinite(l)) return l;
    }

    return UNITIALISED_VARIABLE;
}

double BalisticSolver::find_total_time(const Parametrs& params) const noexcept
{
    // 1. Основная формула через скорость и угол
    if (Parametrs::is_initialised(params.V_0) &&
        Parametrs::is_initialised(params.ALPHA) &&
        Parametrs::is_initialised(params.A_0))
    {
        double theta = deg2rad(params.ALPHA);
        double t = (2 * params.V_0 * std::sin(theta)) / std::abs(params.A_0);
        if (t >= 0 && std::isfinite(t)) return t;
    }

    // 2. Через максимальную высоту
    if (Parametrs::is_initialised(params.M_H) && Parametrs::is_initialised(params.A_0)) {
        double t = 2 * std::sqrt(2 * params.M_H / std::abs(params.A_0));
        if (t >= 0 && std::isfinite(t)) return t;
    }

    // 3. Через время подъема или падения
    if (Parametrs::is_initialised(params.T_r)) {
        return 2 * params.T_r;
    }
    if (Parametrs::is_initialised(params.T_f)) {
        return 2 * params.T_f;
    }

    // 4. Через дальность и горизонтальную скорость
    if (Parametrs::is_initialised(params.M_L) &&
        Parametrs::is_initialised(params.V_0) &&
        Parametrs::is_initialised(params.ALPHA))
    {
        double theta = deg2rad(params.ALPHA);
        double cos_theta = std::cos(theta);
        if (cos_theta != 0) {
            double t = params.M_L / (params.V_0 * cos_theta);
            if (t >= 0 && std::isfinite(t)) return t;
        }
    }

    return UNITIALISED_VARIABLE;
}

double BalisticSolver::find_faling_time(const Parametrs& params) const noexcept
{
    // 1. Через время подъема (симметрия)
    if (Parametrs::is_initialised(params.T_r)) {
        return params.T_r;
    }

    // 2. Через общее время
    if (Parametrs::is_initialised(params.T_t)) {
        double tf = params.T_t / 2.0;
        if (tf >= 0 && std::isfinite(tf)) return tf;
    }

    // 3. Через скорость и угол
    if (Parametrs::is_initialised(params.V_0) &&
        Parametrs::is_initialised(params.ALPHA) &&
        Parametrs::is_initialised(params.A_0))
    {
        double theta = deg2rad(params.ALPHA);
        double tf = (params.V_0 * std::sin(theta)) / std::abs(params.A_0);
        if (tf >= 0 && std::isfinite(tf)) return tf;
    }

    // 4. Через максимальную высоту
    if (Parametrs::is_initialised(params.M_H) && Parametrs::is_initialised(params.A_0)) {
        double tf = std::sqrt(2 * params.M_H / std::abs(params.A_0));
        if (tf >= 0 && std::isfinite(tf)) return tf;
    }

    return UNITIALISED_VARIABLE;
}

double BalisticSolver::find_rising_time(const Parametrs& params) const noexcept
{
    // 1. Через время падения (симметрия)
    if (Parametrs::is_initialised(params.T_f)) {
        return params.T_f;
    }

    // 2. Через общее время
    if (Parametrs::is_initialised(params.T_t)) {
        double tr = params.T_t / 2.0;
        if (tr >= 0 && std::isfinite(tr)) return tr;
    }

    // 3. Через скорость и угол
    if (Parametrs::is_initialised(params.V_0) &&
        Parametrs::is_initialised(params.ALPHA) &&
        Parametrs::is_initialised(params.A_0))
    {
        double theta = deg2rad(params.ALPHA);
        double tr = (params.V_0 * std::sin(theta)) / std::abs(params.A_0);
        if (tr >= 0 && std::isfinite(tr)) return tr;
    }

    // 4. Через максимальную высоту
    if (Parametrs::is_initialised(params.M_H) && Parametrs::is_initialised(params.A_0)) {
        double tr = std::sqrt(2 * params.M_H / std::abs(params.A_0));
        if (tr >= 0 && std::isfinite(tr)) return tr;
    }

    return UNITIALISED_VARIABLE;
}
