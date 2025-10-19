#include "BalisticSolver.h"

BalisticSolver::BalisticSolver(const Parametrs& parametrs_to_solve)
    : p(parametrs_to_solve)
{
   
}

bool BalisticSolver::solve(const Parametrs& input, Parametrs* result) const
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

            double value = UNITIALISED_VARIABLE;

         
            switch (i) {
            case index_V_0:      value = this->find_start_velocity(); break;
            case index_A_0:      value = this->find_start_acceleration(); break;
            case index_ALPHA:    value = this->find_throwing_angle_degrees(); break;
            case index_H:        value = this->find_max_height(); break;
            case index_L:        value = this->find_max_distance(); break;
            case index_T_t:      value = this->find_total_time(); break;
            case index_T_f:      value = this->find_faling_time(); break;
            case index_T_r:      value = this->find_rising_time(); break;
            }

            if (Parametrs::is_initialised(value))
            {
                switch (i) {
                case index_V_0:      result->start_velocity = value; break;
                case index_A_0:      result->start_acceleration = value; break;
                case index_ALPHA:    result->throwing_angle_degrees = value; break;
                case index_H:        result->max_height = value; break;
                case index_L:        result->max_distance = value; break;
                case index_T_t:      result->total_time = value; break;
                case index_T_f:      result->faling_time = value; break;
                case index_T_r:      result->rising_time = value; break;
                }
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

bool BalisticSolver::solve_parallel(const Parametrs& input, Parametrs* result) const
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
                    try {
                        switch (i) {
                        case index_V_0:   value = this->find_start_velocity(); break;
                        case index_A_0:   value = this->find_start_acceleration(); break;
                        case index_ALPHA: value = this->find_throwing_angle_degrees(); break;
                        case index_H:     value = this->find_max_height(); break;
                        case index_L:     value = this->find_max_distance(); break;
                        case index_T_t:   value = this->find_total_time(); break;
                        case index_T_f:   value = this->find_faling_time(); break;
                        case index_T_r:   value = this->find_rising_time(); break;
                        default:          value = UNITIALISED_VARIABLE; break;
                        }
                    }
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
                switch (index) {
                case index_V_0:   result->start_velocity = value; break;
                case index_A_0:   result->start_acceleration = value; break;
                case index_ALPHA: result->throwing_angle_degrees = value; break;
                case index_H:     result->max_height = value; break;
                case index_L:     result->max_distance = value; break;
                case index_T_t:   result->total_time = value; break;
                case index_T_f:   result->faling_time = value; break;
                case index_T_r:   result->rising_time = value; break;
                }
                calculated[index] = true;
                ++known_parameters;
            }
        }

        if (known_parameters == NUMBER_OF_PARAMETRS) return true;
        if (known_parameters == temp_known_parameters) return false; // dead end
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

double BalisticSolver::find_start_velocity() const noexcept
{
    if (!Parametrs::is_initialised(p.ALPHA)) return UNITIALISED_VARIABLE;

    // 1. через дальность и время 
    if (Parametrs::is_initialised(p.M_L) && Parametrs::is_initialised(p.T_t)) {
        double theta = deg2rad(p.ALPHA);
        double cos_theta = std::cos(theta);
        if (cos_theta == 0) return UNITIALISED_VARIABLE;

        // v_x = M_L / T, где v_x = v0 * cos(theta)
        double v0 = p.M_L / (p.T_t * cos_theta);
        if (v0 > 0 && std::isfinite(v0)) return v0;
    }

    // 2. Через максимальную высоту 
    if (Parametrs::is_initialised(p.M_H) && Parametrs::is_initialised(p.A_0)) {
        double theta = deg2rad(p.ALPHA);
        double sin_theta = std::sin(theta);
        if (sin_theta == 0) return UNITIALISED_VARIABLE;

        // V_y² = 2 * a * h, где v_y = v0 * sin(theta)
        double v0 = std::sqrt(2 * std::abs(p.A_0) * p.M_H) / sin_theta;
        if (v0 > 0 && std::isfinite(v0)) return v0;
    }

    // 3. Через дальность полета 
    if (Parametrs::is_initialised(p.M_L) && Parametrs::is_initialised(p.A_0)) {
        double theta = deg2rad(p.ALPHA);
        double sin_2theta = std::sin(2 * theta);
        if (sin_2theta == 0) return UNITIALISED_VARIABLE;

        // M_L = (v0² * sin(2θ)) / |a|  
        double v0 = std::sqrt(p.M_L * std::abs(p.A_0) / sin_2theta);
        if (v0 > 0 && std::isfinite(v0)) return v0;
    }

    // 4. Через общее время полета и ускорение
    if (Parametrs::is_initialised(p.T_t) && Parametrs::is_initialised(p.A_0)) {
        double theta = deg2rad(p.ALPHA);
        double sin_theta = std::sin(theta);
        if (sin_theta == 0) return UNITIALISED_VARIABLE;

        // T_t = 2 * v0 * sin(theta) / |a|
        double v0 = (p.T_t * std::abs(p.A_0)) / (2 * sin_theta);
        if (v0 > 0 && std::isfinite(v0)) return v0;
    }

    return UNITIALISED_VARIABLE;
}

double BalisticSolver::find_start_acceleration() const noexcept
{
    // 1. Через скорость, угол и высоту
    if (Parametrs::is_initialised(p.V_0) &&
        Parametrs::is_initialised(p.ALPHA) &&
        Parametrs::is_initialised(p.M_H))
    {
        double theta = deg2rad(p.ALPHA);
        double sin_theta = std::sin(theta);
        if (sin_theta == 0) return UNITIALISED_VARIABLE;

        // h = (v0² * sin²θ) / (2 * |a|)
        double a0 = (p.V_0 * p.V_0 * sin_theta * sin_theta) / (2 * p.M_H);
        if (a0 > 0 && std::isfinite(a0)) return a0;
    }

    // 2. Через скорость, угол и время
    if (Parametrs::is_initialised(p.V_0) &&
        Parametrs::is_initialised(p.ALPHA) &&
        Parametrs::is_initialised(p.T_t))
    {
        double theta = deg2rad(p.ALPHA);
        double sin_theta = std::sin(theta);
        if (sin_theta == 0) return UNITIALISED_VARIABLE;

        // T_t = 2 * v0 * sin(theta) / |a|
        double a0 = (2 * p.V_0 * sin_theta) / p.T_t;
        if (a0 > 0 && std::isfinite(a0)) return a0;
    }

    return UNITIALISED_VARIABLE;
}

double BalisticSolver::find_throwing_angle_degrees() const noexcept
{

    // 1. Решение через максимальную высоту 
    if (Parametrs::is_initialised(p.M_H) && Parametrs::is_initialised(p.A_0)) {
        // h = (v0² * sin²θ) / (2 * |a|)
        double sin_theta = std::sqrt(2 * std::abs(p.A_0) * p.M_H) / p.V_0;
        if (sin_theta > 1.0 || sin_theta < 0) return UNITIALISED_VARIABLE;

        double angle = rad2deg(std::asin(sin_theta));
        if (angle >= 0 && angle <= 90) return angle;
    }

    // 2. Решение через дальность полета 
    if (Parametrs::is_initialised(p.M_L) && Parametrs::is_initialised(p.A_0)) {
        // M_L = (v0² * sin(2θ)) / |a|
        double sin_2theta = (p.M_L * std::abs(p.A_0)) / (p.V_0 * p.V_0);
        if (sin_2theta > 1.0 || sin_2theta < 0) return UNITIALISED_VARIABLE;

        double angle = rad2deg(std::asin(sin_2theta) / 2.0);
        if (angle >= 0 && angle <= 90) return angle;
    }

    // 3. Решение через общее время полета 
    if (Parametrs::is_initialised(p.T_t) && Parametrs::is_initialised(p.A_0)) {
        // T_t = 2 * v0 * sin(theta) / |a|
        double sin_theta = (p.T_t * std::abs(p.A_0)) / (2 * p.V_0);
        if (sin_theta > 1.0 || sin_theta < 0) return UNITIALISED_VARIABLE;

        double angle = rad2deg(std::asin(sin_theta));
        if (angle >= 0 && angle <= 90) return angle;
    }

    //  4. через соотношение высоты и дальности 
    if (Parametrs::is_initialised(p.M_H) && Parametrs::is_initialised(p.M_L)) {
        // Из соотношения: M_H/L = (tanθ)/4
        double tan_theta = (4 * p.M_H) / p.M_L;
        if (tan_theta > 0 && std::isfinite(tan_theta)) {
            double angle = rad2deg(std::atan(tan_theta));
            if (angle >= 0 && angle <= 90) return angle;
        }
    }

    return UNITIALISED_VARIABLE;
}

double BalisticSolver::find_max_height() const noexcept
{
    // 1.Основная формула через скорость и угол (требует ускорение)
    if (Parametrs::is_initialised(p.V_0) &&
        Parametrs::is_initialised(p.ALPHA) &&
        Parametrs::is_initialised(p.A_0))
    {
        double theta = deg2rad(p.ALPHA);
        // h = (v0² * sin²θ) / (2 * |a|)
        double h = (p.V_0 * p.V_0 * std::sin(theta) * std::sin(theta)) / (2 * std::abs(p.A_0));
        if (h >= 0 && std::isfinite(h)) return h;
    }

    // 2.Решение через дальность полета и угол 
    if (Parametrs::is_initialised(p.M_L) &&
        Parametrs::is_initialised(p.ALPHA))
    {
        double theta = deg2rad(p.ALPHA);
        // h = (M_L * tanθ) / 4
        double h = (p.M_L * std::tan(theta)) / 4.0;
        if (h >= 0 && std::isfinite(h)) return h;
    }

    // 3.Решение через общее время полета (требует ускорение)
    if (Parametrs::is_initialised(p.T_t) && Parametrs::is_initialised(p.A_0)) {
        // h = (a * T_t²) / 8
        double h = (std::abs(p.A_0) * p.T_t * p.T_t) / 8.0;
        if (h >= 0 && std::isfinite(h)) return h;
    }

    return UNITIALISED_VARIABLE;
}

double BalisticSolver::find_max_distance() const noexcept
{
    // 1. через скорость и время 
    if (Parametrs::is_initialised(p.V_0) &&
        Parametrs::is_initialised(p.ALPHA) &&
        Parametrs::is_initialised(p.T_t))
    {
        double theta = deg2rad(p.ALPHA);
        // M_L = v0 * cosθ * T
        double l = p.V_0 * std::cos(theta) * p.T_t;
        if (l >= 0 && std::isfinite(l)) return l;
    }
    // 2. формула через скорость и угол 
    if (Parametrs::is_initialised(p.V_0) &&
        Parametrs::is_initialised(p.ALPHA) &&
        Parametrs::is_initialised(p.A_0))
    {
        double theta = deg2rad(p.ALPHA);
        // M_L = (v0² * sin(2θ)) / |a|
        double l = (p.V_0 * p.V_0 * std::sin(2 * theta)) / std::abs(p.A_0);
        if (l >= 0 && std::isfinite(l)) return l;
    }

    // 3.Решение через максимальную высоту и угол (НЕ требует ускорение!)
    if (Parametrs::is_initialised(p.M_H) &&
        Parametrs::is_initialised(p.ALPHA))
    {
        double theta = deg2rad(p.ALPHA);
        // M_L = (4 * h) / tanθ
        double l = (4 * p.M_H) / std::tan(theta);
        if (l >= 0 && std::isfinite(l)) return l;
    }


    return UNITIALISED_VARIABLE;
}

double BalisticSolver::find_total_time() const noexcept
{
    // 1. Основная формула через скорость и угол
    if (Parametrs::is_initialised(p.V_0) &&
        Parametrs::is_initialised(p.ALPHA) &&
        Parametrs::is_initialised(p.A_0))
    {
        double theta = deg2rad(p.ALPHA);
        // T_t = 2 * v₀ * sinθ / |a|
        double t = (2 * p.V_0 * std::sin(theta)) / std::abs(p.A_0);
        if (t >= 0 && std::isfinite(t)) return t;
    }

    // 2. Через максимальную высоту
    if (Parametrs::is_initialised(p.M_H) && Parametrs::is_initialised(p.A_0)) {
        // T_t = 2 * √(2h / |a|)
        double t = 2 * std::sqrt(2 * p.M_H / std::abs(p.A_0));
        if (t >= 0 && std::isfinite(t)) return t;
    }

    // 3. Через время подъема или падения
    if (Parametrs::is_initialised(p.T_r)) {
        // T_t = 2 * T_r
        return 2 * p.T_r;
    }
    if (Parametrs::is_initialised(p.T_f)) {
        // T_t = 2 * T_f
        return 2 * p.T_f;
    }

    // 4. Через дальность и горизонтальную скорость
    if (Parametrs::is_initialised(p.M_L) &&
        Parametrs::is_initialised(p.V_0) &&
        Parametrs::is_initialised(p.ALPHA))
    {
        double theta = deg2rad(p.ALPHA);
        double cos_theta = std::cos(theta);
        if (cos_theta != 0) {
            // T_t = M_L / (v₀ * cosθ)
            double t = p.M_L / (p.V_0 * cos_theta);
            if (t >= 0 && std::isfinite(t)) return t;
        }
    }

    return UNITIALISED_VARIABLE;
}

double BalisticSolver::find_faling_time() const noexcept
{
    // 1. Через время подъема (симметрия)
    if (Parametrs::is_initialised(p.T_r)) {
        return p.T_r;
    }

    // 2. Через общее время
    if (Parametrs::is_initialised(p.T_t)) {
        // T_f = T_t / 2
        double tf = p.T_t / 2.0;
        if (tf >= 0 && std::isfinite(tf)) return tf;
    }

    // 3. Через скорость и угол
    if (Parametrs::is_initialised(p.V_0) &&
        Parametrs::is_initialised(p.ALPHA) &&
        Parametrs::is_initialised(p.A_0))
    {
        double theta = deg2rad(p.ALPHA);
        // T_f = v₀ * sinθ / |a|
        double tf = (p.V_0 * std::sin(theta)) / std::abs(p.A_0);
        if (tf >= 0 && std::isfinite(tf)) return tf;
    }

    // 4. Через максимальную высоту
    if (Parametrs::is_initialised(p.M_H) && Parametrs::is_initialised(p.A_0)) {
        // T_f = √(2h / |a|)
        double tf = std::sqrt(2 * p.M_H / std::abs(p.A_0));
        if (tf >= 0 && std::isfinite(tf)) return tf;
    }

    return UNITIALISED_VARIABLE;
}

double BalisticSolver::find_rising_time() const noexcept
{
    // 1. Через время падения (симметрия)
    if (Parametrs::is_initialised(p.T_f)) {
        return p.T_f;
    }

    // 2. Через общее время
    if (Parametrs::is_initialised(p.T_t)) {
        // T_r = T_t / 2
        double tr = p.T_t / 2.0;
        if (tr >= 0 && std::isfinite(tr)) return tr;
    }

    // 3. Через скорость и угол
    if (Parametrs::is_initialised(p.V_0) &&
        Parametrs::is_initialised(p.ALPHA) &&
        Parametrs::is_initialised(p.A_0))
    {
        double theta = deg2rad(p.ALPHA);
        // T_r = v₀ * sinθ / |a|
        double tr = (p.V_0 * std::sin(theta)) / std::abs(p.A_0);
        if (tr >= 0 && std::isfinite(tr)) return tr;
    }

    // 4. Через максимальную высоту
    if (Parametrs::is_initialised(p.M_H) && Parametrs::is_initialised(p.A_0)) {
        // T_r = √(2h / |a|)
        double tr = std::sqrt(2 * p.M_H / std::abs(p.A_0));
        if (tr >= 0 && std::isfinite(tr)) return tr;
    }

    return UNITIALISED_VARIABLE;
}