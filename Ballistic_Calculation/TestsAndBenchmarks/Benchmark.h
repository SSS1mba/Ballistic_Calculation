#pragma once
#include <ctime>
#include <chrono>
#include <iostream>

using namespace std::chrono;
using Clock = high_resolution_clock;

template <typename Dur>
void prettyPrintDuration(Dur dur)
{
    auto h = duration_cast<hours>(dur);
    if (h.count()) { std::cout << h << " "; dur -= h; }

    auto m = duration_cast<minutes>(dur);
    if (m.count()) { std::cout << m << " "; dur -= m; }

    auto s = duration_cast<seconds>(dur);
    if (s.count()) { std::cout << s << " "; dur -= s; }

    auto ms = duration_cast<milliseconds>(dur);
    if (ms.count()) { std::cout << ms << " "; dur -= ms; }

    auto us = duration_cast<microseconds>(dur);
    if (us.count()) { std::cout << us << " "; dur -= us; }

    auto ns = duration_cast<nanoseconds>(dur);
    if (ns.count()) { std::cout << ns << " "; dur -= ns; }

    std::cout << std::endl;
}

template<typename Func, typename... Args>
auto benchmark(Func&& f, Args... args) -> decltype(f(std::forward<Args>(args)...))
{
    auto start = Clock::now();

    auto summarize = [&start]() {
        auto dt = Clock::now() - start;
        prettyPrintDuration(dt);
        };

    if constexpr (std::is_same_v<decltype(f(std::forward<Args>(args)...)), void>) {
        f(std::forward<Args>(args)...);
        summarize();
    }
    else {
        auto res = f(std::forward<Args>(args)...);
        summarize();
        return res;
    }
}

