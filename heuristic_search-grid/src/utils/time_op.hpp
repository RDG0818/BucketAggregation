#pragma once

#include <chrono>

namespace utils
{
    template<typename T1, typename T2, typename Func, typename ...Args>
    [[nodiscard]] inline auto time_op(std::chrono::duration<T1, T2> &duration, Func&& func, Args&&... args) -> std::invoke_result_t<Func, Args...>
    {
        if constexpr(std::is_same_v<std::invoke_result_t<Func, Args...>,void>)
        {
            auto const start = std::chrono::steady_clock::now();
            func(std::forward<Args>(args)...);
            duration += std::chrono::duration_cast<std::chrono::duration<T1, T2>>(std::chrono::steady_clock::now() - start);
        }
        else
        {
            auto const start = std::chrono::steady_clock::now();
            std::invoke_result_t<Func, Args...> val = func(std::forward<Args>(args)...);
            duration += std::chrono::duration_cast<std::chrono::duration<T1, T2>>(std::chrono::steady_clock::now() - start);

            return val;
        }
    }
}