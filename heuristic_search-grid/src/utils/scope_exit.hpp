#pragma once

#include <type_traits>
#include <functional>

namespace utils
{
    // struct scope_exit
    // {
    //     template<typename Op>
    //     scope_exit(Op&& op) : op(std::forward<Op>(op))
    //     {}

    //     ~scope_exit()
    //     {
    //         op();
    //     }

    //     scope_exit(scope_exit const&) = delete;
    //     scope_exit(scope_exit&&) = delete;
    //     scope_exit& operator=(scope_exit const&) = delete;
    //     scope_exit& operator=(scope_exit&&) = delete;

    // private:
    //     std::function<void()> op;
    // };

    template<typename Op>
    struct scope_exit
    {
        scope_exit(Op op) : op(std::move(op))
        {}

        ~scope_exit()
        {
            op();
        }

        scope_exit(scope_exit const&) = delete;
        scope_exit(scope_exit&&) = delete;
        scope_exit& operator=(scope_exit const&) = delete;
        scope_exit& operator=(scope_exit&&) = delete;

    private:
        Op op;
    };
}