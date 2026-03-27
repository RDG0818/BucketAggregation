#pragma once

#include <utils/time_op.hpp>

namespace priority_queue_policies
{
    template<typename T>
    class timing_queue final : public T
    {
    public:

        using value_type = T::value_type;
        using difference_type = T::difference_type;
        using location_info = T::location_info;

        template<typename ...Args>
        timing_queue(Args&& ...args) : T(std::forward<Args>(args)...)
        {}

        template<int index = 0>
        [[nodiscard]] inline auto top() -> decltype(auto)
        {
            if constexpr(dual_queue<T>)
            {
                return T::template top<index>();
            }
            else
            {
                return T::top();
            }
        }

        template<int index = 0>
        inline void pop()
        {
            if constexpr(dual_queue<T>)
            {
                utils::time_op(pop_time, [this](){
                    T::template pop<index>();
                });
            }
            else
            {
                utils::time_op(pop_time, [this](){
                    T::pop();
                });
            }
        }

        template<typename U = value_type>
        inline void push(U&& element)
        {
            utils::time_op(push_time, [this](auto&& e){
                T::push(std::forward<U>(e));
            }, std::forward<U>(element));

            max_nodes = std::max(max_nodes, static_cast<difference_type>(T::size()));
            max_buckets = std::max(max_buckets, static_cast<difference_type>(T::count()));
        }

        inline void decrease_key(location_info const& loc_info)
        {
            utils::time_op(decrease_time, [&](){
                T::decrease_key(loc_info);
            });
        }

        inline void erase(location_info const loc_info)
        {
            T::erase(loc_info);
        }

        inline void reorder(difference_type limit)
        {
            ++num_reorders;
            utils::time_op(reorder_time, [=, this](){
                T::reorder(limit);
            });
        }

        [[nodiscard]] inline auto size() const noexcept
        {
            return T::size();
        }

        [[nodiscard]] inline auto is_empty() const noexcept
        {
            return T::is_empty();
        }

        std::chrono::nanoseconds pop_time{}, push_time{}, decrease_time{}, reorder_time{};
        int num_reorders{}, max_nodes{}, max_buckets{};
    };
}