#pragma once

#include <vector>
#include <type_traits>

#include <utils/ordered_set.hpp>
#include <priority_queues/priority_queue_traits.hpp>
#include <priority_queues/policies/binary_heap.hpp>

namespace utils
{
    template<typename T>
        // requires std::is_nothrow_move_constructible_v<T> && std::is_nothrow_move_assignable_v<T>
    class heap_ordered_set
    {
    public:
        using value_type = T;
        using reference = value_type&;
        using const_reference = value_type const&;
        using pointer = value_type*;
        using const_pointer = value_type const*;
        using size_type = unsigned int;
        using difference_type = std::make_signed_t<size_type>;

        using heap_location_info = priority_queue_traits<difference_type, priority_queue_policies::binary_heap>::location_info_type;

        struct bucket_data
        {
            heap_location_info info{};
        };

        struct bucket_compare
        {
            [[nodiscard]] inline bool operator()(difference_type const lhs, difference_type const rhs) const noexcept
            {
                return lhs < rhs;
            }
        };

        struct bucket_info_accessor
        {
            bucket_info_accessor(std::vector<std::pair<value_type, bucket_data>>& buckets) :
                buckets(buckets)
            {}

            [[nodiscard]] inline auto& operator()(difference_type const bucket) noexcept
            {
                return buckets[bucket].second.info;
            }

        private:
            std::vector<std::pair<value_type, bucket_data>>& buckets;
        };

        using heap_type = priority_queue_policies::binary_heap<difference_type, bucket_compare, bucket_info_accessor>;

        heap_ordered_set() :
            container(),
            active_heap(bucket_compare(), bucket_info_accessor(container))
        {}

        heap_ordered_set(heap_ordered_set&&) noexcept = default;
        heap_ordered_set& operator=(heap_ordered_set&&) noexcept = default;
        ~heap_ordered_set() = default;

        heap_ordered_set(heap_ordered_set const&) = delete;
        heap_ordered_set& operator=(heap_ordered_set const&) = delete;

        [[nodiscard]] constexpr auto operator[](difference_type const priority) -> reference
        {
            if(priority >= container.size()) resize(priority);

            if(container[priority].second.info == heap_location_info{})
            {
                active_heap.push(priority);
            }

            return container[priority].first;
        }

        [[nodiscard]] constexpr auto operator[](difference_type const priority) const -> const_reference
        {
            return container[priority].first;
        }

        [[nodiscard]] constexpr auto front() -> reference
        {
            return container[active_heap.top()].first;
        }

        [[nodiscard]] constexpr auto front() const -> const_reference
        {
            return container[active_heap.top()].first;
        }

        constexpr inline void pop_front()
        {
            erase(active_heap.top());
        }

        // [[nodiscard]] inline constexpr auto contains(difference_type const priority) const noexcept -> bool
        // {
        //     return !is_empty()
        //         && priority < container.size()
        //         && non_empty_indices[priority];
        // }

        constexpr void erase(difference_type const priority) //noexcept
        {
            // if(!is_valid_priority(priority) || is_empty()) return;
            active_heap.erase(container[priority].second.info);
        }

        [[nodiscard]] constexpr inline auto size() const noexcept -> size_type
        {
            return container.size();
        }

        [[nodiscard]] constexpr inline auto count() const noexcept -> difference_type
        {
            return active_heap.size();
        }

        [[nodiscard]] constexpr inline auto is_empty() const noexcept -> bool
        {
            return active_heap.is_empty();
        }

    private:
        std::vector<std::pair<value_type, bucket_data>> container;
        heap_type active_heap;

        // [[nodiscard]] constexpr inline auto is_valid_priority(difference_type const priority) const noexcept -> bool
        // {
        //     return (priority < container.size()) && (priority >= begin_index) && non_empty_indices[priority];
        // }

        constexpr inline void resize(difference_type const priority)
        {
            container.resize(std::max(priority + 1, 2 * static_cast<difference_type>(size())));
        }
    };
}