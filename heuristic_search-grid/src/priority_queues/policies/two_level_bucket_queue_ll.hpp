#pragma once

#include <vector>
#include <list>
#include <iterator>
#include <limits>
#include <memory>

#include <priority_queues/priority_queue_traits.hpp>
#include <utils/ordered_set.hpp>
#include <utils/geometric_ordered_set.hpp>

namespace priority_queue_policies
{
    template<typename T, typename PriorityPair, typename ElementInfoAccessor>
    class two_level_bucket_queue_ll
    {
    public:
        using value_type = T;
        using reference = value_type&;
        using const_reference = value_type const&;
        using pointer = value_type*;
        using const_pointer = value_type const*;
        using size_type = unsigned int;
        using difference_type = std::make_signed_t<size_type>;

        using location_info = priority_queue_traits<value_type, two_level_bucket_queue_ll>::location_info_type;

        using secondary_bucket_type = std::list<value_type>;
        // using primary_bucket_type = utils::ordered_set<secondary_bucket_type>;
        // using queue_type = utils::ordered_set<primary_bucket_type>;

        using primary_bucket_type = utils::ordered_set<secondary_bucket_type>;
        using queue_type = utils::ordered_set<primary_bucket_type>;

        two_level_bucket_queue_ll() : two_level_bucket_queue_ll(PriorityPair(), ElementInfoAccessor())
        {}

        two_level_bucket_queue_ll(PriorityPair priority_pair, ElementInfoAccessor elem_info) :
            priority_pair(std::move(priority_pair)),
            info(std::move(elem_info))
        {}

        [[nodiscard]] inline const_reference top() const
        {
            return queue.front().front().back();
        }

        void pop()
        {
            erase(info(queue.front().front().back()));
        }

        template<typename U = value_type>
        inline void push(U&& element)
        {
            ++nodes;
            auto const [first, second] = priority_pair(std::forward<U>(element));
            auto& secondary_bucket = queue[first][second];
            auto& elem_info = info(std::forward<U>(element));// = location_info{ first, second, static_cast<int>(secondary_bucket.size()) };

            secondary_bucket.emplace_back(std::forward<U>(element));

            elem_info = location_info{ first, second, std::prev(secondary_bucket.end()) };
        }

        inline void decrease_key(location_info const& loc_info)
        {
            auto element = *loc_info.third;
            erase(loc_info);
            push(element);
        }

        inline void erase(location_info const loc_info)
        {
            --nodes;
            auto& primary_bucket = queue[loc_info.first];
            auto& secondary_bucket = primary_bucket[loc_info.second];
            
            info(*loc_info.third) = location_info{};
            secondary_bucket.erase(loc_info.third);

            if(!secondary_bucket.empty()) return;

            primary_bucket.erase(loc_info.second);

            if(primary_bucket.is_empty())
            {
                queue.erase(loc_info.first);
            }
        }

        [[nodiscard]] inline size_type size() const noexcept
        {
            return nodes;
        }

        [[nodiscard]] inline size_type count() const noexcept
        {
            return nodes;
        }

        [[nodiscard]] inline bool is_empty() const noexcept
        {
            return nodes == 0;
        }

    private:
        queue_type queue;
        PriorityPair priority_pair;
        ElementInfoAccessor info;
        difference_type nodes{0};
    };
}

template<typename T>
class priority_queue_traits<T, priority_queue_policies::two_level_bucket_queue_ll>
{
    struct location_info
    {
        int first{std::numeric_limits<int>::max()};
        int second{std::numeric_limits<int>::max()};
        typename std::list<T>::iterator third{};

        [[nodiscard]] friend inline auto operator==(location_info const& lhs, location_info const& rhs) noexcept -> bool
        {
            return lhs.first == rhs.first && lhs.second == rhs.second && lhs.third == rhs.third;
        }

        [[nodiscard]] friend inline auto operator!=(location_info const& lhs, location_info const& rhs) noexcept -> bool
        {
            return !(lhs == rhs);
        }
    };

public:
    using location_info_type = location_info;
};