#pragma once

#include <vector>
#include <unordered_map>
#include <iterator>
#include <limits>
#include <memory>

#include <priority_queues/priority_queue_traits.hpp>
#include <priority_queues/policies/binary_heap.hpp>
#include <priority_queues/policies/two_level_bucket_queue.hpp>
#include <utils/ordered_set.hpp>
#include <utils/geometric_ordered_set.hpp>

namespace priority_queue_policies
{
    template<typename T, typename PriorityPair, typename Compare, typename ElementInfoAccessor>
    class gilon_heap
    {
    public:
        using value_type = T;
        using reference = value_type&;
        using const_reference = value_type const&;
        using pointer = value_type*;
        using const_pointer = value_type const*;
        using size_type = unsigned int;
        using difference_type = std::make_signed_t<size_type>;

        using location_info = priority_queue_traits<value_type, gilon_heap>::location_info_type;

        using secondary_bucket_type = std::vector<value_type>;
        using primary_bucket_type = utils::ordered_set<secondary_bucket_type>;
        // using bucket_queue_type = utils::ordered_set<primary_bucket_type>;

        // using primary_bucket_type = utils::geometric_ordered_set<secondary_bucket_type>;
        using bucket_queue_type = utils::geometric_ordered_set<primary_bucket_type>;

        struct bucket_data;
        using heap_location_info = priority_queue_traits<std::pair<difference_type, difference_type>, priority_queue_policies::binary_heap>::location_info_type;

        struct bucket_data
        {
            heap_location_info info{};
        };

        struct bucket_compare
        {
            bucket_compare(Compare compare, bucket_queue_type const& bucket_queue) :
                compare(std::move(compare)),
                bucket_queue(bucket_queue)
            {}

            [[nodiscard]] inline bool operator()(std::pair<difference_type, difference_type> const lhs, std::pair<difference_type, difference_type> const rhs) const noexcept
            {
                return compare(bucket_queue[lhs.first][lhs.second].back(), bucket_queue[rhs.first][rhs.second].back());
            }

        private:
            Compare compare;
            bucket_queue_type const& bucket_queue;
        };

        struct bucket_info_accessor
        {
            bucket_info_accessor(utils::geometric_ordered_set<utils::geometric_ordered_set<bucket_data>>& buckets) :
                buckets(buckets)
            {}

            [[nodiscard]] inline auto& operator()(std::pair<difference_type, difference_type> const bucket) noexcept
            {
                return buckets[bucket.first][bucket.second].info;
            }

        private:
            utils::geometric_ordered_set<utils::geometric_ordered_set<bucket_data>>& buckets;
        };

        using heap_type = binary_heap<std::pair<difference_type, difference_type>, bucket_compare, bucket_info_accessor>;

        gilon_heap(PriorityPair priority_pair, Compare compare, ElementInfoAccessor elem_info) :
            heap(bucket_compare(std::move(compare), bucket_queue), bucket_info_accessor(active_buckets)),
            priority_pair(std::move(priority_pair)),
            info(std::move(elem_info))
        {}

        template<int index = 0>
        [[nodiscard]] inline const_reference top() const
        {
            auto [primary, secondary] = heap.top();
            return bucket_queue[primary][secondary].back();
        }

        template<>
        [[nodiscard]] inline const_reference top<1>() const
        {
            return bucket_queue.front().front().back();
        }

        template<int index = 0>
        void pop()
        {
            --nodes;
            auto [primary, secondary] = heap.top();
            auto& primary_bucket = bucket_queue[primary];
            auto& secondary_bucket = primary_bucket[secondary];

            info(secondary_bucket.back()) = location_info{};
            secondary_bucket.pop_back();

            // Update heap and active buckets
            if(secondary_bucket.empty())
            {
                heap.pop();
                primary_bucket.erase(secondary);
                active_buckets[primary].erase(secondary);

                if(primary_bucket.is_empty())
                {
                    bucket_queue.erase(primary);
                }
            }
        }

        template<>
        void pop<1>()
        {
            --nodes;
            auto& primary_bucket = bucket_queue.front();
            auto& secondary_bucket = primary_bucket.front();
            auto [primary, secondary] = priority_pair(secondary_bucket.back());

            info(secondary_bucket.back()) = location_info{};
            secondary_bucket.pop_back();

            if(secondary_bucket.empty())
            {
                heap.erase(active_buckets[primary][secondary].info);
                primary_bucket.pop_front();
                active_buckets[primary].erase(secondary);

                if(primary_bucket.is_empty())
                {
                    bucket_queue.erase(primary);
                }
            }
        }

        template<typename U = value_type>
        inline void push(U&& element)
        {
            ++nodes;
            auto const [priority1, priority2] = priority_pair(element);

            auto& primary_bucket = bucket_queue[priority1];
            auto& secondary_bucket = primary_bucket[priority2];
            secondary_bucket.push_back(std::forward<U>(element));
            info(secondary_bucket.back()) = location_info{priority1, priority2, static_cast<int>(secondary_bucket.size()) - 1};

            if(secondary_bucket.size() == 1)
            {
                active_buckets[priority1][priority2] = bucket_data{};
                heap.push(std::make_pair(priority1, priority2));
            }
        }

        inline void decrease_key(location_info const& loc_info)
        {
            auto element = bucket_queue[loc_info.first][loc_info.second][loc_info.third];
            erase(loc_info);
            push(element);
        }

        inline void erase(location_info const loc_info)
        {
            --nodes;
            auto& primary_bucket = bucket_queue[loc_info.first];
            auto& secondary_bucket = primary_bucket[loc_info.second];
            std::swap(secondary_bucket[loc_info.third], secondary_bucket.back());
            info(secondary_bucket.back()) = location_info{};
            secondary_bucket.pop_back();

            if(loc_info.third < secondary_bucket.size())
            {
                location_info& i = info(secondary_bucket[loc_info.third]);
                i.third = loc_info.third;
                return;
            }

            if(secondary_bucket.empty())
            {
                heap.erase(active_buckets[loc_info.first][loc_info.second].info);
                primary_bucket.erase(loc_info.second);
                active_buckets[loc_info.first].erase(loc_info.second);

                if(primary_bucket.is_empty())
                {
                    bucket_queue.erase(loc_info.first);
                }
            }
        }

        inline void reorder(difference_type const priority_limit)
        {
            // if(bucket_queue.is_empty()) return;

            // auto it = bucket_queue.end();
            // bool pruned;

            // do
            // {
            //     pruned = false;
            //     --it;

            //     auto const [priority1, priority2] = priority_pair((*it).front().back());

            //     if(priority1 >= priority_limit)
            //     {
            //         pruned = true;
            //         heap.erase(active_buckets[priority1].info);
            //         active_buckets.erase(priority1);
            //         bucket_queue.erase(priority1);
            //     }

            // } while (it != bucket_queue.begin() && !bucket_queue.is_empty() && pruned);

            heap.reorder(priority_limit);
        }

        [[nodiscard]] inline size_type size() const noexcept
        {
            return nodes;
        }

        [[nodiscard]] inline size_type count() const noexcept
        {
            return heap.size();
        }

        [[nodiscard]] inline bool is_empty() const noexcept
        {
            return bucket_queue.is_empty();
        }

    private:
        bucket_queue_type bucket_queue;
        heap_type heap;
        utils::geometric_ordered_set<utils::geometric_ordered_set<bucket_data>> active_buckets;
        PriorityPair priority_pair;
        ElementInfoAccessor info;
        difference_type nodes{0};
    };
}

template<typename T>
class priority_queue_traits<T, priority_queue_policies::gilon_heap>
{
    struct location_info
    {
        int first{std::numeric_limits<int>::max()};
        int second{std::numeric_limits<int>::max()};
        int third{std::numeric_limits<int>::max()};

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