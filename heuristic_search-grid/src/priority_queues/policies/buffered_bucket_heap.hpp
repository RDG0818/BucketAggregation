#pragma once

#include <vector>
#include <unordered_map>
#include <iterator>
#include <limits>
#include <memory>
#include <iostream>

#include <priority_queues/priority_queue_traits.hpp>
#include <priority_queues/policies/binary_heap.hpp>
#include <priority_queues/policies/two_level_bucket_queue.hpp>
#include <utils/ordered_set.hpp>
#include <utils/geometric_ordered_set.hpp>

namespace priority_queue_policies
{
    template<typename T, typename PriorityPair, typename Compare, typename ElementInfoAccessor>
    class buffered_bucket_heap
    {
    public:
        using value_type = T;
        using reference = value_type&;
        using const_reference = value_type const&;
        using pointer = value_type*;
        using const_pointer = value_type const*;
        using size_type = unsigned int;
        using difference_type = std::make_signed_t<size_type>;

        using location_info = priority_queue_traits<value_type, buffered_bucket_heap>::location_info_type;

        using secondary_bucket_type = std::vector<value_type>;
        using primary_bucket_type = utils::ordered_set<secondary_bucket_type>;
        // using bucket_queue_type = utils::ordered_set<primary_bucket_type>;

        // using primary_bucket_type = utils::geometric_ordered_set<secondary_bucket_type>;
        using bucket_queue_type = utils::geometric_ordered_set<primary_bucket_type>;

        struct bucket_data;
        using heap_location_info = priority_queue_traits<difference_type, priority_queue_policies::binary_heap>::location_info_type;

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

            [[nodiscard]] inline bool operator()(difference_type const lhs, difference_type const rhs) const noexcept
            {
                return compare(bucket_queue[lhs].front().back(), bucket_queue[rhs].front().back());
            }

        private:
            Compare compare;
            bucket_queue_type const& bucket_queue;
        };

        struct bucket_info_accessor
        {
            bucket_info_accessor(utils::geometric_ordered_set<bucket_data>& buckets) :
                buckets(buckets)
            {}

            [[nodiscard]] inline auto& operator()(difference_type const bucket) noexcept
            {
                return buckets[bucket].info;
            }

        private:
            utils::geometric_ordered_set<bucket_data>& buckets;
        };

        using heap_type = binary_heap<difference_type, bucket_compare, bucket_info_accessor>;

        buffered_bucket_heap(PriorityPair priority_pair, Compare compare, ElementInfoAccessor elem_info) :
            heap(bucket_compare(std::move(compare), bucket_queue), bucket_info_accessor(active_buckets)),
            priority_pair(std::move(priority_pair)),
            info(std::move(elem_info))
        {}

        template<int index = 0>
        [[nodiscard]] inline const_reference top() const
        {
            return bucket_queue[heap.top()].front().back();
        }

        template<>
        [[nodiscard]] inline const_reference top<1>() const
        {
            return bucket_queue.front().front().back();
        }

        template<int index = 0>
        void pop()
        {
            --count;
            auto const bucket = heap.top();
            auto& primary_bucket = bucket_queue[bucket];
            auto& secondary_bucket = primary_bucket.front();

            info(secondary_bucket.back()) = location_info{};
            secondary_bucket.pop_back();

            if(!secondary_bucket.empty()) return;

            flush_insertion_buffer(bucket);

            primary_bucket.pop_front();

            if(!primary_bucket.is_empty())
            {
                heap.decrease_key(active_buckets[bucket].info);
            }
            else
            {
                heap.pop();
                active_buckets.erase(bucket);
                bucket_queue.erase(bucket);
            }
        }

        template<>
        void pop<1>()
        {
            --count;
            auto& primary_bucket = bucket_queue.front();
            auto& secondary_bucket = primary_bucket.front();
            auto const bucket = priority_pair(secondary_bucket.back()).first;

            info(secondary_bucket.back()) = location_info{};
            secondary_bucket.pop_back();

            if(!secondary_bucket.empty()) return;

            flush_insertion_buffer(bucket);

            primary_bucket.pop_front();

            if(!primary_bucket.is_empty())
            {
                heap.decrease_key(active_buckets[bucket].info);
            }
            else
            {
                // heap.pop();
                heap.erase(active_buckets[bucket].info);
                active_buckets.erase(bucket);
                bucket_queue.erase(bucket);
            }
        }

        template<typename U = value_type>
        inline void push(U&& element)
        {
            ++count;
            auto const [priority1, priority2] = priority_pair(element);

            auto& primary_bucket = bucket_queue[priority1];

            if(!primary_bucket.is_empty())
            {
                auto const [p1, p2] = priority_pair(primary_bucket.front().back());

                if(priority2 > p2)
                {
                    auto& queue = insertion_buffers[priority1];
                    queue.push_back(std::forward<U>(element));
                    info(queue.back()) = location_info{-priority1, priority2, static_cast<int>(queue.size()) - 1};
                }
                else
                {
                    auto& secondary_bucket = primary_bucket[priority2];
                    secondary_bucket.push_back(std::forward<U>(element));
                    info(secondary_bucket.back()) = location_info{priority1, priority2, static_cast<int>(secondary_bucket.size()) - 1};
                    
                    // might be faster to do a check instead of just doing this
                    heap.decrease_key(active_buckets[priority1].info);
                }
            }
            else
            {
                auto& secondary_bucket = primary_bucket[priority2];
                secondary_bucket.push_back(std::forward<U>(element));
                info(secondary_bucket.back()) = location_info{priority1, priority2, static_cast<int>(secondary_bucket.size()) - 1};

                active_buckets[priority1] = bucket_data{};
                heap.push(priority1);
            }
        }

        inline void decrease_key(location_info const& loc_info)
        {
            if(loc_info.first < 0)
            {
                --count;
                auto& buffer = insertion_buffers[-loc_info.first];
                auto element = buffer[loc_info.third];
                std::swap(buffer[loc_info.third], buffer.back());
                info(buffer[loc_info.third]).third = loc_info.third;
                // info(buffer.back()) = location_info{};
                buffer.pop_back();
                push(element);
            }
            else
            {
                auto element = bucket_queue[loc_info.first][loc_info.second][loc_info.third];
                erase(loc_info);
                push(element);
            }
        }

        inline void erase(location_info const loc_info)
        {
            --count;
            auto& primary_bucket = bucket_queue[loc_info.first];
            auto& secondary_bucket = primary_bucket[loc_info.second];
            std::swap(secondary_bucket[loc_info.third], secondary_bucket.back());
            info(secondary_bucket[loc_info.third]).third = loc_info.third;
            info(secondary_bucket.back()) = location_info{};
            secondary_bucket.pop_back();

            // if(loc_info.third < secondary_bucket.size())
            // {
            //     location_info& i = info(secondary_bucket[loc_info.third]);
            //     i.third = loc_info.third;
            //     return;
            // }

            if(!secondary_bucket.empty()) return;

            if(primary_bucket.front() == secondary_bucket) flush_insertion_buffer(loc_info.first);

            primary_bucket.erase(loc_info.second);

            if(!primary_bucket.is_empty())
            {
                heap.decrease_key(active_buckets[loc_info.first].info);
            }
            else
            {
                heap.erase(active_buckets[loc_info.first].info);
                active_buckets.erase(loc_info.first);
                bucket_queue.erase(loc_info.first);
            }
        }

        inline void reorder(difference_type const priority_limit)
        {
            heap.reorder(priority_limit);
        }

        [[nodiscard]] inline size_type size() const noexcept
        {
            return count;
        }

        [[nodiscard]] inline bool is_empty() const noexcept
        {
            return count == 0;
        }

        // ~buffered_bucket_heap()
        // {
        //     std::cout << "Total Flushed: " << std::to_string(total_flushed) << "\n";
        //     std::cout << "Flush Count: " << std::to_string(flush_count) << "\n";
        //     std::cout << "Avg. Flush Size: " << std::to_string(total_flushed / (float)flush_count) << "\n";
        // }

    private:
        bucket_queue_type bucket_queue;
        utils::geometric_ordered_set<std::vector<value_type>> insertion_buffers;
        heap_type heap;
        utils::geometric_ordered_set<bucket_data> active_buckets;
        PriorityPair priority_pair;
        ElementInfoAccessor info;
        difference_type count{0};//, flush_count{0}, total_flushed{0};

        void flush_insertion_buffer(difference_type const priority)
        {
            auto& primary_bucket = bucket_queue[priority];
            auto& buffer = insertion_buffers[priority];

            for(auto const element : buffer)
            {
                auto& i = info(element);
                i.first *= -1;
                auto& secondary_bucket = primary_bucket[i.second];
                i.third = secondary_bucket.size();
                secondary_bucket.push_back(element);
            }

            // if(!buffer.empty())
            // {
            //     ++flush_count;
            //     total_flushed += buffer.size();
            // }

            buffer.clear();
        }
    };
}

template<typename T>
class priority_queue_traits<T, priority_queue_policies::buffered_bucket_heap>
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