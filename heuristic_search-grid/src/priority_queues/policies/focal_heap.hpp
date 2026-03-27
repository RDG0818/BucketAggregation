#pragma once

#include <vector>
#include <unordered_map>
#include <iterator>
#include <limits>
#include <memory>

#include <priority_queues/priority_queue_traits.hpp>

namespace priority_queue_policies
{
    template<typename T, typename Compare, typename Focal, typename ElementInfoAccessor>
    class focal_heap
    {
    public:
        using value_type = T;
        using reference = value_type&;
        using const_reference = value_type const&;
        using pointer = value_type*;
        using const_pointer = value_type const*;
        using size_type = unsigned int;
        using difference_type = std::make_signed_t<size_type>;

        using location_info = priority_queue_traits<value_type, dual_heap>::location_info_type;

        focal_heap(Compare compare, Focal focal, ElementInfoAccessor elem_info) :
            compare(std::move(compare)),
            focal(std::move(focal)),
            info(std::move(elem_info))
        {}

        template<int index = 0>
        [[nodiscard]] inline const_reference top() const
        {
            return focal_list.front();
        }

        template<>
        [[nodiscard]] inline const_reference top<1>() const
        {
            return heap.front();
        }

        template<int index = 0>
        void pop()
        {
            erase(info(focal_list[0]));
        }

        template<>
        void pop<1>()
        {
            erase(info(heap[0]));
        }

        inline void push(value_type element)
        {
            heap.push_back(element);
            focal_list.push_back(element);

            info(heap[heap.size() - 1]) = location_info{ static_cast<difference_type>(focal_list.size()) - 1, static_cast<difference_type>(heap.size()) - 1 };

            shift_up1(focal_list.size() - 1);
            shift_up2(heap.size() - 1);
        }

        inline void decrease_key(location_info const loc_info)
        {
            // Check if in focal or not
            shift_up1(loc_info.first);
            shift_down1(loc_info.first);

            shift_up2(loc_info.second);
            shift_down2(loc_info.second);
        }

        inline void erase(location_info const loc_info)
        {
            info(heap1[loc_info.first]) = location_info{};

            using std::swap;
            swap(heap1[loc_info.first], heap1.back());
            swap(heap2[loc_info.second], heap2.back());

            heap1.pop_back();
            heap2.pop_back();

            if(loc_info.first < heap1.size())
            {
                info(heap1[loc_info.first]).first = loc_info.first;
                shift_up1(loc_info.first);
                shift_down1(loc_info.first);
            }

            if(loc_info.second < heap2.size())
            {
                info(heap2[loc_info.second]).second = loc_info.second;
                shift_up2(loc_info.second);
                shift_down2(loc_info.second);
            }
        }

        inline void reorder(auto&& should_prune)
        {
            focal_list.clear();
            for(auto i = 0; i < heap.size(); ++i)
            {
                if(!should_prune(heap[i]))
                {
                    focal_list.push_back(heap[i]);
                    info(heap[i]).second = focal_list.size() - 1;
                }
            }

            for (difference_type i = focal_list.size() / 2; i >= 0; --i)
            {
                shift_down2(i);
            }
        }

        [[nodiscard]] inline size_type size() const noexcept
        {
            return heap1.size();
        }

        [[nodiscard]] inline bool is_empty() const noexcept
        {
            return heap1.empty();
        }

    private:
        Compare compare;
        Focal focal;
        ElementInfoAccessor info;
        std::vector<value_type> heap;
        std::vector<value_type> focal_list;

        [[nodiscard]] inline difference_type parent(size_type i) const noexcept { return (i - 1u) >> 1u; };
        [[nodiscard]] inline difference_type left_child(size_type i) const noexcept { return (i << 1u) + 1u; };
        [[nodiscard]] inline difference_type right_child(size_type i) const noexcept { return (i << 1u) + 2u; };

        void shift_up1(difference_type i)
        {
            if (i == 0) return;

            auto const parent_index = parent(i);

            if (!compare1(heap1[parent_index], heap1[i]))
            {
                using std::swap;
                swap(heap1[parent_index], heap1[i]);

                info(heap1[parent_index]).first = parent_index;
                info(heap1[i]).first = i;

                shift_up1(parent_index);
            }
        }

        void shift_down1(difference_type i)
        {
            auto highestPriority = i;

            auto const l = left_child(i);
            auto const r = right_child(i);

            if (l < heap1.size() && compare1(heap1[l], heap1[highestPriority]))
            {
                highestPriority = l;
            }

            if (r < heap1.size() && compare1(heap1[r], heap1[highestPriority]))
            {
                highestPriority = r;
            }

            if (i != highestPriority)
            {
                using std::swap;
                swap(heap1[i], heap1[highestPriority]);

                info(heap1[i]).first = i;
                info(heap1[highestPriority]).first = highestPriority;

                shift_down1(highestPriority);
            }
        }

        void shift_up2(difference_type i)
        {
            if (i == 0) return;

            auto const parent_index = parent(i);

            if (!compare2(heap2[parent_index], heap2[i]))
            {
                using std::swap;
                swap(heap2[parent_index], heap2[i]);

                info(heap2[parent_index]).second = parent_index;
                info(heap2[i]).second = i;

                shift_up2(parent_index);
            }
        }

        void shift_down2(difference_type i)
        {
            auto highestPriority = i;

            auto const l = left_child(i);
            auto const r = right_child(i);

            if (l < heap2.size() && compare2(heap2[l], heap2[highestPriority]))
            {
                highestPriority = l;
            }

            if (r < heap2.size() && compare2(heap2[r], heap2[highestPriority]))
            {
                highestPriority = r;
            }

            if (i != highestPriority)
            {
                using std::swap;
                swap(heap2[i], heap2[highestPriority]);

                info(heap2[i]).second = i;
                info(heap2[highestPriority]).second = highestPriority;

                shift_down2(highestPriority);
            }
        }
    };
}

template<typename T>
class priority_queue_traits<T, priority_queue_policies::focal_heap>
{
    struct location_info
    {
        int first{std::numeric_limits<int>::max()};
        int second{std::numeric_limits<int>::max()};

        [[nodiscard]] friend inline auto operator==(location_info const& lhs, location_info const& rhs) noexcept -> bool
        {
            return lhs.first == rhs.first && lhs.second == rhs.second;
        }

        [[nodiscard]] friend inline auto operator!=(location_info const& lhs, location_info const& rhs) noexcept -> bool
        {
            return !(lhs == rhs);
        }
    };

public:
    using location_info_type = location_info;
};