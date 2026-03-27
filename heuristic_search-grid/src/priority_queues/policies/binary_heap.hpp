#pragma once

#include <vector>
#include <unordered_map>
#include <iterator>
#include <limits>
#include <memory>

#include <priority_queues/priority_queue_traits.hpp>

namespace priority_queue_policies
{
    template<typename T, typename Compare, typename ElementInfoAccessor>
    class binary_heap
    {
    public:
        using value_type = T;
        using reference = value_type&;
        using const_reference = value_type const&;
        using pointer = value_type*;
        using const_pointer = value_type const*;
        using size_type = unsigned int;
        using difference_type = std::make_signed_t<size_type>;

        using location_info = priority_queue_traits<value_type, binary_heap>::location_info_type;
        // using location_info = std::remove_cv_t<decltype(std::declval<ElementInfoAccessor>()(std::declval<value_type>()))>;
        // binary_heap() : binary_heap(Compare(), ElementInfoAccessor())
        // {}

        binary_heap(Compare compare, ElementInfoAccessor elem_info) :
            compare(std::move(compare)),
            info(std::move(elem_info))
        {}

        [[nodiscard]] inline const_reference top() const
        {
            return heap.front();
        }

        void pop()
        {
            erase(location_info{ 0 });
        }

        // template<typename U = value_type>
        // inline void push(U&& element)
        // {
        //     heap.push_back(std::forward<U>(element));
        //     info(heap[heap.size() - 1]) = location_info{ static_cast<difference_type>(heap.size()) - 1 };

        //     shift_up(heap.size() - 1);
        // }

        inline void push(value_type element)
        {
            heap.push_back(element);
            info(heap[heap.size() - 1]) = location_info{ static_cast<difference_type>(heap.size()) - 1 };

            shift_up(heap.size() - 1);
        }

        inline void decrease_key(location_info const loc_info)
        {
            shift_up(loc_info);
            shift_down(loc_info);
        }

        inline void erase(location_info const loc_info)
        {
            info(heap[loc_info]) = location_info{};

            using std::swap;
            swap(heap[loc_info], heap.back());

            heap.pop_back();

            if(loc_info < heap.size())
            {
                info(heap[loc_info]) = loc_info;
                shift_up(loc_info);
                shift_down(loc_info);
            }
        }

        inline void reorder(difference_type const priority_limit)
        {
            for (difference_type i = heap.size() / 2; i >= 0; --i)
            {
                shift_down(i);
            }
        }

        [[nodiscard]] inline size_type size() const noexcept
        {
            return heap.size();
        }

        [[nodiscard]] inline size_type count() const noexcept
        {
            return heap.size();
        }

        [[nodiscard]] inline bool is_empty() const noexcept
        {
            return !heap.size();
        }

    public://private:
        Compare compare;
        ElementInfoAccessor info;
        std::vector<value_type> heap;

        [[nodiscard]] inline difference_type parent(size_type i) const noexcept { return (i - 1u) >> 1u; };
        [[nodiscard]] inline difference_type left_child(size_type i) const noexcept { return (i << 1u) + 1u; };
        [[nodiscard]] inline difference_type right_child(size_type i) const noexcept { return (i << 1u) + 2u; };

        void shift_up(difference_type const i)
        {
            if (i == 0) return;

            auto const parent_index = parent(i);

            if (!compare(heap[parent_index], heap[i]))
            {
                using std::swap;
                swap(heap[parent_index], heap[i]);

                info(heap[parent_index]) = location_info{ parent_index };
                info(heap[i]) = location_info{ i };

                shift_up(parent_index);
            }
        }

        void shift_down(difference_type i)
        {
            auto highestPriority = i;

            auto const l = left_child(i);
            auto const r = right_child(i);

            if (l < heap.size() && compare(heap[l], heap[highestPriority]))
            {
                highestPriority = l;
            }

            if (r < heap.size() && compare(heap[r], heap[highestPriority]))
            {
                highestPriority = r;
            }

            if (i != highestPriority)
            {
                using std::swap;
                swap(heap[i], heap[highestPriority]);

                info(heap[i]) = location_info{ i };
                info(heap[highestPriority]) = location_info{ highestPriority };

                shift_down(highestPriority);
            }
        }
    };
}