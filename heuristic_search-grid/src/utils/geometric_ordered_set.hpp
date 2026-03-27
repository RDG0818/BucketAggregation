#pragma once

#include <vector>
#include <type_traits>

namespace utils
{
    template<typename T>
        requires std::is_nothrow_move_constructible_v<T> && std::is_nothrow_move_assignable_v<T>
    class geometric_ordered_set
    {
    public:
        using value_type = T;
        using reference = value_type&;
        using const_reference = value_type const&;
        using pointer = value_type*;
        using const_pointer = value_type const*;
        using size_type = unsigned int;
        using difference_type = std::make_signed_t<size_type>;

        class iterator;
        friend iterator;

        constexpr geometric_ordered_set() noexcept = default;
        constexpr geometric_ordered_set(geometric_ordered_set&&) noexcept = default;
        constexpr geometric_ordered_set& operator=(geometric_ordered_set&&) noexcept = default;
        constexpr ~geometric_ordered_set() = default;

        geometric_ordered_set(geometric_ordered_set const&) = delete;
        geometric_ordered_set& operator=(geometric_ordered_set const&) = delete;

        [[nodiscard]] constexpr auto begin() -> iterator
        {
            return iterator{this, begin_index};
        }

        [[nodiscard]] constexpr auto end() -> iterator
        {
            return iterator{this, std::numeric_limits<difference_type>::max()};
        }

        [[nodiscard]] constexpr auto operator[](difference_type const priority) -> reference
        {
            handle_resize(priority);

            difference_type const index = priority & (container.size() - 1l);

            non_empty_count += !non_empty_indices[index];
            non_empty_indices[index] = true;

            if(priority < begin_value)
            {
                begin_index = index;
                begin_value = priority;
            }
            
            if(priority > end_value)
            {
                end_index = index;
                end_value = priority;
            }

            return container[index];
        }

        [[nodiscard]] constexpr auto operator[](difference_type const priority) const -> const_reference
        {
            return container[priority & (container.size() - 1l)];
        }

        [[nodiscard]] constexpr inline auto front() -> reference
        {
            return container[begin_index];
        }

        [[nodiscard]] constexpr inline auto front() const -> const_reference
        {
            return container[begin_index];
        }

        constexpr void pop_front()
        {
            erase(begin_value);
        }

        [[nodiscard]] inline constexpr auto contains(difference_type const priority) const noexcept -> bool
        {
            return !is_empty()
                && begin_value <= priority && priority <= end_value
                && non_empty_indices[priority & (container.size() - 1l)];
        }

        constexpr void erase(difference_type const priority) //noexcept
        {
            // if(!is_valid_priority(priority) || is_empty()) return;

            --non_empty_count;
            difference_type const index = priority & (container.size() - 1l);
            non_empty_indices[index] = false;
            container[index] = value_type{};

            if(is_empty())
            {
                reset_begin_end();
                return;
            }

            update_begin_end();
        }

        constexpr void set_priority_limit(difference_type const priority_limit)
        {
            auto const range = priority_limit - begin_value;
            if(range <= (container.size() << 2)) shrink(range);
        }

        [[nodiscard]] constexpr inline auto size() const noexcept -> size_type
        {
            return container.size();
        }

        [[nodiscard]] constexpr inline auto count() const noexcept -> difference_type
        {
            return non_empty_count;
        }

        [[nodiscard]] constexpr inline auto is_empty() const noexcept -> bool
        {
            return non_empty_count == 0;
        }

    private:
        std::vector<T> container;
        std::vector<bool> non_empty_indices;
        difference_type begin_index{std::numeric_limits<difference_type>::max()};
        difference_type begin_value{std::numeric_limits<difference_type>::max()};
        difference_type end_index{0};
        difference_type end_value{0};
        difference_type non_empty_count{0};

        constexpr inline void update_begin_end() noexcept
        {
            auto const size_mask = container.size() - 1;

            for(auto i = 0; i < container.size(); ++i)
            {
                if(non_empty_indices[begin_index]) break;

                begin_index = (begin_index + 1) & size_mask;
                ++begin_value;
            }

            for(auto i = 0; i < container.size(); ++i)
            {
                if(non_empty_indices[end_index]) break;

                end_index = (end_index + size_mask) & size_mask;
                --end_value;
            }
        }

        constexpr inline void reset_begin_end() noexcept
        {
            begin_index = begin_value = std::numeric_limits<difference_type>::max();
            end_index = end_value = 0;
        }

        constexpr inline void handle_resize(difference_type const priority)
        {
            difference_type required_range = std::max({1, end_value - priority + 1, priority - begin_value + 1});

            if (required_range > container.size())// || required_range <= (container.size() << 2))
            {
                // Compute the power of 2 that is >= requiredRange
                size_type n = required_range - 1;
                n |= n >> 1;
                n |= n >> 2;
                n |= n >> 4;
                n |= n >> 8;
                n |= n >> 16;
                // n |= n >> 32;
                ++n;
                resize(n);
            }
        }

        constexpr inline void shrink(difference_type const priority_limit)
        {
            // Compute the power of 2 that is >= requiredRange
            size_type n = priority_limit - 1;
            n |= n >> 1;
            n |= n >> 2;
            n |= n >> 4;
            n |= n >> 8;
            n |= n >> 16;
            // n |= n >> 32;
            ++n;
            resize(n);
        }

        constexpr inline void resize(size_type const new_size)
        {
            std::vector<value_type> new_container(new_size);
            std::vector<bool> new_non_empty(new_size, false);
            auto const size_mask = container.size() - 1;
            auto const new_size_mask = new_size - 1;

            for(auto value = begin_value, index = begin_index; value <= end_value; ++value, index = (index + 1) & size_mask)
            {
                if(non_empty_indices[index])
                {
                    new_container[value & new_size_mask] = std::move(container[index]);
                    new_non_empty[value & new_size_mask] = true;
                }
            }

            std::swap(container, new_container);
            std::swap(non_empty_indices, new_non_empty);
            
            if(!is_empty())
            {
                begin_index = begin_value & new_size_mask;
                end_index = end_value & new_size_mask;
            }
        }

    public:
        // Invalidated on container resize
        class iterator
        {
            friend geometric_ordered_set<T>;
        
            constexpr iterator(geometric_ordered_set<T>* container, difference_type const index) noexcept :
                container(container),
                index(index)
            {}

        public:
            constexpr iterator() noexcept = default;

            [[nodiscard]] friend inline auto operator==(const iterator& lhs, const iterator& rhs) noexcept -> bool
            {
                return lhs.index == rhs.index && lhs.container == rhs.container;
            }

            [[nodiscard]] friend inline auto operator!=(const iterator& lhs, const iterator& rhs) noexcept -> bool
            {
                return !(lhs == rhs);
            }

            [[nodiscard]] constexpr inline auto operator*() const -> T&
            {
                return container->container[index];
            }

            // Prefix: ++it
            iterator& operator++()
            {
                if(container->is_empty()) 
                {
                    *this = container->end();
                    return *this;
                }
                
                if(index == container->end_index)
                {
                    index = std::numeric_limits<difference_type>::max();
                    return *this;
                }

                do
                {
                    index = (index + 1) & (container->container.size() - 1l);
                } while (!container->non_empty_indices[index] && index != container->end_index);

                return *this;
            }

            // Postfix: it++
            iterator operator++(int)
            {
                auto copy = *this;

                if(container->is_empty()) 
                {
                    *this = container->end();
                    return copy;
                }

                if(index == container->end_index)
                {
                    index = std::numeric_limits<difference_type>::max();
                    return copy;
                }

                do
                {
                    index = (index + 1) & (container->container.size() - 1l);
                } while (!container->non_empty_indices[index] && index != container->end_index);

                return copy;
            }

            // Prefix: --it
            iterator& operator--()
            {
                if(container->is_empty()) 
                {
                    *this = container->end();
                    return *this;
                }

                if(index == container->begin_index)
                {
                    index = std::numeric_limits<difference_type>::max();
                    return *this;
                }

                if(index == std::numeric_limits<difference_type>::max())
                {
                    index = container->end_index;
                    return *this;
                }

                do
                {
                    index = (index + container->container.size() - 1l) & (container->container.size() - 1l);
                } while (!container->non_empty_indices[index] && index != container->begin_index);

                return *this;
            }

            // Postfix: it--
            iterator operator--(int)
            {
                auto copy = *this;

                if(container->is_empty()) 
                {
                    *this = container->end();
                    return copy;
                }

                if(index == container->begin_index)
                {
                    index = std::numeric_limits<difference_type>::max();
                    return copy;
                }

                if(index == std::numeric_limits<difference_type>::max())
                {
                    index = container->end_index;
                    return copy;
                }

                do
                {
                    index = (index + container->container.size() - 1l) & (container->container.size() - 1l);
                } while (!container->non_empty_indices[index] && index != container->begin_index);

                return copy;
            }

        private:
            geometric_ordered_set<T>* container{nullptr};
            difference_type index{std::numeric_limits<difference_type>::max()};
        };
    };
}