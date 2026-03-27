#pragma once

#include <vector>
#include <type_traits>

namespace utils
{
    template<typename T>
        // requires std::is_nothrow_move_constructible_v<T> && std::is_nothrow_move_assignable_v<T>
    class ordered_set
    {
    public:
        using value_type = T;
        using reference = value_type&;
        using const_reference = value_type const&;
        using pointer = value_type*;
        using const_pointer = value_type const*;
        using size_type = unsigned int;
        using difference_type = std::make_signed_t<size_type>;

        friend class iterator;
        friend class const_iterator;

        constexpr ordered_set() noexcept = default;
        // ordered_set() :
        //     container(100),
        //     non_empty_indices(100)
        // {}

        constexpr ordered_set(ordered_set&&) noexcept = default;
        constexpr ordered_set& operator=(ordered_set&&) noexcept = default;
        constexpr ~ordered_set() = default;

        ordered_set(ordered_set const&) = delete;
        ordered_set& operator=(ordered_set const&) = delete;

        // [[nodiscard]] constexpr inline auto begin() noexcept -> iterator
        // {
        //     if(non_empty_count == 0) return end();

        //     return iterator(this, begin_index);
        // }

        // [[nodiscard]] constexpr inline auto cbegin() const noexcept -> const_iterator
        // {
        //     if(non_empty_count == 0) return cend();

        //     return const_iterator(this, begin_index);
        // }

        // [[nodiscard]] constexpr inline auto end() noexcept -> iterator
        // {
        //     return iterator(this, size());
        // }

        // [[nodiscard]] constexpr inline auto cend() const noexcept -> const_iterator
        // {
        //     return const_iterator(this, size());
        // }

        [[nodiscard]] constexpr auto operator[](difference_type const priority) -> reference
        {
            if(priority >= container.size()) resize(priority);

            non_empty_count += !non_empty_indices[priority];
            non_empty_indices[priority] = true;
            begin_index = std::min(begin_index, priority);

            return container[priority];
        }

        [[nodiscard]] constexpr auto operator[](difference_type const priority) const -> const_reference
        {
            return container[priority];
        }

        [[nodiscard]] constexpr auto front() -> reference
        {
            return container[begin_index];
        }

        [[nodiscard]] constexpr auto front() const -> const_reference
        {
            return container[begin_index];
        }

        constexpr inline void pop_front()
        {
            erase(begin_index);
        }

        [[nodiscard]] inline constexpr auto contains(difference_type const priority) const noexcept -> bool
        {
            return !is_empty()
                && priority < container.size()
                && non_empty_indices[priority];
        }

        constexpr void erase(difference_type const priority) //noexcept
        {
            // if(!is_valid_priority(priority) || is_empty()) return;

            --non_empty_count;
            non_empty_indices[priority] = false;

            if(is_empty())
            {
                reset_begin();
                return;
            }

            update_begin();
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
        difference_type non_empty_count{0};

        [[nodiscard]] constexpr inline auto is_valid_priority(difference_type const priority) const noexcept -> bool
        {
            return (priority < container.size()) && (priority >= begin_index) && non_empty_indices[priority];
        }

        constexpr inline void update_begin() noexcept
        {
            for(; begin_index < non_empty_indices.size(); ++begin_index)
            {
                if(non_empty_indices[begin_index]) break;
            }
        }

        constexpr inline auto reset_begin() noexcept -> void
        {
            begin_index = std::numeric_limits<difference_type>::max();
        }

        constexpr inline void resize(difference_type const priority)
        {
            container.resize(std::max(priority + 1, 2 * static_cast<difference_type>(size())));
            non_empty_indices.resize(container.size());
        }
    };
}