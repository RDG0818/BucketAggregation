#pragma once

#include <vector>
#include <list>
#include <type_traits>
#include <memory_resource>

namespace utils
{
    template<typename T>
        // requires std::is_nothrow_move_constructible_v<T> && std::is_nothrow_move_assignable_v<T>
    class lazy_ordered_set
    {
    public:
        using value_type = T;
        using reference = value_type&;
        using const_reference = value_type const&;
        using pointer = value_type*;
        using const_pointer = value_type const*;
        using size_type = unsigned int;
        using difference_type = std::make_signed_t<size_type>;

        // constexpr ordered_set() noexcept = default;
        lazy_ordered_set() :
            // pool(),
            // allocator(&pool),
            container(100),
            non_empty_indices(100)
            // active_list(allocator)
        {}

        constexpr lazy_ordered_set(lazy_ordered_set&&) noexcept = default;
        constexpr lazy_ordered_set& operator=(lazy_ordered_set&&) noexcept = default;
        constexpr ~lazy_ordered_set() = default;

        lazy_ordered_set(lazy_ordered_set const&) = delete;
        lazy_ordered_set& operator=(lazy_ordered_set const&) = delete;


        [[nodiscard]] constexpr auto operator[](difference_type const priority) -> reference
        {
            if(priority >= container.size()) resize(priority);

            if(active_list_constructed && !non_empty_indices[priority])
            {
                active_list.insert(std::ranges::upper_bound(active_list, priority), priority);
            }

            non_empty_count += !non_empty_indices[priority];
            non_empty_indices[priority] = true;

            return container[priority];
        }

        [[nodiscard]] constexpr auto operator[](difference_type const priority) const -> const_reference
        {
            return container[priority];
        }

        [[nodiscard]] constexpr auto front() -> reference
        {
            if(!active_list_constructed) construct_active_list();

            return container[active_list.front()];
        }

        // [[nodiscard]] constexpr auto front() const -> const_reference
        // {
        //     if(!active_list_constructed) construct_active_list();

        //     return container[active_list.front()];
        // }

        constexpr inline void pop_front()
        {
            if(!active_list_constructed) throw;

            erase(active_list.front());
        }

        constexpr void erase(difference_type const priority) //noexcept
        {
            --non_empty_count;
            non_empty_indices[priority] = false;

            if(active_list_constructed)
            {
                // if(priority != active_list.front()) throw;

                active_list.pop_front();
            }
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
        // std::pmr::unsynchronized_pool_resource pool;
        // std::pmr::polymorphic_allocator<difference_type> allocator;
        std::vector<T> container;
        std::vector<bool> non_empty_indices;
        std::list<difference_type> active_list;
        difference_type non_empty_count{0};
        bool active_list_constructed{false};

        constexpr inline void resize(difference_type const priority)
        {
            container.resize(std::max(priority + 1, 2 * static_cast<difference_type>(size())));
            non_empty_indices.resize(container.size());
        }

        void construct_active_list()
        {
            for(auto i = 0; i < container.size(); ++i)
            {
                if(non_empty_indices[i]) active_list.push_back(i);
            }

            active_list_constructed = true;
        }
    };
}