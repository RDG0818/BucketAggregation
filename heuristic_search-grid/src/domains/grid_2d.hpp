#pragma once

#include <array>
#include <vector>
#include <optional>
#include <algorithm>
#include <ranges>
#include <cstdlib>

namespace search::domains
{
    template<bool uniform_cost = true>
    class grid_2d
    {
    public:
        using g_type = int;
        using h_type = int;
        inline static constexpr h_type max_heuristic_delta = 1;

        enum direction
        {
            NORTH = 0,
            EAST = 1,
            SOUTH = 2,
            WEST = 3,
            INVALID
        };

        struct action
        {
            char c = 1;

            direction d{direction::INVALID};

            [[nodiscard]] inline constexpr auto cost() const noexcept -> char
            {
                return c;
            }
        };

        struct state
        {
            std::size_t key;
            int x, y, index;
        };

        struct packed_state 
        {
            std::size_t key;

            [[nodiscard]] inline constexpr friend auto operator==(packed_state const& lhs, packed_state const& rhs) noexcept -> bool
            {
                return lhs.key == rhs.key;
            }

            [[nodiscard]] inline constexpr friend auto hash_value(packed_state const& val) noexcept -> std::size_t
            {
                return val.key;
            }
        };

        grid_2d(std::vector<char> grid, std::pair<int, int> dimensions, std::pair<int, int> initial_loc, std::pair<int, int> goal_loc) noexcept :
            dimensions(std::move(dimensions)),
            initial_loc(std::move(initial_loc)),
            goal_loc(std::move(goal_loc)),
            grid(std::move(grid))
        {}

        [[nodiscard]] constexpr auto initial_state() const noexcept -> state
        {
            return state{
                (static_cast<std::size_t>(initial_loc.first) << 32) | initial_loc.second,
                initial_loc.first,
                initial_loc.second,
                initial_loc.second * dimensions.first + initial_loc.first
            };
        }

        template<bool u = uniform_cost>
        [[nodiscard]] inline constexpr auto actions(state const& s, action const generating_action) const noexcept
        {
            static constexpr auto actions = {action{1, direction::NORTH}, action{1, direction::SOUTH}, action{1, direction::EAST}, action{1, direction::WEST}};

            // for(auto i = 0; i < actions.size(); ++i)
            // {
            //     actions[i] = {1, static_cast<direction>(i)};
            // }

            return actions | std::views::filter([=](auto const& a) -> bool{
                return ((a.d & 0x01) != (generating_action.d & 0x01)) || generating_action.d == direction::INVALID || a.d == generating_action.d;
            });
        }

        template<>
        [[nodiscard]] auto actions<false>(state const& s, action const generating_action) const noexcept
        {
            std::vector<action> actions;
            actions.reserve(4);

            for(auto i = 0; i < 4; ++i)
            {
                auto [x_offset, y_offset] = xy_offsets[i];
                auto const x = s.x + x_offset;
                auto  const y = s.y + y_offset;

                if(x >= dimensions.first || x < 0
                || y >= dimensions.second || y < 0) continue;

                auto const index = y * dimensions.first + x;

                auto const cost = std::abs(grid[index] - grid[s.index]) + 1;

                actions.emplace_back(cost, static_cast<direction>(i));
            }

            return actions;
        }

        [[nodiscard]] inline constexpr auto apply(action const& a, state const& s) const noexcept -> std::optional<packed_state>
        {
            auto [x_offset, y_offset] = xy_offsets[a.d];
            auto x = s.x + x_offset;
            auto y = s.y + y_offset;

            if(x >= dimensions.first || x < 0
            || y >= dimensions.second || y < 0) return std::nullopt;

            auto const index = y * dimensions.first + x;

            if constexpr(uniform_cost)
            {
                if(grid[index] != 0) return std::nullopt;
            }

            return std::optional{ packed_state{(static_cast<std::size_t>(x) << 32) | y} };
        }

        inline void undo(action const& a, state const& s) const noexcept
        {}

        [[nodiscard]] inline constexpr auto pack(state const& s) const noexcept -> packed_state
        {
            return packed_state{s.key};
        }

        [[nodiscard]] constexpr auto unpack(packed_state const& ps) const noexcept -> state
        {
            constexpr std::size_t bit_mask = (~0ul) >> 32;

            state s{};
            s.key = ps.key;
            s.y = ps.key & bit_mask;
            s.x = (ps.key >> 32) & bit_mask;
            s.index = s.y * dimensions.first + s.x;

            return s;
        }

        [[nodiscard]] inline constexpr auto heuristic_value(state const& s) const noexcept -> h_type
        {
            return std::abs(s.x - goal_loc.first) + std::abs(s.y - goal_loc.second);
        }

        [[nodiscard]] inline constexpr auto heuristic_value(state const& s, action const& a, char parent_h) const noexcept -> h_type
        {
            auto [x_offset, y_offset] = xy_offsets[a.d];
            
            auto h = std::abs(s.x + x_offset - goal_loc.first) + std::abs(s.y + y_offset - goal_loc.second);
            if(h == 0)
            {
                bool found = true;
            }
            return h;
        }

    private:
        std::vector<char> grid;
        std::pair<int, int> dimensions;
        std::pair<int, int> initial_loc, goal_loc;

        // North, East, South, West
        inline static constexpr auto xy_offsets = std::array{std::pair{0, -1}, std::pair{1, 0}, std::pair{0, 1}, std::pair{-1, 0}};
    };
}

// template<>
// struct std::hash<search::domains::grid_2d<true>::packed_state>
// {
//     [[nodiscard]] inline constexpr auto operator()(search::domains::grid_2d::packed_state const& key) const noexcept -> std::size_t
//     {
//         return key.key;
//     }
// };