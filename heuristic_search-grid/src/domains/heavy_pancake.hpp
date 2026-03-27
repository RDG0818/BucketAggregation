#pragma once

#include <array>
#include <optional>
#include <algorithm>
#include <ranges>
#include <cstdlib>
#include <execution>
// #include <generator>
#include <utils/parallel_hashmap/phmap.h>

namespace search::domains
{
    template<std::size_t N>
    class heavy_pancake
    {
    public:
        using g_type = int;
        using h_type = int;
        inline static constexpr h_type max_heuristic_delta = 1;

        struct action
        {
            g_type c;

            char flip_index{0};

            [[nodiscard]] inline constexpr auto cost() const noexcept -> g_type
            {
                return c;
            }
        };

        struct state
        {
            std::array<char, N + 1> pancakes;
            std::array<char, N + 1> inverse;
            std::array<char, N + 1> xors;
            char top_pancake;
        };

        struct packed_state
        {
            std::array<char, N + 1> xors;
            std::size_t key;
            char top_pancake;

            [[nodiscard]] inline constexpr friend auto operator==(packed_state const& lhs, packed_state const& rhs) noexcept -> bool
            {
                return lhs.key == rhs.key && std::equal(std::execution::unseq, lhs.xors.cbegin(), lhs.xors.cend(), rhs.xors.cbegin());
            }

            [[nodiscard]] inline constexpr friend auto hash_value(packed_state const& val) noexcept -> std::size_t
            {
                return val.key;
            }
        };

        constexpr heavy_pancake(std::array<char, N> starting_order) noexcept :
            start_order(std::move(starting_order))
        {}

        [[nodiscard]] constexpr auto initial_state() const noexcept -> state
        {
            state s{};

            std::ranges::copy(start_order, s.pancakes.begin());
            s.pancakes[N] = N;
            s.top_pancake = s.pancakes[0];

            for(auto i = 0; i < N; ++i)
            {
                s.inverse[s.pancakes[i]] = i;
            }

            s.xors[s.pancakes[0]] = s.pancakes[1];
            for(auto i = 1; i < N; ++i)
            {
                s.xors[s.pancakes[i]] = s.pancakes[i - 1] ^ s.pancakes[i + 1];
            }
            s.xors[N] = s.pancakes[N - 1];

            return s;
        }

        [[nodiscard]] inline constexpr auto actions(state const& s, action const generating_action) const noexcept
        {
            std::vector<action> actions;
            actions.reserve(N - 1);

            for(short i = 1; i < N; ++i)
            {
                if(i == generating_action.flip_index) continue;

                actions.emplace_back(std::max(s.pancakes[0] + 1, s.pancakes[i] + 1), i);
                // actions.emplace_back(1, i);
            }

            return actions;
        }

        [[nodiscard]] constexpr auto apply(action const a, state const& s) const noexcept -> std::optional<packed_state>
        {
            // using the xors keeps us from needing to use std::reverse(...)
            auto const build_return = [&](){
                packed_state ps{s.xors, 0, s.pancakes[a.flip_index]};

                ps.xors[s.pancakes[0]] ^= s.pancakes[a.flip_index + 1];
                ps.xors[s.pancakes[a.flip_index]] ^= s.pancakes[a.flip_index + 1];
                ps.xors[s.pancakes[a.flip_index + 1]] ^= s.pancakes[a.flip_index] ^ s.pancakes[0];

                for(auto x : ps.xors)
                {
                    ps.key = phmap::HashState().combine(ps.key, x);
                }

                return ps;
            };

            return std::optional{ build_return() }; // use lambda to ensure the std::optional won't copy-construct the packed_state
        }

        constexpr inline void undo(action const& a, state const& s) const noexcept
        {
            // Undo the changes to xors that were made in apply(...)

            // s.top_pancake = s.pancakes[0];
            // s.xors[s.pancakes[0]] ^= s.pancakes[a.flip_index + 1];
            // s.xors[s.pancakes[a.flip_index]] ^= s.pancakes[a.flip_index + 1];
            // s.xors[s.pancakes[a.flip_index + 1]] ^= s.pancakes[a.flip_index] ^ s.pancakes[0];
        }

        [[nodiscard]] constexpr auto pack(state const& s) const noexcept -> packed_state
        {
            packed_state ps{s.xors, 0, s.top_pancake};
            
            for(auto x : ps.xors)
            {
                ps.key = phmap::HashState().combine(ps.key, x);
            }

            return ps;
        }

        [[nodiscard]] inline constexpr auto unpack(packed_state const& ps) const noexcept -> state
        {
            state s{{}, {}, ps.xors, ps.top_pancake};

            // Use the xors to extract the pancake positions
            s.pancakes[0] = ps.top_pancake;
            s.pancakes[1] = s.xors[s.pancakes[0]];

            for(auto i = 2; i < N; ++i)
            {
                s.pancakes[i] = s.xors[s.pancakes[i-1]] ^ s.pancakes[i-2];
            }
            s.pancakes[N] = N;
            s.xors[N] = s.pancakes[N - 1];

            // Build the inverse array
            // for(auto i = 0; i < N; ++i)
            // {
            //     s.inverse[s.pancakes[i]] = i;
            // }
            // s.inverse[s.pancakes[N]] = N;

            return s;
        }

        // [[nodiscard]] auto has_gap_decreasing_move(state const& s) const noexcept -> bool
        // {
        //     auto const top = s.pancakes[0];

        //     if(top == N - 1) return true;

        //     auto const larger_index = s.inverse[top + 1];

        //     if(std::abs(s.pancakes[larger_index - 1] - top - 1) != 1) return true;

        //     if(top > 0)
        //     {
        //         auto const smaller_index = s.inverse[top - 1];

        //         if(std::abs(s.pancakes[smaller_index - 1] - top + 1) != 1) return true;
        //     }

        //     return false;
        // }

        [[nodiscard]] constexpr auto heuristic_value(state const& s) const noexcept -> h_type
        {
            h_type h{};
            for(auto i = 1; i < N + 1; ++i)
            {
                h += std::abs(s.pancakes[i - 1] - s.pancakes[i]) > 1;
            }

            return h;// + !has_gap_decreasing_move(s);
        }

        [[nodiscard]] constexpr inline auto heuristic_value(state const& s, action const a, h_type parent_h) const noexcept -> h_type
        {
            // apply(...) only updates xors and does not affect actual pancake order, so old_val is computed using the current pancake order
            // and new_val is computed using the order if the flip had actually been applied.

            // static h_type min_h = std::numeric_limits<h_type>::max();
            
            // auto const prev_had_decreasing_moves = has_gap_decreasing_move(s);
            // parent_h -= !prev_had_decreasing_moves;

            h_type const old_val = std::abs(s.pancakes[a.flip_index] - s.pancakes[a.flip_index + 1]) > 1;
            h_type const new_val = std::abs(s.pancakes[0] - s.pancakes[a.flip_index + 1]) > 1;

            return parent_h + new_val - old_val;

            // bool has_decreasing_moves = false;
            // auto const new_top = s.pancakes[a.flip_index];
            
            // if(new_top == N - 1) has_decreasing_moves = true;

            // auto const larger_index = s.inverse[new_top + 1];
            // auto const larger_inc = (larger_index < a.flip_index) ? 1 : -1;
            // if(std::abs(s.pancakes[larger_index + larger_inc] - new_top - 1) != 1) has_decreasing_moves = true;

            // if(new_top > 0)
            // {
            //     auto const smaller_index = s.inverse[new_top - 1];
            //     auto const smaller_inc = (smaller_index < a.flip_index) ? 1 : -1;
            //     if(std::abs(s.pancakes[smaller_index + smaller_inc] - new_top + 1) != 1) has_decreasing_moves = true;
            // }

            // auto h_gap = parent_h + new_val - old_val;
            // return h_gap + !has_decreasing_moves * (h_gap > 0);
        }

    private:
        std::array<char, N> start_order;
    };
}