#pragma once

#include <array>
#include <optional>
#include <algorithm>
#include <ranges>
#include <cstdlib>
#include <execution>
#include <utils/parallel_hashmap/phmap.h>

namespace search::domains
{
    // Horrible, monstrous code, specifically optimized for the 48 pancake puzzle...
    class pancake_48
    {
        inline static constexpr int N = 48;
    public:
        using g_type = int;
        using h_type = int;
        inline static constexpr h_type max_heuristic_delta = 1;

        struct action
        {
            char flip_index{0};

            [[nodiscard]] inline constexpr auto cost() const noexcept -> g_type
            {
                return 1;
            }
        };

        struct state
        {
            std::array<std::size_t, 5> packed_xors;
            std::array<char, N + 1> pancakes;
            std::array<char, N + 1> xors;
            char top_pancake;
        };

        struct packed_state
        {
            std::array<std::size_t, 5> packed_xors;
            std::size_t key;
            char top_pancake;

            [[nodiscard]] inline constexpr friend auto operator==(packed_state const& lhs, packed_state const& rhs) noexcept -> bool
            {
                return lhs.key == rhs.key && lhs.top_pancake == rhs.top_pancake && std::equal(std::execution::unseq, lhs.packed_xors.cbegin(), lhs.packed_xors.cend(), rhs.packed_xors.cbegin());
            }

            [[nodiscard]] inline constexpr friend auto hash_value(packed_state const& val) noexcept -> std::size_t
            {
                return val.key;
            }
        };

        constexpr pancake_48(std::array<char, N> starting_order) noexcept :
            start_order(std::move(starting_order))
        {}

        [[nodiscard]] constexpr auto initial_state() const noexcept -> state
        {
            state s{};

            std::ranges::copy(start_order, s.pancakes.begin());
            s.pancakes[N] = N;
            s.top_pancake = s.pancakes[0];

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
            // std::vector<action> actions;
            // actions.reserve(N - 1);

            // for(short i = 1; i < N; ++i)
            // {
            //     if(i == generating_action.flip_index) continue;

            //     // actions.emplace_back(std::max(s.pancakes[0] + 1, s.pancakes[i] + 1), i);
            //     actions.emplace_back(1, i);
            // }

            // return actions;
            static constexpr auto actions = [](){
                std::array<action, N - 1> arr;
                std::ranges::generate(arr, [i = 0]() mutable {
                    ++i;
                    return action{ static_cast<char>(i) };
                });

                return arr;
            }();

            return actions | std::views::filter([=](action const& a) -> bool {
                return a.flip_index != generating_action.flip_index;
            });
        }

        [[nodiscard]] constexpr auto apply(action const a, state const& s) const noexcept -> std::optional<packed_state>
        {
            // using the xors keeps us from needing to use std::reverse(...)
            // auto const build_return = [&](){
            //     packed_state ps{s.xors, 0, s.pancakes[a.flip_index]};

            //     ps.xors[s.pancakes[0]] ^= s.pancakes[a.flip_index + 1];
            //     ps.xors[s.pancakes[a.flip_index]] ^= s.pancakes[a.flip_index + 1];
            //     ps.xors[s.pancakes[a.flip_index + 1]] ^= s.pancakes[a.flip_index] ^ s.pancakes[0];

            //     for(auto x : ps.xors)
            //     {
            //         ps.key = phmap::HashState().combine(ps.key, x);
            //     }

            //     return ps;
            // };

            // return std::optional{ build_return() }; // use lambda to ensure the std::optional won't copy-construct the packed_state

            // std::size_t const clear_mask = static_cast<std::size_t>(s.tiles[s.blank_index + a.blank_offset]) << (PuzzleType::num_tiles - (s.blank_index + a.blank_offset) - 1) * 4;
            // std::size_t const insert_mask = static_cast<std::size_t>(s.tiles[s.blank_index + a.blank_offset]) << (PuzzleType::num_tiles - s.blank_index - 1) * 4;

            // return std::optional{ packed_state{ s.key ^ clear_mask ^ insert_mask } };

            

            // Compute locations in the packed_xor data, modify packed data, update key

            // Trickiest parts:
            // use index to compute which size_t the xor belongs to.
            // int index = first_index * 0.1f; // equivalent to divide by 10, avoids integer div. Truncates to index because 10 vals per size_t, except last which is handled later.

            // Determine specific bit offset in the size_t
            // int shift_distance = ((index + 1) * 10 - first_index - 1) * 6; // (Max element - this element - 1) * bits_per_element

            // replace those bits with the new xor value. Requires clearing them then setting them. Can clear by xoring with old value.
            // only need to perform the shift-over once because of xor self-inverse properties
            // ps.packed_xors[index] ^= (first_new_val << shift_distance); // first_new_val should be the actual computed mask, which is first_old_val ^ first_new_val

            // auto const build_return = [&](){
            //     packed_state ps{s.packed_xors, 0, s.pancakes[a.flip_index]};

            //     int const first_index = s.pancakes[0] * 0.1f;
            //     int const first_shift_distance = ((first_index + 1) * 10 - s.pancakes[0] - 1) * 6 - 12 * (first_index == 4); // should handle offset for last 8 values
            //     ps.packed_xors[first_index] ^= static_cast<std::size_t>(s.pancakes[a.flip_index + 1]) << first_shift_distance;

            //     int const second_index = s.pancakes[a.flip_index] * 0.1f;
            //     int const second_shift_distance = ((second_index + 1) * 10 - s.pancakes[a.flip_index] - 1) * 6 - 12 * (second_index == 4);
            //     ps.packed_xors[second_index] ^= static_cast<std::size_t>(s.pancakes[a.flip_index + 1]) << second_shift_distance;

            //     // "Plate" info isn't stored in packed data
            //     if(a.flip_index + 1 < N)
            //     {
            //         int const third_index = s.pancakes[a.flip_index + 1] * 0.1f;
            //         int const third_shift_distance = ((third_index + 1) * 10 - s.pancakes[a.flip_index + 1] - 1) * 6 - 12 * (third_index == 4);
            //         ps.packed_xors[third_index] ^= static_cast<std::size_t>(s.pancakes[a.flip_index] ^ s.pancakes[0]) << third_shift_distance;
            //     }

            //     // Update key. Should be plenty fast.
            //     ps.key = ps.packed_xors[0] ^ ps.packed_xors[1] ^ ps.packed_xors[2] ^ ps.packed_xors[3] ^ ps.packed_xors[4];

            //     return ps;
            // };

            // return std::optional{ build_return() }; // build ps with lambda to ensure std::optional doesn't copy construct

            // std::optional<packed_state> ps;
            // ps.emplace(s.packed_xors, 0, s.pancakes[a.flip_index]);

            // int const first_index = s.pancakes[0] * 0.1f;
            // int const first_shift_distance = ((first_index + 1) * 10 - s.pancakes[0] - 1) * 6 - 12 * (first_index == 4); // should handle offset for last 8 values
            // ps->packed_xors[first_index] ^= static_cast<std::size_t>(s.pancakes[a.flip_index + 1]) << first_shift_distance;

            // int const second_index = s.pancakes[a.flip_index] * 0.1f;
            // int const second_shift_distance = ((second_index + 1) * 10 - s.pancakes[a.flip_index] - 1) * 6 - 12 * (second_index == 4);
            // ps->packed_xors[second_index] ^= static_cast<std::size_t>(s.pancakes[a.flip_index + 1]) << second_shift_distance;

            // // "Plate" info isn't stored in packed data
            // if(a.flip_index + 1 < N)
            // {
            //     int const third_index = s.pancakes[a.flip_index + 1] * 0.1f;
            //     int const third_shift_distance = ((third_index + 1) * 10 - s.pancakes[a.flip_index + 1] - 1) * 6 - 12 * (third_index == 4);
            //     ps->packed_xors[third_index] ^= static_cast<std::size_t>(s.pancakes[a.flip_index] ^ s.pancakes[0]) << third_shift_distance;
            // }

            // // Update key. Should be plenty fast.
            // ps->key = ps->packed_xors[0] ^ ps->packed_xors[1] ^ ps->packed_xors[2] ^ ps->packed_xors[3] ^ ps->packed_xors[4];

            // return ps;

            std::optional<packed_state> ps;
            ps.emplace(s.packed_xors, 0, s.pancakes[a.flip_index]);

            int const first_index = s.pancakes[0] * 0.1f;
            std::size_t const first_val = s.pancakes[a.flip_index + 1];
            int const first_shift_distance = 60 * first_index + 54 - 6 * s.pancakes[0] - (first_index == 4 ? 12 : 0); // should handle offset for last 8 values
            ps->packed_xors[first_index] ^= first_val << first_shift_distance;

            int const second_index = s.pancakes[a.flip_index] * 0.1f;
            std::size_t const second_val = s.pancakes[a.flip_index + 1];
            int const second_shift_distance = 60 * second_index + 54 - 6 * s.pancakes[a.flip_index] - (second_index == 4 ? 12 : 0);
            ps->packed_xors[second_index] ^= second_val << second_shift_distance;

            // "Plate" info isn't stored in packed data
            [[likely]]
            if(a.flip_index + 1 < N)
            {
                int const third_index = s.pancakes[a.flip_index + 1] * 0.1f;
                std::size_t const third_val = s.pancakes[a.flip_index] ^ s.pancakes[0];
                int const third_shift_distance = 60 * third_index + 54 - 6 * s.pancakes[a.flip_index + 1] - (third_index == 4 ? 12 : 0);
                ps->packed_xors[third_index] ^= third_val << third_shift_distance;
            }

            // Update key. Should be plenty fast.
            ps->key = ps->packed_xors[0] ^ ps->packed_xors[1] ^ ps->packed_xors[2] ^ ps->packed_xors[3] ^ ps->packed_xors[4];

            return ps;
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
            // packed_state ps{s.xors, 0, s.top_pancake};
            
            // for(auto x : ps.xors)
            // {
            //     ps.key = phmap::HashState().combine(ps.key, x);
            // }

            // return ps;
            packed_state ps{};
            ps.top_pancake = s.top_pancake;

            constexpr int shift_distance = 6; // 6 bits per xor

            // First 40 vals
            for(int written = 0, index = 0; written < 40; written += 10, ++index)
            {
                // 10 values per size_t
                std::size_t packed = 0;
                for(int i = 0; i < 10; ++i)
                {
                    packed <<= shift_distance;
                    packed |= s.xors[written + i];
                }

                ps.packed_xors[index] = packed;
            }

            // Last 8 vals
            std::size_t packed = 0;
            for(int i = 0; i < 8; ++i)
            {
                packed <<= shift_distance;
                packed |= s.xors[40 + i];
            }
            ps.packed_xors[4] = packed;

            ps.key = ps.packed_xors[0] ^ ps.packed_xors[1] ^ ps.packed_xors[2] ^ ps.packed_xors[3] ^ ps.packed_xors[4];

            return ps;
        }

        [[nodiscard]] inline constexpr auto unpack(packed_state const& ps) const noexcept -> state
        {
            // state s{{}, {}, ps.xors, ps.top_pancake};

            // // Use the xors to extract the pancake positions
            // s.pancakes[0] = ps.top_pancake;
            // s.pancakes[1] = s.xors[s.pancakes[0]];

            // for(auto i = 2; i < N; ++i)
            // {
            //     s.pancakes[i] = s.xors[s.pancakes[i-1]] ^ s.pancakes[i-2];
            // }
            // s.pancakes[N] = N;
            // s.xors[N] = s.pancakes[N - 1];

            // return s;

            state s{ps.packed_xors, {}, {}, ps.top_pancake};
            
            // Unpack the packed xor data
            constexpr std::size_t bit_mask = 0x3F; // 6 LSBs
            constexpr int shift_distance = 6;
            // Last 8
            std::size_t packed = ps.packed_xors[4];
            for(int i = 47; i >= 40; --i)
            {
                s.xors[i] = packed & bit_mask;
                packed >>= shift_distance;
            }

            // First 40
            for(int index = 3; index >= 0; --index)
            {
                std::size_t packed = ps.packed_xors[index];

                for(int i = 9; i >= 0; --i)
                {
                    s.xors[10 * index + i] = packed & bit_mask;
                    packed >>= shift_distance;
                }
            }

            // Use the xors to extract the pancake positions
            s.pancakes[0] = ps.top_pancake;
            s.pancakes[1] = s.xors[s.pancakes[0]];

            for(auto i = 2; i < N; ++i)
            {
                s.pancakes[i] = s.xors[s.pancakes[i-1]] ^ s.pancakes[i-2];
            }
            s.pancakes[N] = N;
            s.xors[N] = s.pancakes[N - 1];

            return s;
        }

        [[nodiscard]] constexpr auto heuristic_value(state const& s) const noexcept -> h_type
        {
            h_type h{};
            for(auto i = 1; i < N + 1; ++i)
            {
                h += std::abs(s.pancakes[i - 1] - s.pancakes[i]) > 1;
            }

            return h;
        }

        [[nodiscard]] constexpr inline auto heuristic_value(state const& s, action const a, h_type parent_h) const noexcept -> h_type
        {
            // apply(...) only updates xors and does not affect actual pancake order, so old_val is computed using the current pancake order
            // and new_val is computed using the order if the flip had actually been applied.

            h_type const old_val = std::abs(s.pancakes[a.flip_index] - s.pancakes[a.flip_index + 1]) > 1;
            h_type const new_val = std::abs(s.pancakes[0] - s.pancakes[a.flip_index + 1]) > 1;

            return parent_h + new_val - old_val;
        }

    private:
        std::array<char, N> start_order;
    };
}