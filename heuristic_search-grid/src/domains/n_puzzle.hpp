#pragma once

#include <array>
#include <vector>
#include <optional>
#include <algorithm>
#include <ranges>
#include <cstdlib>

namespace search::domains
{
    struct puzzle_8
    {
        inline static constexpr unsigned char width = 3;
        inline static constexpr unsigned char num_tiles = 9;
    };

    struct puzzle_15
    {
        inline static constexpr unsigned char width = 4;
        inline static constexpr unsigned char num_tiles = 16;
    };

    template<typename PuzzleType>
    class n_puzzle
    {
    public:

        template<typename p_type = PuzzleType>
        struct packed_state_template;

        template<>
        struct packed_state_template<puzzle_8>
        {
            unsigned int first_8;
            unsigned char last;

            [[nodiscard]] inline constexpr friend auto operator==(packed_state_template<puzzle_8> const& lhs, packed_state_template<puzzle_8> const& rhs) noexcept -> bool
            {
                return lhs.first_8 == rhs.first_8 && lhs.last == rhs.last;
            }
        };

        template<>
        struct packed_state_template<puzzle_15>
        {
            std::size_t key;

            [[nodiscard]] inline constexpr friend auto operator==(packed_state_template<puzzle_15> const& lhs, packed_state_template<puzzle_15> const& rhs) noexcept -> bool
            {
                return lhs.key == rhs.key;
            }

            [[nodiscard]] inline constexpr friend auto hash_value(packed_state_template<puzzle_15> const& val) noexcept -> std::size_t
            {
                return val.key;
            }
        };
        
        using g_type = short;
        using h_type = char;
        inline static constexpr h_type max_heuristic_delta = 1;

        struct action
        {
            inline static constexpr char c = 1;

            char blank_offset{0};

            [[nodiscard]] inline constexpr auto cost() const noexcept -> char
            {
                return c;
            }
        };

        struct state
        {
            std::array<char, PuzzleType::num_tiles> tiles;
            std::size_t key;
            char blank_index;
        };

        using packed_state = packed_state_template<PuzzleType>;

        constexpr n_puzzle(std::array<char, PuzzleType::num_tiles> starting_tiles) noexcept :
            start_tiles(std::move(starting_tiles))
        {}

        [[nodiscard]] constexpr auto initial_state() const noexcept -> state
        {
            return state{
                start_tiles,
                0,
                static_cast<char>(std::distance(start_tiles.cbegin(), std::ranges::find(start_tiles, 0)))
            };
        }

        [[nodiscard]] inline constexpr auto actions(state const& s, action const& generating_action) const noexcept
        {
            // Return the possible actions for this blank position, filtering out the action that would generate
            // this state's parent
            return action_table[s.blank_index] | std::views::filter([=](action const a) -> bool{
                return a.blank_offset != -generating_action.blank_offset;
            });
        }

        [[nodiscard]] inline constexpr auto apply(action const& a, state const& s) const noexcept -> std::optional<packed_state>
        {
            // std::swap(s.tiles[s.blank_index], s.tiles[s.blank_index + a.blank_offset]);
            // s.blank_index += a.blank_offset;

            // // Not necessary to check state validity - only valid actions can be applied
            // return std::optional{ pack(s) };

            std::size_t const clear_mask = static_cast<std::size_t>(s.tiles[s.blank_index + a.blank_offset]) << (PuzzleType::num_tiles - (s.blank_index + a.blank_offset) - 1) * 4;
            std::size_t const insert_mask = static_cast<std::size_t>(s.tiles[s.blank_index + a.blank_offset]) << (PuzzleType::num_tiles - s.blank_index - 1) * 4;

            return std::optional{ packed_state{ s.key ^ clear_mask ^ insert_mask } };
        }

        // inline constexpr void undo(action const& a, state const& s) const noexcept
        // {
        //     // std::swap(s.tiles[s.blank_index], s.tiles[s.blank_index - a.blank_offset]);
        //     // s.blank_index -= a.blank_offset;
        // }
        void undo(action a, state s) const noexcept
        {}

        template<typename PType = PuzzleType>
        [[nodiscard]] constexpr auto pack(state const& s) const noexcept -> packed_state
        {
            packed_state ps{0,0};

            std::ranges::for_each_n(s.tiles.cbegin(), PuzzleType::num_tiles - 1, [&ps](auto const tile){
                ps.first_8 <<= 4;
                ps.first_8 |= tile;
            });

            ps.last = *std::rbegin(s.tiles);

            return ps;
        }

        template<>
        [[nodiscard]] constexpr auto pack<puzzle_15>(state const& s) const noexcept -> packed_state
        {
            packed_state ps{0};

            for(auto const tile : s.tiles)
            {
                ps.key <<= 4;
                ps.key |= tile;
            }

            return ps;
        }

        template<typename PType = PuzzleType>
        [[nodiscard]] constexpr auto unpack(packed_state const& ps) const noexcept -> state
        {
            state s;
            s.blank_index = 0;

            auto extract = [packed = ps.first_8]() mutable -> char{
                char tile = packed & 0x0F;
                packed >>= 4;
                return tile;
            };

            auto it = std::rbegin(s.tiles);
            *it = ps.last;
            ++it;
            s.blank_index += (PuzzleType::num_tiles - 1) * (ps.last == 0);

            std::for_each(it, std::rend(s.tiles), [&, i = PuzzleType::num_tiles - 2](auto& tile) mutable {
                tile = extract();
                s.blank_index += i * (tile == 0);
                --i;
            });

            return s;
        }

        template<>
        [[nodiscard]] constexpr auto unpack<puzzle_15>(packed_state const& ps) const noexcept -> state
        {
            state s;
            s.key = ps.key;
            s.blank_index = 0;

            auto extract = [packed = ps.key]() mutable -> char {
                char tile = packed & 0x0F;
                packed >>= 4;
                return tile;
            };

            std::for_each(std::rbegin(s.tiles), std::rend(s.tiles), [&, i = PuzzleType::num_tiles - 1](auto& tile) mutable{
                tile = extract();
                s.blank_index += i * (tile == 0);
                --i;
            });

            return s;
        }

        [[nodiscard]] constexpr auto heuristic_value(state const& s) const noexcept -> char
        {
            auto const manhattan_distance = [i = 0](char tile) mutable -> char{
                ++i;
                return md_table[tile][i - 1];
            };

            return std::ranges::fold_left(s.tiles | std::views::transform(manhattan_distance), 0, std::plus<char>{});
        }

        [[nodiscard]] inline constexpr auto heuristic_value(state const& s, action const& a, char parent_h) const noexcept -> char
        {
            // auto const moved_tile_loc = s.blank_index - a.blank_offset;
            // auto const old_moved_tile_h = md_table[s.tiles[moved_tile_loc]][s.blank_index];
            // auto const new_moved_tile_h = md_table[s.tiles[moved_tile_loc]][moved_tile_loc];

            // return parent_h - old_moved_tile_h + new_moved_tile_h;
            auto const moved_tile_loc = s.blank_index + a.blank_offset;

            auto const old_moved_tile_h = md_table[s.tiles[moved_tile_loc]][moved_tile_loc];
            auto const new_moved_tile_h = md_table[s.tiles[moved_tile_loc]][s.blank_index];

            return parent_h - old_moved_tile_h + new_moved_tile_h;
        }

    private:
        std::array<char, PuzzleType::num_tiles> start_tiles;

        inline static constexpr char up_offset = PuzzleType::width;
        inline static constexpr char down_offset = -PuzzleType::width;
        inline static constexpr char left_offset = 1;
        inline static constexpr char right_offset = -1;

        [[nodiscard]] static constexpr auto compute_md_table() noexcept -> std::array<std::array<char, PuzzleType::num_tiles>, PuzzleType::num_tiles>
        {
            std::array<std::array<char, PuzzleType::num_tiles>, PuzzleType::num_tiles> md_table;

            std::ranges::for_each(md_table, [tile = 0](std::array<char, PuzzleType::num_tiles>& table_row) mutable -> void{

                auto const target_row = tile / PuzzleType::width;
                auto const target_col = tile % PuzzleType::width;

                auto manhattan_distance = [=, index = 0]() mutable -> char{
                    auto const row = index / PuzzleType::width;
                    auto const col = index % PuzzleType::width;
                    ++index;

                    return (std::max(row - target_row, target_row - row) + std::max(col - target_col, target_col - col)) * (tile != 0);
                };

                std::ranges::generate(table_row, manhattan_distance);
                ++tile;
            });

            return md_table;
        }

        [[nodiscard]] static auto compute_action_table() -> std::array<std::vector<action>, PuzzleType::num_tiles>
        {
            static constexpr auto possible_actions = std::array{
                action{up_offset},
                action{down_offset},
                action{left_offset},
                action{right_offset}
            };

            std::array<std::vector<action>, PuzzleType::num_tiles> action_table;

            std::ranges::for_each(action_table, [blank_index = 0](std::vector<action>& actions) mutable -> void{
                auto const blank_row = blank_index / PuzzleType::width;
                auto const blank_col = blank_index % PuzzleType::width;

                auto const is_valid = [=](action const a) -> bool{
                    auto const new_index = blank_index + a.blank_offset;
                    auto const new_row = new_index / PuzzleType::width;
                    auto const new_col = new_index % PuzzleType::width;

                    // Must be a valid index, and either the row changed or the column changed - not both
                    return (0 <= new_index && new_index < PuzzleType::num_tiles) && !(new_row != blank_row && new_col != blank_col);
                };

                actions.reserve(possible_actions.size());
                std::ranges::copy_if(possible_actions, std::back_inserter(actions), is_valid);

                ++blank_index;
            });

            return action_table;
        }

        inline static constexpr std::array<std::array<char, PuzzleType::num_tiles>, PuzzleType::num_tiles> md_table = compute_md_table();
        inline static std::array<std::vector<action>, PuzzleType::num_tiles> action_table = compute_action_table();
    };
}

template<>
struct std::hash<search::domains::n_puzzle<search::domains::puzzle_8>::packed_state_template<search::domains::puzzle_8>>
{
    [[nodiscard]] inline constexpr auto operator()(search::domains::n_puzzle<search::domains::puzzle_8>::packed_state_template<search::domains::puzzle_8> const& key) const noexcept -> std::size_t
    {
        return (static_cast<std::size_t>(key.first_8) << 4) | key.last;
    }
};

template<>
struct std::hash<search::domains::n_puzzle<search::domains::puzzle_15>::packed_state_template<search::domains::puzzle_15>>
{
    [[nodiscard]] inline constexpr auto operator()(search::domains::n_puzzle<search::domains::puzzle_15>::packed_state_template<search::domains::puzzle_15> const& key) const noexcept -> std::size_t
    {
        return key.key;
    }
};