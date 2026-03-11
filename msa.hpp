#pragma once

#include <array>
#include <vector>
#include <optional>
#include <algorithm>
#include <ranges>
#include <cstdlib>
#include <cmath>

#include <utils/parallel_hashmap/phmap.h>

namespace search::domains
{
    template<std::size_t num_sequences>
    class msa
    {
    public:
        using g_type = int;
        using h_type = int;
        inline static constexpr h_type max_heuristic_delta = 1000; // placeholder used to prevent early pruning

        struct action
        {
            std::array<char, num_sequences> offset;
            g_type c;

            [[nodiscard]] inline constexpr auto cost() const noexcept -> g_type
            {
                return c;
            }
        };

        struct state
        {
            std::array<short, num_sequences> coordinate;
            std::size_t key;

            [[nodiscard]] inline constexpr friend auto operator==(state const& lhs, state const& rhs) noexcept -> bool
            {
                return lhs.key == rhs.key && std::ranges::equal(lhs.coordinate, rhs.coordinate);
            }

            [[nodiscard]] inline constexpr friend auto hash_value(state const& val) noexcept -> std::size_t
            {
                return val.key;
            }
        };

        using packed_state = state;

        msa(std::array<std::vector<char>, num_sequences> seqs) noexcept :
            sequences(std::move(seqs)),
            pairwise_alignments(precompute_pairwise_alignments()),
            start_coord{}
        {
            for(auto i = 0; i < num_sequences; ++i) goal_coord[i] = sequences[i].size() - 1;
        }

        [[nodiscard]] auto initial_state() const noexcept -> state
        {
            return state{start_coord, 0};
        }

        [[nodiscard]] inline auto actions(state const& s, action const& generating_action) const noexcept
        {
            static int max_actions = std::pow(2, num_sequences) - 1;
            std::vector<action> actions;
            actions.reserve(max_actions);

            std::array<char, num_sequences> offset{};

            auto const update_offset = [&](){
                for(auto i = 0; i < offset.size(); ++i)
                {
                    if(offset[i] == 0)
                    {
                        offset[i] = 1;
                        return;
                    }

                    offset[i] = 0;
                }
            };

            auto const is_valid_offset = [&](){
                for(auto i = 0; i < offset.size(); ++i)
                {
                    if(offset[i] && s.coordinate[i] == sequences[i].size() - 1) return false;
                }
                return true;
            };

            for(auto i = 0; i < max_actions; ++i)
            {
                update_offset();
                if(is_valid_offset()) actions.emplace_back(offset, compute_action_cost(s, offset));
            }

            return actions;
        }


        [[nodiscard]] inline constexpr auto apply(action const& a, state const& s) const noexcept -> std::optional<packed_state>
        {
            auto const build_successor = [&](){
                state successor{};

                // std::ranges::transform(s.coordinate, a.offset, std::back_inserter(successor.coordinate), std::plus<>{});

                auto view = std::views::zip_transform(std::plus<>{}, s.coordinate, a.offset);

                auto i = 0;
                for(auto c : view)
                {
                    successor.coordinate[i] = c;
                    successor.key = phmap::HashState().combine(successor.key, c);
                    ++i;
                }

                return successor;
            };

            return std::optional{build_successor()};
        }

        inline void undo(action const& a, state const& s) const noexcept
        {
            // apply(...) doesn't modifiy s, so nothing to undo
        }

        [[nodiscard]] inline auto pack(state s) const noexcept -> packed_state
        {
            s.key = 0;
            for(auto c : s.coordinate)
            {
                s.key = phmap::HashState().combine(s.key, c);
            }

            return s;
        }

        [[nodiscard]] auto unpack(packed_state ps) const noexcept -> state
        {
            return ps;
        }

        [[nodiscard]] inline auto heuristic_value(state const& s) const noexcept -> h_type
        {
            h_type sum{};

            for(auto i = 0; i < s.coordinate.size(); ++i)
            {
                for(auto j = i + 1; j < s.coordinate.size(); ++j)
                {
                    auto const& alignment = pairwise_alignments[i][j - i - 1]; // subtract (i + 1) because [i][i] and before do not exist.
                    sum += alignment[s.coordinate[i]][s.coordinate[j]];
                }
            }

            return sum;
        }

        [[nodiscard]] inline auto heuristic_value(state const& s, action const& a, char parent_h) const noexcept -> h_type
        {
            // Apply does not actually modify the state, so when computing the heuristic we must account for the change that the action should have made.

            h_type sum{};

            std::array<short, num_sequences> coordinate;

            // std::ranges::transform(s.coordinate, a.offset, coordinate.begin(), std::plus<>{});

            auto view = std::views::zip_transform(std::plus<>{}, s.coordinate, a.offset);

            auto i = 0;
            bool is_goal = true;

            for(auto c : view)
            {
                coordinate[i] = c;
                is_goal = is_goal && c == goal_coord[i];
                ++i;
            }

            if(is_goal)
            {
                return sum;
            }

            // if(std::ranges::equal(coordinate, goal_coord)) return sum;

            for(auto i = 0; i < coordinate.size(); ++i)
            {
                for(auto j = i + 1; j < coordinate.size(); ++j)
                {
                    auto const& alignment = pairwise_alignments[i][j - i - 1]; // subtract (i + 1) because [i][i] and before do not exist.
                    sum += alignment[coordinate[i]][coordinate[j]];
                }
            }

            return sum;
        }

    private:
        using pairwise_alignment_type = std::vector<std::vector<h_type>>;
        static constexpr g_type gap_cost = 1;
        static constexpr g_type mismatch_cost = 2;

        std::array<std::vector<char>, num_sequences> sequences;
        std::vector<std::vector<pairwise_alignment_type>> pairwise_alignments;
        std::array<short, num_sequences> start_coord, goal_coord;

        // static constexpr char ascii_offset = 0x41;
        //                                             //  A,  B, C, D, E, F, G, H, I,  J, K, L,  M,  N,  O,  P,  Q,  R,  S,  T,  U,  V,  W,  X,  Y,  Z
        // static constexpr auto index_lookup = std::array{0, -1, 1, 2, 3, 4, 5, 6, 7, -1, 8, 9, 10, 11, -1, 12, 13, 14, 15, 16, -1, 17, 18, -1, 19, -1};
        // inline static auto pam_250 = std::array{
        //     std::vector{-2, 2, 0, 0, 3, -1, 1, 1, 1, 2, 1, 0, -1, 0, 2, -1, -1, 0, 6, 3}, // A
        //     std::vector{-12, 5, 5, 4, 3, 3, 2, 5, 6, 5, 4, 3, 5, 4, 0, 2, 2, 8, 0}, // C
        //     std::vector{-4, -3, 6, -1, -1, 2, 0, 4, 3, -2, 1, -2, 1, 0, 0, 2, 7, 4}, // D
        //     std::vector{-4, 5, 0, -1, 2, 0, 3, 2, -1, 1, -2, 1, 0, 0, 2, 7, 4}, // E
        //     std::vector{-9, 5, 2, -1, 5, -2, 0, 3, 5, 5, 4, 3, 3, 1, 0, -7}, // F
        //     std::vector{-5, 2, 3, 2, 4, 3, 0, 0, 1, 3, -1, 0, 1, 7, 5}, // G
        //     std::vector{-6, 2, 0, 2, 2, -2, 0, -3, -2, 1, 1, 2, 3, 0}, // H
        //     std::vector{-5, 2, -2, -2, 2, 2, 2, 2, 1, 0, -4, 5, 1}, // I
        //     std::vector{-5, 3, 0, -1, 1, -1, -3, 0, 0, 2, 3, 4}, // K
        //     std::vector{-6, -4, 3, 3, 2, 3, 3, 2, -2, 2, 1}, // L
        //     std::vector{-6, 2, 2, 1, 0, 2, 1, -2, 4, 2}, // M
        //     std::vector{-2, 0, -1, 0, -1, 0, 2, 4, 2}, // N
        //     std::vector{-6, 0, 0, -1, 0, 1, 6, 5}, // P
        //     std::vector{-4, -1, 1, 1, 2, 5, 4}, // Q
        //     std::vector{-6, 0, 1, 2, -2, 4}, // R
        //     std::vector{-2, -1, 1, 2, 3}, // S
        //     std::vector{-3, 0, 5, 3}, // T
        //     std::vector{-4, 6, 2}, // V
        //     std::vector{-17, 0}, // W
        //     std::vector{-10}, // Y
        // };

        [[nodiscard]] h_type inline pairwise_cost(char a, char b) const noexcept
        {
            // if(b < a) std::swap(a, b);

            // auto i = index_lookup[a - ascii_offset];
            // auto j = index_lookup[b - ascii_offset] - i;

            // return pam_250[i][j];
            
            return (a != b) * mismatch_cost;
        }

        // Compute pairwise alignments using the Needleman-Wunsch scoring
        [[nodiscard]] auto compute_pairwise_alignment(std::vector<char> const& seq1, std::vector<char> const& seq2) const -> pairwise_alignment_type
        {
            // initialize this alignment to all zeroes
            std::vector<std::vector<h_type>> alignment(seq1.size(), std::vector<h_type>(seq2.size(), 0));

            // Alignment is performed in reverse from the standard so that it can be seamlessly used for heuristic estimates.
            // Initialize costs for when one sequence is empty
            for(int j = seq2.size() - 1; j >= 1; --j)
            {
                alignment[seq1.size() - 1][j - 1] = alignment[seq1.size() - 1][j] + 1;
            }

            for(int i = seq1.size() - 1; i >= 1; --i)
            {
                alignment[i - 1][seq2.size() - 1] = alignment[i][seq2.size() - 1] + 1;
            }
            // End cost initialization

            // Compute pairwise alignment via dynamic programming
            for(int i = seq1.size() - 2; i >= 0; --i)
            {
                for(int j = seq2.size() - 2; j >= 0; --j)
                {
                    auto const m1 = alignment[i + 1][j + 1] + pairwise_cost(seq1[i + 1], seq2[j + 1]);
                    auto const m2 = alignment[i + 1][j] + gap_cost;
                    auto const m3 = alignment[i][j + 1] + gap_cost;

                    alignment[i][j] = std::min({m1, m2, m3});
                }
            }

            return alignment;
        }

        [[nodiscard]] auto precompute_pairwise_alignments() const -> std::vector<std::vector<pairwise_alignment_type>>
        {
            std::vector<std::vector<pairwise_alignment_type>> alignments(sequences.size());

            for(auto i = 0; i < sequences.size(); ++i)
            {
                for(auto j = i + 1; j < sequences.size(); ++j)
                {
                    alignments[i].push_back(compute_pairwise_alignment(sequences[i], sequences[j]));
                }
            }

            return alignments;
        }

        [[nodiscard]] auto compute_action_cost(state const& s, std::array<char, num_sequences> const& offset) const noexcept -> g_type
        {
            // Compute the sum of pairwise costs
            g_type cost{};
            g_type gaps{};
            for(auto i = 0; i < offset.size(); ++i)
            {
                for(auto j = i + 1; j < offset.size(); ++j)
                {
                    if(offset[i] == 1 && offset[j] == 1) cost += pairwise_cost(sequences[i][s.coordinate[i] + 1], sequences[j][s.coordinate[j] + 1]);
                    else gaps += offset[i] | offset[j]; // Gap was inserted, so don't compare characters, just increase gap count
                }
            }

            return cost + gaps * gap_cost;
        }
    };
}