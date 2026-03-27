#pragma once

#include <array>
#include <vector>
#include <queue>
#include <optional>
#include <algorithm>
#include <ranges>
#include <numbers>
#include <cstdlib>
#include <cmath>
#include <utils/time_op.hpp>
#include <chrono>
#include <iostream>
#include <execution>

namespace search::domains
{
    struct DoF6
    {
        inline static constexpr char num_joints = 6;
    };

    struct DoF20
    {
        inline static constexpr char num_joints = 20;
    };

    template<typename DoF, bool uniform_cost>
    class robot_arm
    {
    public:

        template<typename dof_type = DoF>
        struct packed_state_template;

        template<>
        struct packed_state_template<DoF6>
        {
            std::size_t coords;

            [[nodiscard]] inline constexpr friend auto operator==(packed_state_template<DoF6> const& lhs, packed_state_template<DoF6> const& rhs) noexcept -> bool
            {
                return lhs.coords == rhs.coords;
            }
        };

        template<>
        struct packed_state_template<DoF20>
        {
            // Break into 3, 4-byte numbers to improve packing/alignment
            unsigned int first6Coords; // Each of these coords is 5 bits. 30 bits used, 2 MSBs unused.
            unsigned int next7Coords; // The first 4 of these are 5 bits, the next 3 are 4 bits. No bits unused.
            unsigned int last7Coords; // Each of these coords is 4 bits. 28 bits used, 4 MSBs unused.

            [[nodiscard]] inline constexpr friend auto operator==(packed_state_template<DoF20> const& lhs, packed_state_template<DoF20> const& rhs) noexcept -> bool
            {
                return (lhs.first6Coords == rhs.first6Coords) && (lhs.next7Coords == rhs.next7Coords) && (lhs.last7Coords == rhs.last7Coords);
            }
        };
        
        using g_type = short;
        using h_type = short;
        inline static constexpr h_type max_heuristic_delta = 1;

        struct action
        {
            char joint_index{0};
            char direction{0};

            [[nodiscard]] inline constexpr auto cost() const noexcept -> char
            {
                if constexpr(uniform_cost)
                {
                    return 1;
                }
                else
                {
                    // return joint_index + 1;
                    return DoF::num_joints - joint_index;
                }
            }
        };

        using packed_state = packed_state_template<DoF>;

        struct state
        {
            struct index_info
            {
                //float angle;
                //float sin;
                //float cos;
                float x;
                float y;
                char coordinate;
            };

            std::array<index_info, DoF::num_joints> info;
            packed_state packed;
            float effector_x, effector_y;
        };

        constexpr robot_arm(std::vector<char> environment, std::pair<float, float> environment_dimensions, std::pair<float, float> cell_scale_factors, int arm_base_x, std::array<float, DoF::num_joints> arm_segment_lengths,
                            std::array<float, DoF::num_joints> initial_joint_angles, std::pair<int, int> goal_xy)
            :
            environment(std::move(environment)),            
            heuristic_lookup(compute_heuristic_lookup(goal_xy, environment_dimensions)),
            segment_lengths(std::move(arm_segment_lengths)),
            env_dims(environment_dimensions),
            cell_dims(env_dims.first / cell_scale_factors.first, env_dims.second / cell_scale_factors.second),
            inv_cell_dims(1.0f / cell_dims.first, 1.0f / cell_dims.second),
            base_x(arm_base_x)
        {
            std::ranges::transform(initial_joint_angles, initial_angles.begin(), [](float const angle_in_degrees){
                return angle_in_degrees * std::numbers::pi_v<float> / 180.0f;
            });

            std::ranges::transform(segment_lengths, joint_angle_deltas.begin(), [&](float const segment_length){
                return 2.0f * std::asin(cell_dims.first * 0.5f / segment_length);
            });

            std::ranges::transform(joint_angle_deltas, joint_angle_limits.begin(), [](float const delta){
                return std::ceil(2.0f * std::numbers::pi_v<float> / delta);
                // return 2.0f * std::numbers::pi_v<float> / delta + 0.999999f;
            });

            std::ranges::for_each(sin_cos_lookup, [this, i = 0](auto& vec) mutable {
                vec.reserve(joint_angle_limits[i]);
                
                std::ranges::generate_n(std::back_inserter(vec), joint_angle_limits[i], [&, j = 0]() mutable {
                    float const angle = j * joint_angle_deltas[i];
                    std::pair<float, float> p{ std::sin(angle) * segment_lengths[i], std::cos(angle) * segment_lengths[i] };

                    ++j;
                    return p;
                });

                ++i;
            });
        }

        [[nodiscard]] constexpr auto initial_state() const noexcept -> state
        {
            state initial;
            auto [x, y] = cell_to_continuous(base_x, env_dims.second - 1);

            std::ranges::generate(initial.info, [this, i = 0, x = x, y = y]() mutable -> typename state::index_info {
                char const coord = (initial_angles[i] + joint_angle_deltas[i] * 0.5f) / joint_angle_deltas[i];
                // float const angle = coord * joint_angle_deltas[i];
                // float const sin = std::sin(angle);
                // float const cos = std::cos(angle);
                float const x_ = x;
                float const y_ = y;

                // x += segment_lengths[i] * cos;
                // y -= segment_lengths[i] * sin;
                x += sin_cos_lookup[i][coord].second; //segment_lengths[i] * sin_cos_lookup[i][coord].second;
                y -= sin_cos_lookup[i][coord].first; //segment_lengths[i] * sin_cos_lookup[i][coord].first;
                ++i;
                return {x_, y_, coord};//{angle, sin, cos, x_, y_, coord};
            });

            return initial;
        }

        [[nodiscard]] inline constexpr auto actions(state const& s, action const generating_action) const noexcept
        {
            static constexpr auto actions = [](){
                std::array<action, 2 * DoF::num_joints> arr;
                std::ranges::generate(arr, [i = 0, d = -1]() mutable {
                    action a { static_cast<char>(i), static_cast<char>(d) };
                    d *= -1;
                    if(d == -1) ++i;

                    return a;
                });

                return arr;
            }();

            return actions | std::views::filter([=](action const& a) -> bool {
                return a.joint_index != generating_action.joint_index || a.direction != -generating_action.direction;
            });
        }

        template<typename DoFType = DoF>
        [[nodiscard]] constexpr auto apply(action const a, state& s) const noexcept -> std::optional<packed_state>
        {
            s.info[a.joint_index].coordinate = (s.info[a.joint_index].coordinate + joint_angle_limits[a.joint_index] + a.direction) % joint_angle_limits[a.joint_index];

            if(!is_valid(a.joint_index, s)) return std::nullopt;

            auto const l_shift = (DoF::num_joints - a.joint_index - 1) << 3;
            std::size_t const mask = ~(0xFFul << l_shift);

            return std::optional<packed_state>{ (s.packed.coords & mask) | (static_cast<std::size_t>(s.info[a.joint_index].coordinate) << l_shift) };
        }

        template<>
        [[nodiscard]] constexpr auto apply<DoF20>(action const a, state& s) const noexcept -> std::optional<packed_state>
        {
            s.info[a.joint_index].coordinate = (s.info[a.joint_index].coordinate + joint_angle_limits[a.joint_index] + a.direction) % joint_angle_limits[a.joint_index];

            if(!is_valid(a.joint_index, s)) return std::nullopt;

            return pack(s);
        }

        inline constexpr void undo(action const a, state& s) const noexcept
        {
            s.info[a.joint_index].coordinate = (s.info[a.joint_index].coordinate + joint_angle_limits[a.joint_index] - a.direction) % joint_angle_limits[a.joint_index];
        }

        template<typename DoFType = DoF>
        [[nodiscard]] constexpr auto pack(state const& s) const noexcept -> packed_state
        {
            packed_state ps{};

            for(auto const& info : s.info)
            {
                ps.coords <<= 8;
                ps.coords |= info.coordinate;
            }

            return ps;
        }

        template<>
        [[nodiscard]] constexpr auto pack<DoF20>(state const& s) const noexcept -> packed_state
        {
            packed_state ps{};

            // Pack the first 6 coordinates
            for(auto i = 0; i < 6; ++i)
            {
                ps.first6Coords <<= 5;
                ps.first6Coords |= s.info[i].coordinate;
            }

            // Pack the 4 remaining 5-bit coordinates
            for(auto i = 6; i < 10; ++i)
            {
                ps.next7Coords <<= 5;
                ps.next7Coords |= s.info[i].coordinate;
            }

            // Pack the next 3, 4-bit coordinates
            for(auto i = 10; i < 13; ++i)
            {
                ps.next7Coords <<= 4;
                ps.next7Coords |= s.info[i].coordinate;
            }

            // Pack the remaining 7 coordinates into 32 bits
            for(auto i = 13; i < DoF::num_joints; ++i)
            {
                ps.last7Coords <<= 4;
                ps.last7Coords |= s.info[i].coordinate;
            }

            return ps;
        }

        template<typename DoFType = DoF>
        [[nodiscard]] constexpr auto unpack(packed_state ps) const noexcept -> state
        {
            static constexpr std::size_t bit_mask = 0xFFul << 56;
            
            state s;
            s.packed = ps;

            ps.coords <<= 16; // 16 MSBs aren't used

            auto [x, y] = cell_to_continuous(base_x, env_dims.second - 1);
            std::ranges::generate(s.info, [&, this, i = 0, x = x, y = y]() mutable -> typename state::index_info {
                char const coord = (ps.coords & bit_mask) >> 56;
                float x_ = x;
                float y_ = y;

                x += sin_cos_lookup[i][coord].second;
                y -= sin_cos_lookup[i][coord].first;
                ++i;
                ps.coords <<= 8;

                return {x_, y_, coord};
            });

            return s;
        }

        template<>
        [[nodiscard]] constexpr auto unpack<DoF20>(packed_state ps) const noexcept -> state
        {
            static constexpr std::size_t five_bit_mask = 0x1Ful << 27;
            static constexpr std::size_t four_bit_mask = 0x0Ful << 28;

            auto const get_generator = [this](auto& packed_data, auto const& bit_mask, auto r_shift, auto l_shift, auto current_index, auto& current_x, auto& current_y){
                return [&, i = current_index, l_shift = l_shift, r_shift = r_shift]() mutable -> typename state::index_info {
                    char const coord = (packed_data & bit_mask) >> r_shift;

                    float const x = current_x;
                    float const y = current_y;

                    current_x += sin_cos_lookup[i][coord].second; //segment_lengths[i] * sin_cos_lookup[i][coord].second;
                    current_y -= sin_cos_lookup[i][coord].first; //segment_lengths[i] * sin_cos_lookup[i][coord].first;
                    ++i;
                    packed_data <<= l_shift;

                    return {x, y, coord};//{angle, sin, cos, x, y, coord};
                };
            };

            ps.first6Coords <<= 2; // 2 MSBs unused
            ps.last7Coords <<= 4; // 4 MSBs unused

            auto [x, y] = cell_to_continuous(base_x, env_dims.second - 1);

            state s;
            s.packed = ps;

            std::ranges::generate_n(s.info.begin(), 6, get_generator(ps.first6Coords, five_bit_mask, 27, 5, 0, x, y));
            std::ranges::generate_n(std::next(s.info.begin(), 6), 4, get_generator(ps.next7Coords, five_bit_mask, 27, 5, 6, x, y));
            std::ranges::generate_n(std::next(s.info.begin(), 10), 3, get_generator(ps.next7Coords, four_bit_mask, 28, 4, 10, x, y));
            std::ranges::generate_n(std::next(s.info.begin(), 13), 7, get_generator(ps.last7Coords, four_bit_mask, 28, 4, 13, x, y));

            return s;
        }

        [[nodiscard]] constexpr inline auto heuristic_value(state const& s) const noexcept -> h_type
        {
            // auto const effector_x = s.info[DoF::num_joints - 1].x + s.info[DoF::num_joints - 1].cos * segment_lengths[DoF::num_joints - 1];
            // auto const effector_y = s.info[DoF::num_joints - 1].y + s.info[DoF::num_joints - 1].sin * segment_lengths[DoF::num_joints - 1];

            // auto const [x, y] = continuous_to_cell(effector_x, effector_y);

            // return heuristic_lookup[x + y * env_dims.first];

            auto [x, y] = cell_to_continuous(base_x, env_dims.second - 1);

            for(auto i = 0; i < DoF::num_joints; ++i)
            {
                // x += segment_lengths[i] * s.info[i].cos;
                // y -= segment_lengths[i] * s.info[i].sin;
                x += sin_cos_lookup[i][s.info[i].coordinate].second; //segment_lengths[i] * sin_cos_lookup[i][s.info[i].coordinate].second;
                y -= sin_cos_lookup[i][s.info[i].coordinate].first; //segment_lengths[i] * sin_cos_lookup[i][s.info[i].coordinate].first;
            }

            auto const [effector_x, effector_y] = continuous_to_cell(x, y);
            
            return heuristic_lookup[effector_x + effector_y * env_dims.first];
        }

        [[nodiscard]] constexpr inline auto heuristic_value(state const& s, action const& a, char parent_h) const noexcept -> h_type
        {
            // auto x = s.info[a.joint_index].x;
            // auto y = s.info[a.joint_index].y;

            // for(auto i = a.joint_index; i < DoF::num_joints; ++i)
            // {
            //     // x += segment_lengths[i] * s.info[i].cos;
            //     // y -= segment_lengths[i] * s.info[i].sin;
            //     x += segment_lengths[i] * sin_cos_lookup[i][s.info[i].coordinate].second;
            //     y -= segment_lengths[i] * sin_cos_lookup[i][s.info[i].coordinate].first;
            // }

            // auto const [effector_x, effector_y] = continuous_to_cell(x, y);
            
            // return heuristic_lookup[effector_x + effector_y * env_dims.first];

            auto const [x, y] = continuous_to_cell(s.effector_x, s.effector_y);
            return heuristic_lookup[x + y * env_dims.first];
        }

    private:
        std::vector<char> environment;
        std::vector<short> heuristic_lookup;
        std::array<float, DoF::num_joints> segment_lengths;
        std::array<float, DoF::num_joints> joint_angle_deltas;
        std::array<unsigned char, DoF::num_joints> joint_angle_limits;
        std::array<std::vector<std::pair<float, float>>, DoF::num_joints> sin_cos_lookup;
        std::array<float, DoF::num_joints> initial_angles;
        std::pair<float, float> env_dims;
        std::pair<float, float> cell_dims, inv_cell_dims;
        int base_x;

        [[nodiscard]] inline auto cell_to_continuous(int const x, int const y) const noexcept -> std::pair<float, float>
        {
            // return { x * cell_dims.first + 0.5f * cell_dims.first, y * cell_dims.second + 0.5f * cell_dims.second };
            return { x + 0.5f, y + 0.5f };
        }

        [[nodiscard]] inline auto continuous_to_cell(float const x, float const y) const noexcept -> std::pair<int, int>
        {
            // std::pair<int, int> p = { x * inv_cell_dims.first, y * inv_cell_dims.second };
            // if(x < 0) cellX = 0;
            // if(x >= envWidth) cellX = envWidth - 1;

            // if(y < 0) cellY = 0;
            // if(y >= envHeight) y = envHeight - 1;
            // return p;
            return { x, y };
        }

        [[nodiscard]] bool is_valid(int start_index, state& s) const noexcept
        {
            // float x0 = s.info[start_index].x;
            // float y0 = s.info[start_index].y;

            // for(auto i = start_index; i < DoF::num_joints; ++i)
            // {
            //     // float const x1 = x0 + segment_lengths[i] * s.info[i].cos;
            //     // float const y1 = y0 - segment_lengths[i] * s.info[i].sin;
            //     float const x1 = x0 + sin_cos_lookup[i][s.info[i].coordinate].second; //segment_lengths[i] * sin_cos_lookup[i][s.info[i].coordinate].second;
            //     float const y1 = y0 - sin_cos_lookup[i][s.info[i].coordinate].first; //segment_lengths[i] * sin_cos_lookup[i][s.info[i].coordinate].first;

            //     if(!is_valid_line_segment(x0, y0, x1, y1)) return false;

            //     x0 = x1;
            //     y0 = y1;
            // }

            // s.effector_x = x0;
            // s.effector_y = y0;

            // return true;

            auto const line_segments = [&](){
                std::vector<std::pair<std::pair<float, float>, std::pair<float, float>>> segments;
                segments.reserve(DoF::num_joints);

                float x0 = s.info[start_index].x;
                float y0 = s.info[start_index].y;

                for(auto i = start_index; i < DoF::num_joints; ++i)
                {
                    float const x1 = x0 + sin_cos_lookup[i][s.info[i].coordinate].second;
                    float const y1 = y0 - sin_cos_lookup[i][s.info[i].coordinate].first;

                    segments.emplace_back(std::pair{x0, y0}, std::pair{x1, y1});

                    x0 = x1;
                    y0 = y1;
                }

                s.effector_x = x0;
                s.effector_y = y0;

                return segments;
            }();

            return !std::any_of(std::execution::unseq, line_segments.cbegin(), line_segments.cend(), [&](auto const& coord_pairs){
                auto [x0, y0] = coord_pairs.first;
                auto [x1, y1] = coord_pairs.second;

                return is_valid_line_segment(x0, y0, x1, y1) == false;
            });
        }

        [[nodiscard]] bool is_valid_line_segment(float const x0, float const y0, float const x1, float const y1) const noexcept
        {
            if(x0 < 0 || x0 >= env_dims.first
            || x1 < 0 || x1 >= env_dims.first
            || y0 < 0 || y0 >= env_dims.second
            || y1 < 0 || y1 >= env_dims.second)
            {
                return false;
            }

            auto const in_obstacle_region = [](auto const x0, auto const y0, auto const x1, auto const y1){
                return (x0 <= 17 && y0 <= 21) || (x1 <= 17 && y1 <= 21);
            };

            auto [nX0, nY0] = continuous_to_cell(x0, y0);
            auto [nX1, nY1] = continuous_to_cell(x1, y1);

            if(!in_obstacle_region(nX0, nY0, nX1, nY1)) return true;

            if(environment[nX0 + nY0 * env_dims.first] == 1 || environment[nX1 + nY1 * env_dims.first] == 1) return false;

            // Use Bresenham's algorithm to walk along the line segment
            int dx = (int)nX1 - (int)nX0;
            int dy = (int)nY1 - (int)nY0;
            int absdx = std::abs(dx);
            int absdy = std::abs(dy);
            
            int x = nX0;
            int y = nY0;
            int xIndexScale = 1;
            int yIndexScale = env_dims.first;

            // Slope > 1
            if(absdy >= absdx)
            {
                std::swap(xIndexScale, yIndexScale);
                std::swap(absdx, absdy);
                std::swap(dx, dy);
                std::swap(x, y);
                std::swap(nX1, nY1);
                std::swap(nX0, nY0);
            }

            // int incX = 1;
            // int incY = 1;

            // if(dx < 0) incX = -1;
            // if(dy < 0) incY = -1;

            int const incX = (dx < 0) * -1 + (dx >= 0) * 1;
            int const incY = (dy < 0) * -1 + (dy >= 0) * 1;

            auto const slope = 2 * absdy;
            auto error = slope - absdx;

            do
            {
                // Check for obstacle
                if(environment[x * xIndexScale + y * yIndexScale] == 1)
                {
                    return false;
                }

                x += incX;

                error += slope;
                // if(error >= 0)
                // {
                //     y += incY;
                //     error -= 2 * absdx;
                // }

                y += (error >= 0) * incY;
                error -= (error >= 0) * 2 * absdx;
            }
            while(x != nX1);

            // // Check for obstacle
            // if(environment[x * xIndexScale + y * yIndexScale] == 1)
            // {
            //     return false;
            // }

            return environment[x * xIndexScale + y * yIndexScale] != 1;
        }

        [[nodiscard]] auto compute_heuristic_lookup(std::pair<int, int> goal_xy, std::pair<int, int> env_dims) const -> std::vector<short>
        {
            std::vector<short> table(environment.size(), std::numeric_limits<short>::max());

            table[goal_xy.first + goal_xy.second * env_dims.first] = 0;

            // Use BFS to compute the cost from the goal cell to all other cells
            std::queue<int> q;
            q.push(goal_xy.first + goal_xy.second * env_dims.first);

            while(!q.empty())
            {
                auto const index = q.front();
                q.pop();

                auto const x = index % static_cast<int>(env_dims.first);
                auto const y = index / static_cast<int>(env_dims.first);

                for(auto x_inc = -1; x_inc < 2; ++x_inc)
                {
                    for(auto y_inc = -1; y_inc < 2; ++y_inc)
                    {
                        if(x_inc == 0 && y_inc == 0) continue;

                        auto const new_x = x + x_inc;
                        auto new_y = y + y_inc;
                        auto const new_index = new_x + new_y * env_dims.first;

                        if(new_x < 0 || new_x >= env_dims.first || new_y < 0 || new_y >= env_dims.second
                        || environment[new_index] == 1) continue;

                        auto const new_g_value = table[index] + 1;

                        if(new_g_value < table[new_index])
                        {
                            table[new_index] = new_g_value;
                            q.push(new_index);
                        }
                    }
                }
            }

            return table;
        }
    };
}

template<>
struct std::hash<search::domains::robot_arm<search::domains::DoF6, true>::packed_state>
{
    [[nodiscard]] inline auto operator()(search::domains::robot_arm<search::domains::DoF6, true>::packed_state const& key) const noexcept -> std::size_t
    {
        return key.coords;
    }
};

template<>
struct std::hash<search::domains::robot_arm<search::domains::DoF6, false>::packed_state>
{
    [[nodiscard]] inline auto operator()(search::domains::robot_arm<search::domains::DoF6, false>::packed_state const& key) const noexcept -> std::size_t
    {
        auto x = key.coords;
        x = (x >> 48) ^ (x >> 32) ^ (x >> 16) ^ x;

        auto y = key.coords;
        y = (y << 48) ^ (y << 32) ^ (y << 16) ^ y;

        return x ^ y;
        // return key.coords;
    }
};

template<>
struct std::hash<search::domains::robot_arm<search::domains::DoF20, true>::packed_state>
{
    [[nodiscard]] inline auto operator()(search::domains::robot_arm<search::domains::DoF20, true>::packed_state const& key) const noexcept -> std::size_t
    {
        auto temp = static_cast<std::size_t>(key.first6Coords) << 34;  // 2 MSBs of first6Coords are unused, so shift them out
        temp |= static_cast<std::size_t>(key.last7Coords);
        return temp ^ (static_cast<std::size_t>(key.next7Coords) << 4);
    }
};

template<>
struct std::hash<search::domains::robot_arm<search::domains::DoF20, false>::packed_state>
{
    [[nodiscard]] inline auto operator()(search::domains::robot_arm<search::domains::DoF20, false>::packed_state const& key) const noexcept -> std::size_t
    {
        auto temp = static_cast<std::size_t>(key.first6Coords) << 34;  // 2 MSBs of first6Coords are unused, so shift them out
        temp |= static_cast<std::size_t>(key.last7Coords);
        return temp ^ (static_cast<std::size_t>(key.next7Coords) << 4);
    }
};