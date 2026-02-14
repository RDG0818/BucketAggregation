#pragma once

#include <array>
#include <cmath>
#include <optional>
#include <random>
#include <vector>
#include "external/parallel_hashmap/phmap.h"

namespace search::domains {

template<bool uniform_cost = true>
class GridEnvironment {

public:

using g_type = int;
using h_type = int;

enum class Direction { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3, NONE = 4 };

struct Action {
  char c = 1;

  Direction d{Direction::NONE};

  [[nodiscard]] inline constexpr auto cost() const noexcept -> char {
    return c;
  }
};

struct State {
  std::size_t key;
  uint32_t x, y, index;
};

struct PackedState {
  std::size_t key;

  [[nodiscard]] inline constexpr friend auto operator==(PackedState const& lhs, PackedState const& rhs) noexcept -> bool {
      return lhs.key == rhs.key;
  }

  [[nodiscard]] inline constexpr friend auto hash_value(PackedState const& val) noexcept -> std::size_t {
    return val.key;
  }
};

GridEnvironment();

[[nodiscard]] constexpr auto initial_state() const noexcept -> State {
  return State{
    (static_cast<std::size_t>(start_coor.first) << 32) | start_coor.second,
    start_coor.first,
    start_coor.second,
    start_coor.second * dimensions.first + start_coor.first 
  };
};

void actions();

template<bool u = uniform_cost>
[[nodiscard]] inline constexpr auto actions(State const& s, Action const generating_action) const noexcept {
    static constexpr auto actions = {action{1, direction::NORTH}, action{1, direction::SOUTH}, action{1, direction::EAST}, action{1, direction::WEST}};

    return actions | std::views::filter([=](auto const& a) -> bool{
        return ((a.d & 0x01) != (generating_action.d & 0x01)) || generating_action.d == direction::INVALID || a.d == generating_action.d;
    });
}

template<>
[[nodiscard]] auto actions<false>(state const& s, action const generating_action) const noexcept
{
  std::vector<action> actions;
  actions.reserve(4);

  for(auto i = 0; i < 4; ++i) {
    auto [x_offset, y_offset] = xy_offsets[i];
    auto const x = s.x + x_offset;
    auto const y = s.y + y_offset;

    if(x >= dimensions.first || x < 0
    || y >= dimensions.second || y < 0) continue;

    auto const index = y * dimensions.first + x;

    auto const cost = std::abs(grid[index] - grid[s.index]) + 1;

    actions.emplace_back(cost, static_cast<direction>(i));
  }

  return actions;
}

void apply();

[[nodiscard]] inline constexpr auto pack(State const& s) const noexcept -> PackedState {
  return PackedState{s.key};
};

[[nodiscard]] constexpr auto unpack(PackedState const& ps) const noexcept -> State {
  constexpr std::size_t bit_mask = (~0ul) >> 32;

  State s{};
  s.key = ps.key;
  s.y = ps.key & bit_mask;
  s.x = (ps.key >> 32) & bit_mask;
  s.index = s.y * dimensions.first + s.x;

  return s;
};

[[nodiscard]] inline constexpr auto heuristic_value(State const& s) const noexcept -> h_type {
    return (goal_coor.first- s.x) + (goal_coor.second - s.y); // goal_coor will always have max x and y
}



private:

  std::pair<uint32_t, uint32_t> dimensions, start_coor, goal_coor;
  inline static constexpr auto xy_offsets = std::array{std::pair{0, -1}, std::pair{1, 0}, std::pair{0, 1}, std::pair{-1, 0}};

};



}

