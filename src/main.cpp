#include "binary_heap.h"
#include "a_star.h"
#include "environments/environments.h"

int main() {

  GridEnvironment env(10, 10);
  BinaryHeap heap(env.get_pool());
  AStar<GridEnvironment, BinaryHeap> solver(env, heap);
  solver.solve();

  return 0;
}
