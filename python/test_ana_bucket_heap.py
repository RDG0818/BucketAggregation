import pytest
import math
from ana_bucket_heap import ANABucketHeap, SearchNode

# Helper function to calculate ANA* potential for verification in tests
def _calculate_ana_potential(g, h, C):
    if h == 0:
        return math.inf
    return (C - g) / h

def test_initialization():
    pq = ANABucketHeap()
    assert len(pq) == 0
    assert pq.incumbent_cost == math.inf

def test_push_and_len():
    pq = ANABucketHeap()
    pq.incumbent_cost = 100
    
    node1 = SearchNode("A", g_score=10, h_score=80) # f=90, u=1.125
    node2 = SearchNode("B", g_score=20, h_score=75) # f=95, u=1.066
    
    pq.push(node1)
    assert len(pq) == 1
    pq.push(node2)
    assert len(pq) == 2

def test_pop_order_initial():
    pq = ANABucketHeap()
    pq.incumbent_cost = 100
    
    nodeA = SearchNode("A", g_score=10, h_score=80) # f=90, u=1.125
    nodeB = SearchNode("B", g_score=20, h_score=75) # f=95, u=1.066
    nodeC = SearchNode("C", g_score=30, h_score=60) # f=90, u=1.166
    nodeD = SearchNode("D", g_score=5, h_score=85)  # f=90, u=1.117

    pq.push(nodeA)
    pq.push(nodeB)
    pq.push(nodeC)
    pq.push(nodeD)

    # Node C has the highest potential (1.166)
    popped = pq.pop()
    assert popped.state == "C"
    assert len(pq) == 3

    # Node A has the next highest potential (1.125)
    popped = pq.pop()
    assert popped.state == "A"
    assert len(pq) == 2

    # Node D has the next highest potential (1.117)
    popped = pq.pop()
    assert popped.state == "D"
    assert len(pq) == 1

    # Node B is last (1.066)
    popped = pq.pop()
    assert popped.state == "B"
    assert len(pq) == 0

def test_update_incumbent_reorders():
    pq = ANABucketHeap()
    pq.incumbent_cost = 100
    
    nodeA = SearchNode("A", g_score=10, h_score=80) # f=90, u=(100-10)/80 = 1.125
    nodeB = SearchNode("B", g_score=20, h_score=75) # f=95, u=(100-20)/75 = 1.066
    nodeC = SearchNode("C", g_score=30, h_score=60) # f=90, u=(100-30)/60 = 1.166

    pq.push(nodeA)
    pq.push(nodeB)
    pq.push(nodeC)

    # Initial pop order: C (1.166)
    popped = pq.pop()
    assert popped.state == "C"

    # Update incumbent cost, which should reorder priorities
    pq.update_incumbent(92)
    # New potentials:
    # A: u=(92-10)/80 = 1.025
    # B: u=(92-20)/75 = 0.96

    # Now, A should be popped next (1.025)
    popped = pq.pop()
    assert popped.state == "A"

    # Then B (0.96)
    popped = pq.pop()
    assert popped.state == "B"
    assert len(pq) == 0

def test_empty_queue():
    pq = ANABucketHeap()
    assert pq.pop() is None
    assert len(pq) == 0

def test_multiple_nodes_same_f_h():
    pq = ANABucketHeap()
    pq.incumbent_cost = 50
    
    node1 = SearchNode("X1", g_score=10, h_score=20) # f=30, u=(50-10)/20 = 2.0
    node2 = SearchNode("X2", g_score=10, h_score=20) # f=30, u=(50-10)/20 = 2.0
    node3 = SearchNode("Y", g_score=15, h_score=10)  # f=25, u=(50-15)/10 = 3.5

    pq.push(node1)
    pq.push(node2)
    pq.push(node3)

    # Y has highest potential
    popped = pq.pop()
    assert popped.state == "Y"
    assert len(pq) == 2

    # X1 and X2 have same potential, order doesn't matter for pop_anytime
    popped1 = pq.pop()
    popped2 = pq.pop()
    assert {popped1.state, popped2.state} == {"X1", "X2"}
    assert len(pq) == 0

def test_potential_at_goal():
    pq = ANABucketHeap()
    pq.incumbent_cost = 100
    
    node_goal = SearchNode("Goal", g_score=90, h_score=0) # f=90, u=inf
    node_other = SearchNode("Other", g_score=10, h_score=80) # f=90, u=1.125

    pq.push(node_other)
    pq.push(node_goal)

    # Goal node should have infinite potential and be popped first
    popped = pq.pop()
    assert popped.state == "Goal"
    assert len(pq) == 1

    popped = pq.pop()
    assert popped.state == "Other"
    assert len(pq) == 0

def test_empty_bucket_cleanup():
    pq = ANABucketHeap()
    pq.incumbent_cost = 100
    
    node1 = SearchNode("A", g_score=10, h_score=80) # f=90
    pq.push(node1)
    
    # Pop the only node
    pq.pop()
    
    # Ensure the bucket is cleaned up
    assert not pq._buckets # Should be empty
    assert not pq._heap # Should be empty
    assert not pq._bucket_metadata # Should be empty
    assert len(pq) == 0

def test_update_incumbent_with_empty_queue():
    pq = ANABucketHeap()
    pq.update_incumbent(50)
    assert pq.incumbent_cost == 50
    assert len(pq) == 0
    assert not pq._heap

def test_update_incumbent_with_no_change_in_order():
    pq = ANABucketHeap()
    pq.incumbent_cost = 200
    
    nodeA = SearchNode("A", g_score=10, h_score=80) # f=90, u=(200-10)/80 = 2.375
    nodeB = SearchNode("B", g_score=20, h_score=75) # f=95, u=(200-20)/75 = 2.4
    
    pq.push(nodeA)
    pq.push(nodeB)

    # B has higher potential
    popped = pq.pop()
    assert popped.state == "B"

    # Update incumbent, but A still has higher potential than any other node
    pq.update_incumbent(150)
    # A: u=(150-10)/80 = 1.75
    # B was popped.
    
    popped = pq.pop()
    assert popped.state == "A"
    assert len(pq) == 0
