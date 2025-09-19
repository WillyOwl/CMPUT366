from collections import deque
import random
import string

class Node:
    def __init__(self, state, parent = None, action = None, depth = 0):
        self.state = state
        self.parent = parent
        self.action = action
        self.depth = depth

    def path(self):
        """Reconstruct path from root to this node"""

        node, path_back = self, []

        while node:
            path_back.append((node.state, node.action))
            node = node.parent

        # Starting from the goal state to the start state

        return list(reversed(path_back))
    
        # Reverse the list to align with the correct order

def bfs(initial_state, goal_state, successors):
    """
    BFS search with OPEN (all nodes encountered but have not been expanded)
    and CLOSED (all nodes encountered)
    """

    root = Node(initial_state)

    if goal_state(root.state):
        return root.path()
    
    OPEN = deque([root])
    CLOSED = set()

    while OPEN:
        node = OPEN.popleft()
        CLOSED.add(node.state)

        for action, child_state in successors(node.state):
            if child_state in CLOSED or any(n.state == child_state for n in OPEN):
                continue

            child = Node(child_state, parent = node, action = action, depth = node.depth + 1)

            if goal_state(child.state):
                return child.path()
                
            OPEN.append(child)

    return None

def random_graph(num_nodes, edge_prob):
    """
    Create an undirected random graph.
    
    num_nodes: how many nodes
    edge_prob: chance of edge between any two nodes
    
    """

    # Label nodes A, B, C, ...
    labels = list(string.ascii_uppercase[:num_nodes])
    graph = {node: [] for node in labels}
    
    for i in range(num_nodes):
        for j in range(i+1, num_nodes):  # only consider pairs once
            if random.random() < edge_prob:  # e.g. 30% chance
                a, b = labels[i], labels[j]
                graph[a].append(b)
                graph[b].append(a)
    return graph

if __name__ == "__main__":

    graph = random_graph(num_nodes=8, edge_prob=0.3)


    def successors(state):
        return [(f"Go to {n}", n) for n in graph[state]]
    
    def goal_state(state):
        return state == 'G'
    
    solution = bfs('A', goal_state, successors)

    print(graph)

    print(solution)