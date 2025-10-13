from collections import deque
import random
import string

import matplotlib
import matplotlib.pyplot as plt
import networkx as nx

# Use non-interactive backend
matplotlib.use('Agg')


class Node:
    def __init__(self, state, parent=None, action=None, depth=0):
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
            if (child_state in CLOSED or
                    any(n.state == child_state for n in OPEN)):
                continue

            child = Node(child_state, parent=node, action=action,
                         depth=node.depth + 1)

            if goal_state(child.state):
                return child.path()

            OPEN.append(child)

    return None


def random_graph(num_nodes, edge_prob):
    """Create an undirected random graph.

    Args:
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


def visualize_graph(graph, title="Random Graph", path=None, save_file=None):
    """Visualize the graph using networkx and matplotlib.

    Args:
        graph: Dictionary representing the graph
        title: Title for the graph visualization
        path: Optional list of nodes representing a path to highlight
        save_file: Optional filename to save the visualization
    """
    # Create a networkx graph
    G = nx.Graph()

    # Add nodes and edges
    for node, edges in graph.items():
        G.add_node(node)
        for edge in edges:
            G.add_edge(node, edge)

    # Set up the figure
    plt.figure(figsize=(10, 8))

    # Create a layout for the nodes
    pos = nx.spring_layout(G, seed=42)  # Using seed for reproducibility

    # Draw the graph
    nx.draw_networkx_nodes(G, pos, node_size=700, node_color='lightblue')
    nx.draw_networkx_labels(G, pos, font_size=12, font_family='sans-serif')
    nx.draw_networkx_edges(G, pos, width=1.5, alpha=0.7, edge_color='gray')

    # Highlight the path if provided
    if path and len(path) > 1:
        path_nodes = [state for state, _ in path]
        path_edges = [(path_nodes[i], path_nodes[i+1])
                      for i in range(len(path_nodes)-1)]

        # Draw the path in red
        nx.draw_networkx_edges(G, pos, edgelist=path_edges, width=3,
                               alpha=0.8, edge_color='red')

        # Highlight nodes in the path
        nx.draw_networkx_nodes(G, pos, nodelist=path_nodes, node_size=700,
                               node_color='lightcoral')

    plt.title(title, fontsize=16)
    plt.axis('off')  # Turn off the axis

    # Save the figure if a filename is provided
    if save_file:
        plt.savefig(save_file, dpi=300, bbox_inches='tight')
        print(f"Graph visualization saved to {save_file}")
    else:
        # If no save file is provided, save with a default name
        default_file = "graph_visualization.png"
        plt.savefig(default_file, dpi=300, bbox_inches='tight')
        print(f"Graph visualization saved to {default_file}")

    plt.close()  # Close the figure to free memory


def visualize_bfs_solution(graph, start, goal, solution):
    """Visualize the graph with the BFS solution path highlighted.

    Args:
        graph: Dictionary representing the graph
        start: Starting node
        goal: Goal node
        solution: Solution path from BFS
    """
    if solution:
        visualize_graph(graph, f"BFS Path from {start} to {goal}", solution,
                        "bfs_solution.png")
        print("BFS solution visualization saved to bfs_solution.png")
    else:
        print("No solution found to visualize.")


if __name__ == "__main__":
    # Generate a random graph
    graph = random_graph(num_nodes=8, edge_prob=0.3)
    print("Generated graph:")
    print(graph)
    print()

    # Define the successor function
    def successors(state):
        return [(f"Go to {n}", n) for n in graph[state]]

    # Define the goal test function
    def goal_state(state):
        return state == 'G'

    # Run BFS to find a path from 'A' to 'G'
    print("Searching for path from 'A' to 'G'...")
    solution = bfs('A', goal_state, successors)

    if solution:
        print("Solution found:")
        for i, (state, action) in enumerate(solution):
            print(f"  Step {i}: {state} -> {action}")
        print()
    else:
        print("No solution found!")
        print()

    # Visualize the graph
    print("Visualizing the graph...")
    visualize_graph(graph, "Random Graph Generated by BFS")

    # Visualize the solution if found
    if solution:
        print("Visualizing the BFS solution path...")
        visualize_bfs_solution(graph, 'A', 'G', solution)
