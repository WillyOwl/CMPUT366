import heapq
import random
import string
import matplotlib
import matplotlib.pyplot as plt
import networkx as nx

# Use non-interactive backend
matplotlib.use('Agg')

class Node:
    def __init__(self, state, parent=None, action=None, depth=0, cost=0):
        self.state = state
        self.parent = parent
        self.action = action
        self.depth = depth
        self.cost = cost  # g-cost: cost from start to this node

    def path(self):
        """Reconstruct path from root to this node"""
        node, path_back = self, []
        while node:
            path_back.append((node.state, node.action))
            node = node.parent
        return list(reversed(path_back))
    
    def __lt__(self, other):
        """Less-than operator for priority queue comparison"""
        return self.cost < other.cost

def dijkstra(initial_state, goal_state, successors):
    """
    Args:
        initial_state: The starting state
        goal_state: A function that returns True if a state is a goal
        successors: A function that returns (action, next_state, cost) tuples
        
    Returns:
        A list of (state, action) tuples representing the path from start to goal,
        or None if no path exists
    """
    
    root = Node(initial_state, cost=0)
    
    if goal_state(root.state):
        return root.path()
    
    # Priority queue ordered by path cost (g-value)
    OPEN = [root]
    heapq.heapify(OPEN)
    
    # CLOSED stores already visited states with their best costs
    CLOSED = {}
    
    while OPEN:
        current = heapq.heappop(OPEN)
        
        # If we've reached the goal, return the path
        if goal_state(current.state):
            return current.path()
        
        # If we've already found a better path to this state, skip it
        if current.state in CLOSED and CLOSED[current.state] <= current.cost:
            continue
            
        # Add current state to CLOSED with its cost
        CLOSED[current.state] = current.cost
        
        # Expand current node
        for action, next_state, step_cost in successors(current.state):
            new_cost = current.cost + step_cost
            
            # If we've found a better path to next_state, or haven't visited it yet
            if next_state not in CLOSED or new_cost < CLOSED.get(next_state, float('inf')):
                child = Node(
                    state=next_state,
                    parent=current,
                    action=action,
                    depth=current.depth + 1,
                    cost=new_cost
                )
                heapq.heappush(OPEN, child)
    
    # No path found
    return None

def weighted_random_graph(num_nodes, edge_prob, min_cost=1, max_cost=10):
    """
    Create a weighted undirected random graph for testing Dijkstra's algorithm.
    
    Args:
        num_nodes: Number of nodes in the graph
        edge_prob: Probability of creating an edge between any two nodes
        min_cost: Minimum edge cost
        max_cost: Maximum edge cost
        
    Returns:
        A dictionary representing the weighted graph where each key is a node
        and each value is a list of (neighbor, cost) tuples
    """
    # Label nodes A, B, C, ...
    labels = list(string.ascii_uppercase[:num_nodes])
    graph = {node: [] for node in labels}
    
    for i in range(num_nodes):
        for j in range(i+1, num_nodes):  # only consider pairs once
            if random.random() < edge_prob:  # e.g. 30% chance
                a, b = labels[i], labels[j]
                cost = random.randint(min_cost, max_cost)
                graph[a].append((b, cost))
                graph[b].append((a, cost))
    return graph

def visualize_graph(graph, title="Weighted Graph", path=None, save_file=None):
    """Visualize the weighted graph using networkx and matplotlib.

    Args:
        graph: Dictionary representing the weighted graph
        title: Title for the graph visualization
        path: Optional list of nodes representing a path to highlight
        save_file: Optional filename to save the visualization
    """
    # Create a networkx graph
    G = nx.Graph()

    # Add nodes and edges
    for node, edges in graph.items():
        G.add_node(node)
        for neighbor, cost in edges:
            G.add_edge(node, neighbor, weight=cost)

    # Set up the figure
    plt.figure(figsize=(10, 8))

    # Create a layout for the nodes
    pos = nx.spring_layout(G, seed=42)  # Using seed for reproducibility

    # Draw the graph
    nx.draw_networkx_nodes(G, pos, node_size=700, node_color='lightblue')
    nx.draw_networkx_labels(G, pos, font_size=12, font_family='sans-serif')
    nx.draw_networkx_edges(G, pos, width=1.5, alpha=0.7, edge_color='gray')
    
    # Draw edge labels (weights)
    edge_labels = nx.get_edge_attributes(G, 'weight')
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels)

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

def visualize_dijkstra_solution(graph, start, goal, solution):
    """Visualize the graph with the Dijkstra solution path highlighted.

    Args:
        graph: Dictionary representing the weighted graph
        start: Starting node
        goal: Goal node
        solution: Solution path from Dijkstra
    """
    if solution:
        visualize_graph(graph, f"Dijkstra Path from {start} to {goal}", solution,
                        "dijkstra_solution.png")
        print("Dijkstra solution visualization saved to dijkstra_solution.png")
    else:
        print("No solution found to visualize.")

def dijkstra_with_visualization(initial_state, goal_state, successors):
    """
    Dijkstra's algorithm with visualization of the search process.
    
    Returns both the path and statistics about the search process.
    """
    root = Node(initial_state, cost=0)
    
    if goal_state(root.state):
        return root.path(), {"nodes_expanded": 0, "max_frontier_size": 1}
    
    OPEN = [root]
    heapq.heapify(OPEN)
    CLOSED = {}
    
    nodes_expanded = 0
    max_frontier_size = 1
    
    while OPEN:
        current = heapq.heappop(OPEN)
        nodes_expanded += 1
        
        if goal_state(current.state):
            return current.path(), {"nodes_expanded": nodes_expanded, "max_frontier_size": max_frontier_size}
        
        if current.state in CLOSED and CLOSED[current.state] <= current.cost:
            continue
            
        CLOSED[current.state] = current.cost
        
        for action, next_state, step_cost in successors(current.state):
            new_cost = current.cost + step_cost
            
            if next_state not in CLOSED or new_cost < CLOSED.get(next_state, float('inf')):
                child = Node(
                    state=next_state,
                    parent=current,
                    action=action,
                    depth=current.depth + 1,
                    cost=new_cost
                )
                heapq.heappush(OPEN, child)
        
        max_frontier_size = max(max_frontier_size, len(OPEN))
    
    return None, {"nodes_expanded": nodes_expanded, "max_frontier_size": max_frontier_size}

if __name__ == "__main__":
    # Generate a weighted random graph
    graph = weighted_random_graph(num_nodes=8, edge_prob=0.3, min_cost=1, max_cost=10)
    print("Generated weighted graph:")
    for node, edges in graph.items():
        print(f"  {node}: {edges}")
    print()

    # Define the successor function
    def successors(state):
        return [(f"Go to {neighbor} (cost {cost})", neighbor, cost)
                for neighbor, cost in graph[state]]

    # Define the goal test function
    def goal_state(state):
        return state == 'H'

    # Run Dijkstra to find a path from 'A' to 'H'
    print("Searching for path from 'A' to 'H'...")
    solution = dijkstra('A', goal_state, successors)

    if solution:
        print("Solution found:")
        total_cost = 0
        for i, (state, action) in enumerate(solution):
            if i > 0:
                # Extract cost from action string
                cost_str = action.split("cost ")[1].split(")")[0]
                step_cost = int(cost_str)
                total_cost += step_cost
                print(f"  Step {i}: {action}")
            else:
                print(f"  Start at: {state}")
        
        print(f"\nTotal path cost: {total_cost}")
        print(f"Path length: {len(solution)} states")
        print()
    else:
        print("No solution found!")
        print()

    # Visualize the graph
    print("Visualizing the graph...")
    visualize_graph(graph, "Weighted Random Graph Generated by Dijkstra")

    # Visualize the solution if found
    if solution:
        print("Visualizing the Dijkstra solution path...")
        visualize_dijkstra_solution(graph, 'A', 'H', solution)