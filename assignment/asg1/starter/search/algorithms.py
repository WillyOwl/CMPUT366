import heapq
import math

class State:
    """
    Class to represent a state on grid-based pathfinding problems. The class contains two static variables:
    map_width and map_height containing the width and height of the map. Although these variables are properties
    of the map and not of the state, they are used to compute the hash value of the state, which is used
    in the CLOSED list. 

    Each state has the values of x, y, g, h, and cost. The cost is used as the criterion for sorting the nodes
    in the OPEN list for both Dijkstra's algorithm and A*. For Dijkstra the cost should be the g-value, while
    for A* the cost should be the f-value of the node. 
    """
    map_width = 0
    map_height = 0
    
    def __init__(self, x, y):
        """
        Constructor - requires the values of x and y of the state. All the other variables are
        initialized with the value of 0.
        """
        self._x = x
        self._y = y
        self._g = 0
        self._cost = 0
        self._parent = None
        
    def __repr__(self):
        """
        This method is invoked when we call a print instruction with a state. It will print [x, y],
        where x and y are the coordinates of the state on the map. 
        """
        state_str = "[" + str(self._x) + ", " + str(self._y) + "]"
        return state_str
    
    def __lt__(self, other):
        """
        Less-than operator; used to sort the nodes in the OPEN list
        """
        return self._cost < other._cost
    
    def state_hash(self):
        """
        Given a state (x, y), this method returns the value of x * map_width + y. This is a perfect 
        hash function for the problem (i.e., no two states will have the same hash value). This function
        is used to implement the CLOSED list of the algorithms. 
        """
        return self._y * State.map_width + self._x
    
    def __eq__(self, other):
        """
        Method that is invoked if we use the operator == for states. It returns True if self and other
        represent the same state; it returns False otherwise. 
        """
        return self._x == other._x and self._y == other._y

    def get_x(self):
        """
        Returns the x coordinate of the state
        """
        return self._x
    
    def set_parent(self, parent):
        """
        Sets the parent of a node in the search tree
        """
        self._parent = parent

    def get_parent(self):
        """
        Returns the parent of a node in the search tree
        """
        return self._parent
    
    def get_y(self):
        """
        Returns the y coordinate of the state
        """
        return self._y
    
    def get_g(self):
        """
        Returns the g-value of the state
        """
        return self._g
        
    def set_g(self, g):
        """
        Sets the g-value of the state
        """
        self._g = g

    def get_cost(self):
        """
        Returns the cost of a state; the cost is determined by the search algorithm
        """
        return self._cost
    
    def set_cost(self, cost):
        """
        Sets the cost of the state; the cost is determined by the search algorithm 
        """
        self._cost = cost
    
class Dijkstra:
    """Dijkstra's Algorithm Implementation"""

    def __init__ (self, gridded_map):
        self.map = gridded_map
        self.closed_data = [] # closed_data will be used in the prepared function calls in main.py later

    def search(self, start, goal):
        """
        The process of searching using Dijkstra.

        Return: A tuple (path, cost, expanded_dijkstra)
        """

        open_list = []
        closed_list = set()
        expanded_dijkstra = 0
        self.closed_data = {}  # Store as instance variable for plotting
        
        g_values = {}

        start.set_g(0)
        start.set_cost(0) # cost = g-value
        start.set_parent(None)
        g_values[start.state_hash()] = 0

        heapq.heappush(open_list, start)

        while open_list:
            current = heapq.heappop(open_list)

            if current.state_hash() in closed_list:
                continue

            if current.get_g() > g_values.get(current.state_hash(), float('inf')):
                continue

            closed_list.add(current.state_hash())
            self.closed_data[current.state_hash()] = current
            expanded_dijkstra += 1

            if current == goal:
                # If the goal is reached

                path = []
                node = current

                while node is not None:
                    path.append(node)
                    node = node.get_parent()

                path.reverse()

                return path, current.get_g(), expanded_dijkstra
            
            successors = self.map.successors(current)

            for successor in successors:
                if successor.state_hash() in closed_list:
                    continue

                if successor.get_g() < g_values.get(successor.state_hash(), float('inf')):
                    g_values[successor.state_hash()] = successor.get_g()
                    successor.set_parent(current)
                    successor.set_cost(successor.get_g())  # For Dijkstra, cost = g-value

                    heapq.heappush(open_list, successor)

        # No path found

        return None, -1.0, expanded_dijkstra
    
    def get_closed_data(self):

        return self.closed_data
    
class AStar:
    """A* algorithm implementation"""

    def __init__ (self, gridded_map):
        self.map = gridded_map
        self.closed_data = [] # closed_data will be used in the prepared function calls in main.py later

    def heuristic(self, state, goal):
        dx = abs(state.get_x() - goal.get_x())
        dy = abs(state.get_y() - goal.get_y())

        if dx > dy:
            return dy * 1.5 + (dx - dy) * 1.0
        
        else:
            return dx * 1.5 + (dy - dx) * 1.0
    
    def search(self, start, goal):
        """
        The process of searching using A*.

        Return: A tuple (path, cost, expanded_nodes)
        """

        open_list = []
        closed_list = set()
        expanded_dijkstra = 0
        self.closed_data = {}
        
        g_values = {}

        start.set_g(0)
        h = self.heuristic(start, goal)
        start.set_cost(start.get_g() + h)
        start.set_parent(None)
        g_values[start.state_hash()] = 0

        heapq.heappush(open_list, start)

        while open_list:
            current = heapq.heappop(open_list)

            if current.state_hash() in closed_list:
                continue

            if current.get_g() > g_values.get(current.state_hash(), float('inf')):
                continue

            closed_list.add(current.state_hash())
            self.closed_data[current.state_hash()] = current
            expanded_dijkstra += 1

            if current == goal:
                # If the goal is reached

                path = []
                node = current

                while node is not None:
                    path.append(node)
                    node = node.get_parent()

                path.reverse()

                return path, current.get_g(), expanded_dijkstra
            
            successors = self.map.successors(current)

            for successor in successors:
                # Only add to open list if we found a better path
                if successor.get_g() < g_values.get(successor.state_hash(), float('inf')):
                    g_values[successor.state_hash()] = successor.get_g()
                    successor.set_parent(current)
                    h = self.heuristic(successor, goal)
                    successor.set_cost(successor.get_g() + h)  

                    heapq.heappush(open_list, successor)

        # If no path found

        return None, -1.0, expanded_dijkstra
    
    def get_closed_data(self):

        return self.closed_data