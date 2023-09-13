"""Implementating the Best-First Search algorithm for finding the optimal route between two cities."""

import heapq
import matplotlib.pyplot as plt
import networkx as nx


#Implementation for de class Node
class Node:
    #the city is the name of the node we use this name only for assume the node
    def __init__(self, city):
        self.city = city
        self.parent = None

    def __lt__(self, other):
        return False  # Dummy comparison for priority queue

class Graph:
    def __init__(self):
        self.edges = {}
    
    def add_edge(self, start, end, cost):
        if start not in self.edges:
            self.edges[start] = []
        self.edges[start].append((end, cost))
        if end not in self.edges:
            self.edges[end] = []  # Ensure both directions are available
        self.edges[end].append((start, cost))

def best_first_search(graph, start, goal):
    # Initialize the frontier using the start state
    frontier = [(0, Node(start))]
    # Initialize the reached set to be empty
    reached = {start: None}

    while frontier:
        # Choose the lowest-cost node in the frontier
        _, current_node = heapq.heappop(frontier)
        #we do a pop to the frontier and we get the current node

        # Print information about expanding the current node
        #expanding node is the parent node
        if reached[current_node.city]:
            parent_node = reached[current_node.city]
            cost_to_parent = None
            for neighbor, cost in graph.edges[parent_node.city]:
                if neighbor == current_node.city:
                    cost_to_parent = cost
                    break
            #print information about expanding the current node
            print(f"Expanding node: {parent_node.city} -> {current_node.city}")
            print(f"Cost from start to {parent_node.city}: {cost_to_parent} miles")
            print(f"Cost from {parent_node.city} to {current_node.city}: {cost_to_parent} miles")
        if current_node.city == goal:
            break

        for neighbor, cost in graph.edges[current_node.city]:
            if neighbor not in reached:
                reached[neighbor] = current_node
                heapq.heappush(frontier, (0, Node(neighbor)))

    path = []
    while current_node:
        path.append(current_node.city)
        current_node = reached[current_node.city]

    return path[::-1]

# Create a graph and add edges
graph = Graph()
graph.add_edge("Ellensburg", "Spokane", 175)
graph.add_edge("Ellensburg", "Pendleton", 168)
graph.add_edge("Pendleton", "Spokane", 200)
graph.add_edge("Spokane", "BonnersFerry", 112)
graph.add_edge("Spokane", "Missoula", 199)
graph.add_edge("Missoula", "BonnersFerry", 249)
graph.add_edge("Missoula", "Butte", 119)
graph.add_edge("Missoula", "Helena", 111)
graph.add_edge("Missoula", "WestGlacier", 151)
graph.add_edge("BonnersFerry", "WestGlacier", 176)
graph.add_edge("Helena", "Butte", 65)
graph.add_edge("Helena", "GreatFalls", 91)
graph.add_edge("GreatFalls", "WestGlacier", 211)
graph.add_edge("Helena", "WestGlacier", 243)
graph.add_edge("Havre", "WestGlacier", 231)
graph.add_edge("GreatFalls", "Havre", 115)

# Define the start and goal cities
start_city = "Ellensburg"
goal_city = "Havre"

# Find the route using Best-First Search
route = best_first_search(graph, start_city, goal_city)

# Print the optimal route
if route:
    total_cost = 0
    print("Optimal Route:")
    for i in range(len(route) - 1):
        parent_city = route[i]
        child_city = route[i + 1]
        cost_to_child = None
        for neighbor, cost in graph.edges[parent_city]:
            if neighbor == child_city:
                cost_to_child = cost
                break
        total_cost += cost_to_child
        print(f"{parent_city} -> {child_city}: {cost_to_child} miles")

    print(f"Total Cost: {total_cost} miles")
else:
    print("No route found from", start_city, "to", goal_city)



'''The code below is just for plot the graph does not affect the code above and use nx library for construct the graph.
Again, it is just for visualization purposes.'''
# Create an empty directed graph
G = nx.DiGraph()

# Add edges with associated costs
edges = [
    ("Ellensburg", "Spokane", 175),
    ("Ellensburg", "Pendleton", 168),
    ("Pendleton", "Spokane", 200),
    ("Spokane", "BonnersFerry", 112),
    ("Spokane", "Missoula", 199),
    ("Missoula", "BonnersFerry", 249),
    ("Missoula", "Butte", 119),
    ("Missoula", "Helena", 111),
    ("Missoula", "WestGlacier", 151),
    ("BonnersFerry", "WestGlacier", 176),
    ("Helena", "Butte", 65),
    ("Helena", "GreatFalls", 91),
    ("GreatFalls", "WestGlacier", 211),
    ("Helena", "WestGlacier", 243),
    ("Havre", "WestGlacier", 231),
    ("GreatFalls", "Havre", 115),
]

for edge in edges:
    G.add_edge(edge[0], edge[1], weight=edge[2])

# Visualization of the graph
pos = nx.spring_layout(G, seed=42)  # Position nodes using a spring layout
labels = nx.get_edge_attributes(G, 'weight')
nx.draw(G, pos, with_labels=True, node_size=500, node_color='skyblue', font_size=10, labels={node: node for node in G.nodes()})
nx.draw_networkx_edge_labels(G, pos, edge_labels=labels)
plt.show()




