import sys
import heapq
from collections import deque

# Helper function to read and parse the problem file
def parse_problem(filename):
    with open(filename, 'r') as file:
        lines = file.readlines()

    nodes = {}
    edges = {}
    origin = None
    destinations = []

    node_section = True

    for line in lines:
        line = line.strip()
        if not line:
            continue  # Skip empty lines
        
        if line == "Edges:":
            node_section = False
            continue
        if node_section:
            if "Nodes:" in line:
                continue  # Skip the "Nodes:" line
            try:
                node_id, coords = line.split(":")
                coords = coords.strip()
                x, y = map(int, coords.strip("()").split(","))
                nodes[int(node_id)] = (x, y)
            except ValueError as e:
                print(f"Error parsing coordinates for node {line}: {e}")
                continue
        elif line.startswith("Origin:"):
            origin = int(line.split(":")[1].strip())
        elif line.startswith("Destinations:"):
                 # destinations = list(map(int, line.split(":")[1].split(";")))
            destinations = list(map(int, line.split(":")[1].strip().replace(';', ',').split(",")))

        # Parsing Edges
        elif "(" in line and ")" in line:
            try:
                edge, cost = line.split(":")
                node1, node2 = map(int, edge.strip("()").split(","))
                cost = int(cost.strip())
                if node1 not in edges:
                    edges[node1] = []
                edges[node1].append((node2, cost))
            except ValueError as e:
                print(f"Error parsing edge {line}: {e}")
                continue

    return nodes, edges, origin, destinations

# Uninformed Search Algorithms
def dfs(nodes, edges, origin, destinations):
    stack = [origin]
    visited = set()
    parent = {origin: None}

    while stack:
        current = stack.pop()

        if current in destinations:
            # Reconstruct path
            path = []
            while current is not None:
                path.append(current)
                current = parent[current]
            return path[::-1]

        visited.add(current)
        for neighbor, _ in edges.get(current, []):
            if neighbor not in visited:
                stack.append(neighbor)
                parent[neighbor] = current
    return None

def bfs(nodes, edges, origin, destinations):
    queue = deque([origin])
    visited = set()
    parent = {origin: None}

    while queue:
        current = queue.popleft()

        if current in destinations:
            # Reconstruct path
            path = []
            while current is not None:
                path.append(current)
                current = parent[current]
            return path[::-1]

        visited.add(current)
        for neighbor, _ in edges.get(current, []):
            if neighbor not in visited:
                queue.append(neighbor)
                parent[neighbor] = current
    return None

# Informed Search Algorithms (GBFS and A*)
def gbfs(nodes, edges, origin, destinations, heuristic):
    priority_queue = []
    heapq.heappush(priority_queue, (heuristic[origin], origin))
    visited = set()
    parent = {origin: None}

    while priority_queue:
        _, current = heapq.heappop(priority_queue)

        if current in destinations:
            # Reconstruct path
            path = []
            while current is not None:
                path.append(current)
                current = parent[current]
            return path[::-1]

        visited.add(current)
        for neighbor, _ in edges.get(current, []):
            if neighbor not in visited:
                heapq.heappush(priority_queue, (heuristic[neighbor], neighbor))
                parent[neighbor] = current
    return None

# using : python3 search.py test_case_7.txt AS
def a_star(nodes, edges, origin, destinations, heuristic):
    g_cost = {origin: 0}
    f_cost = {origin: heuristic[origin]}
    priority_queue = []
    heapq.heappush(priority_queue, (f_cost[origin], origin))
    visited = set()
    parent = {origin: None}

    while priority_queue:
        _, current = heapq.heappop(priority_queue)

        if current in destinations:
            # Reconstruct path
            path = []
            while current is not None:
                path.append(current)
                current = parent[current]
            return path[::-1]

        visited.add(current)
        for neighbor, cost in edges.get(current, []):
            new_g_cost = g_cost[current] + cost
            if neighbor not in visited or new_g_cost < g_cost.get(neighbor, float('inf')):
                g_cost[neighbor] = new_g_cost
                f_cost[neighbor] = new_g_cost + heuristic[neighbor]
                heapq.heappush(priority_queue, (f_cost[neighbor], neighbor))
                parent[neighbor] = current
    return None

def depth_limited_dfs(edges, origin, destinations, limit):
    stack = [(origin, 0)]
    visited = set()
    parent = {origin: None}

    while stack:
        current, depth = stack.pop()

        if current in destinations:
            # Reconstruct path
            path = []
            while current is not None:
                path.append(current)
                current = parent[current]
            return path[::-1]

        if depth < limit:
            visited.add(current)
            for neighbor, _ in sorted(edges.get(current, []), key=lambda x: x[0]):
                if neighbor not in visited:
                    stack.append((neighbor, depth + 1))
                    parent[neighbor] = current
    return None

# Custom Algorithms (CUS1 and CUS2)
def cus1(nodes, edges, origin, destinations):
    max_depth = 20  # Arbitrary depth cap to prevent infinite loop
    for depth in range(max_depth):
        result = depth_limited_dfs(edges, origin, destinations, depth)
        if result is not None:
            return result
    return None

def cus2(nodes, edges, origin, destinations):
    # Placeholder custom logic for CUS2
    return bfs(nodes, edges, origin, destinations)

# Main function to run the program
def main():
    if len(sys.argv) != 3:
        print("Usage: python search.py <filename> <method>")
        sys.exit(1)

    filename = sys.argv[1]
    method = sys.argv[2]

    nodes, edges, origin, destinations = parse_problem(filename)

    # Heuristic for GBFS and A* (simple heuristic based on Euclidean distance)
    heuristic = {node: float('inf') for node in nodes}
    for node in nodes:
        x1, y1 = nodes[node]
        x2, y2 = nodes[destinations[0]]  # Using first destination as heuristic
        heuristic[node] = abs(x1 - x2) + abs(y1 - y2)

    # Select the search method
    if method == "DFS":
        path = dfs(nodes, edges, origin, destinations)
    elif method == "BFS":
        path = bfs(nodes, edges, origin, destinations)
    elif method == "GBFS":
        path = gbfs(nodes, edges, origin, destinations, heuristic)
    elif method == "AS":
        path = a_star(nodes, edges, origin, destinations, heuristic)
    elif method == "CUS1":
        path = cus1(nodes, edges, origin, destinations)
    elif method == "CUS2":
        path = cus2(nodes, edges, origin, destinations)
    else:
        print("Unknown method")
        sys.exit(1)

    # Output the result
    if path:
        print(f"{filename} {method}")
        print(f"goal {path[-1]}")
        print(f"number_of_nodes {len(path)}")
        print("path:", " -> ".join(map(str, path)))
    else:
        print("No path found")

if __name__ == "__main__":
    main()
