from collections import deque
import heapq

class Problem:
    def __init__(self, filename):
        self.nodes = {}
        self.edges = {}
        self.origin = None
        self.destinations = set()
        self._read_file(filename)
    
    def _read_file(self, filename):
        with open(filename, 'r') as file:
            lines = file.readlines()
        
        section = None
        for line in lines:
            line = line.strip()
            if not line:
                continue
            
            if line.startswith("Nodes:"):
                section = "nodes"
            elif line.startswith("Edges:"):
                section = "edges"
            elif line.startswith("Origin:"):
                section = "origin"
            elif line.startswith("Destinations:"):
                section = "destinations"
            elif section == "nodes":
                node_id, coord = line.split(":")
                self.nodes[int(node_id.strip())] = tuple(map(int, coord.strip()[1:-1].split(',')))
            elif section == "edges":
                edge, cost = line.split(":")
                n1, n2 = map(int, edge.strip()[1:-1].split(','))
                cost = int(cost.strip())
                if n1 not in self.edges:
                    self.edges[n1] = {}
                self.edges[n1][n2] = cost
            elif section == "origin":
                self.origin = int(line.strip())
            elif section == "destinations":
                self.destinations.update(map(int, line.strip().split(';')))

def bfs_search(problem):
    queue = deque([(problem.origin, [problem.origin])])
    visited = set()
    
    while queue:
        node, path = queue.popleft()
        
        if node in problem.destinations:
            return node, len(visited), path
        
        if node not in visited:
            visited.add(node)
            for neighbor in sorted(problem.edges.get(node, {}).keys()):
                if neighbor not in visited:
                    queue.append((neighbor, path + [neighbor]))
    
    return None, len(visited), []

def dfs_search(problem):
    stack = [(problem.origin, [problem.origin])]
    visited = set()
    
    while stack:
        node, path = stack.pop()
        
        if node in problem.destinations:
            return node, len(visited), path
        
        if node not in visited:
            visited.add(node)
            for neighbor in sorted(problem.edges.get(node, {}).keys()):
                if neighbor not in visited:
                    stack.append((neighbor, path + [neighbor]))
    
    return None, len(visited), []

def gbfs_search(problem):
    priority_queue = [(0, problem.origin, [problem.origin])]
    visited = set()
    
    while priority_queue:
        cost, node, path = heapq.heappop(priority_queue)
        
        if node in problem.destinations:
            return node, len(visited), path
        
        if node not in visited:
            visited.add(node)
            for neighbor, edge_cost in sorted(problem.edges.get(node, {}).items(), key=lambda x: x[1]):
                if neighbor not in visited:
                    heapq.heappush(priority_queue, (edge_cost, neighbor, path + [neighbor]))
    
    return None, len(visited), []

def astar_search(problem):
    def heuristic(node):
        return min(((problem.nodes[dest][0] - problem.nodes[node][0])**2 + (problem.nodes[dest][1] - problem.nodes[node][1])**2)**0.5 for dest in problem.destinations)
    
    priority_queue = [(0 + heuristic(problem.origin), 0, problem.origin, [problem.origin])]
    visited = {}
    
    while priority_queue:
        _, cost_so_far, node, path = heapq.heappop(priority_queue)
        
        if node in problem.destinations:
            return node, len(visited), path
        
        if node not in visited or cost_so_far < visited[node]:
            visited[node] = cost_so_far
            for neighbor, edge_cost in problem.edges.get(node, {}).items():
                new_cost = cost_so_far + edge_cost
                heapq.heappush(priority_queue, (new_cost + heuristic(neighbor), new_cost, neighbor, path + [neighbor]))
    
    return None, len(visited), []
