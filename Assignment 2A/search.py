import sys
import algorithm

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python search.py <filename> <method>")
        sys.exit(1)
    
    filename = sys.argv[1]
    method = sys.argv[2].upper()
    
    problem = algorithm.Problem(filename)
    
    if method == "BFS":
        goal, num_nodes, path = algorithm.bfs_search(problem)
    elif method == "DFS":
        goal, num_nodes, path = algorithm.dfs_search(problem)
    elif method == "GBFS":
        goal, num_nodes, path = algorithm.gbfs_search(problem)
    elif method == "AS":
        goal, num_nodes, path = algorithm.astar_search(problem)
    else:
        print("Only BFS, DFS, GBFS, and AS methods are implemented in this script.")
        sys.exit(1)
    
    if goal:
        print(f"{filename} {method}")
        print(f"{goal} {num_nodes}")
        print(" ".join(map(str, path)))
    else:
        print("No solution found.")

