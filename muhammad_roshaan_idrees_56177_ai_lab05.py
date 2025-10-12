"""Muhammad Roshaan Idrees_56177_AI_Lab05

**Muhammad Roshaan Idrees**
**56177**

**DFS**
"""

graph = {
    0: [1, 2],
    1: [],
    2: [3, 4],
    3: [],
    4: []
}

# DFS function
def dfs(node, visited):
    if node not in visited:
        print(f"Visited {node}, Adjacent: {graph[node]}")
        visited.add(node)
        for neighbor in graph[node]:
            dfs(neighbor, visited)


print("DFS Traversal Order:")
dfs(0, set())

"""**BFS**"""

from collections import deque

graph = {
    0: [1, 2],
    1: [],
    2: [3, 4],
    3: [],
    4: []
}


def bfs(start):
    visited = set()
    queue = deque([start])

    while queue:
        node = queue.popleft()
        if node not in visited:
            print(f"Visited {node}, Adjacent: {graph[node]}")
            visited.add(node)
            queue.extend(graph[node])

print("BFS Traversal Order:")
bfs(0)