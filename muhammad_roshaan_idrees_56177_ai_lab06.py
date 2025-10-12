"""Muhammad Roshaan Idrees_56177_AI_Lab06

**Muhammad Roshaan Idrees**
 **56177**

**Task 1**
"""

import numpy as np
import time
from copy import deepcopy

# Function to get positions of tiles
def coordinates(puzzle):
    pos = {}
    for i in range(len(puzzle)):
        pos[puzzle[i]] = i
    return pos

# Heuristic: number of misplaced tiles
def misplaced_tiles(curr_pos, goal_pos):
    count = 0
    for tile in curr_pos:
        if tile != 0 and curr_pos[tile] != goal_pos[tile]:
            count += 1
    return count

# Solve the puzzle (generalized for n x n)
def solve_puzzle(puzzle, goal, n):
    # Structured arrays for state and priority
    dtstate = [('puzzle', np.object_), ('parent', np.int32), ('gn', np.int32), ('hn', np.int32)]
    dtpriority = [('position', np.int32), ('fn', np.float64)]
    goal_pos = coordinates(goal)
    parent = -1
    gn = 0
    hn = misplaced_tiles(coordinates(puzzle), goal_pos)
    state = np.array([(puzzle, parent, gn, hn)], dtstate)
    priority = np.array([(0, hn)], dtpriority)
    visited = [tuple(puzzle)]

    # Moves: up, down, left, right (deltas)
    moves = [('up', -n), ('down', n), ('left', -1), ('right', 1)]
    while len(priority) > 0:
        priority = np.sort(priority, kind='mergesort', order=['fn', 'position'])
        position, fn = priority[0]
        priority = np.delete(priority, 0, 0)
        puzzle, parent, gn, hn = state[position]
        puzzle = np.array(puzzle)
        blank = int(np.where(puzzle == 0)[0])
        gn += 1
        start_time = time.time()
        for move_name, delta in moves:
            new_blank = blank + delta
            if 0 <= new_blank < n*n:
                # Check move validity based on position
                if (move_name == 'up' and blank < n) or \
                   (move_name == 'down' and blank >= n*(n-1)) or \
                   (move_name == 'left' and blank % n == 0) or \
                   (move_name == 'right' and blank % n == n-1):
                    continue
                openstate = deepcopy(puzzle)
                openstate[blank], openstate[new_blank] = openstate[new_blank], openstate[blank]
                if tuple(openstate) not in visited:
                    hn_new = misplaced_tiles(coordinates(openstate), goal_pos)
                    q = np.array([(openstate, position, gn, hn_new)], dtstate)
                    state = np.append(state, q, 0)
                    fn_new = gn + hn_new
                    q = np.array([(len(state) - 1, fn_new)], dtpriority)
                    priority = np.append(priority, q, 0)
                    if np.array_equal(openstate, goal):
                        print('Puzzle solved!')
                        return state, len(priority)
        end_time = time.time()
        if (end_time - start_time) > 5:  # Increased timeout to 5 seconds
            print('Puzzle not solvable within time limit.')
            return None, len(priority)
        visited.append(tuple(puzzle))
    print('Puzzle not solvable.')
    return None, 0

# Task 1: Update puzzle values, size, and analyze
n = 4  # Updated size to 4x4 (15-puzzle)
goal = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 0]  # Standard solved state
puzzle = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 0, 14, 15]  # Updated values (simple solvable)
state, remaining = solve_puzzle(puzzle, goal, n)
if state is not None:
    path = []
    idx = len(state) - 1
    while idx != -1:
        path.append(state[idx]['puzzle'])
        idx = state[idx]['parent']
    path.reverse()
    print("Solution path for 4x4 puzzle (moves: {}):".format(len(path) - 1))
    for p in path:
        print(np.array(p).reshape(n, n))
        print()

# Task 2: Implement A* example other than above (shortest path in a grid)
def a_star_grid(start, goal, grid):
    moves = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # up, down, left, right
    def heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])  # Manhattan distance
    def is_valid(x, y):
        return 0 <= x < len(grid) and 0 <= y < len(grid[0]) and grid[x][y] == 0
    open_set = [(0, start)]  # (f_score, position)
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}
    while open_set:
        open_set.sort()  # Sort by f_score
        current_f, current = open_set.pop(0)
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]
        for dx, dy in moves:
            neighbor = (current[0] + dx, current[1] + dy)
            if is_valid(neighbor[0], neighbor[1]):
                tentative_g = g_score[current] + 1
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                    if neighbor not in [item[1] for item in open_set]:
                        open_set.append((f_score[neighbor], neighbor))
    return None

# Example grid with obstacles (0 = open, 1 = blocked)
grid = [
    [0, 0, 1, 0],
    [0, 1, 0, 0],
    [1, 0, 0, 0],
    [0, 0, 0, 0]
]
start = (0, 0)
goal = (3, 3)
path = a_star_grid(start, goal, grid)
if path:
    print("Shortest path in grid (moves: {}):".format(len(path) - 1))
    for pos in path:
        print(pos)
else:
    print("No path found.")

"""**Task 2**"""

import heapq
import math

# Graph representation: dictionary of cities with adjacent cities and distances
graph = {
    'Lahore': {'Islamabad': 300, 'Multan': 320, 'Faisalabad': 130},
    'Islamabad': {'Lahore': 300, 'Peshawar': 160, 'Rawalpindi': 20},
    'Multan': {'Lahore': 320, 'Bahawalpur': 100},
    'Faisalabad': {'Lahore': 130, 'Sargodha': 90},
    'Peshawar': {'Islamabad': 160},
    'Rawalpindi': {'Islamabad': 20},
    'Bahawalpur': {'Multan': 100},
    'Sargodha': {'Faisalabad': 90}
}

# Heuristic: straight-line distances to goal (approximated Euclidean distances in km)
# These are precomputed or estimated values for simplicity
heuristic = {
    'Lahore': 0,  # Goal city
    'Islamabad': 300,
    'Multan': 320,
    'Faisalabad': 130,
    'Peshawar': 460,
    'Rawalpindi': 280,
    'Bahawalpur': 420,
    'Sargodha': 220
}

def a_star(graph, start, goal):
    # Priority queue to store (f_score, node)
    open_set = [(0, start)]
    # Dictionary to store the cost to reach each node (g_score)
    g_score = {node: float('infinity') for node in graph}
    g_score[start] = 0
    # Dictionary to store the estimated total cost (f_score = g_score + heuristic)
    f_score = {node: float('infinity') for node in graph}
    f_score[start] = heuristic[start]
    # Dictionary to reconstruct the path
    came_from = {}

    while open_set:
        # Get the node with the lowest f_score
        current_f, current = heapq.heappop(open_set)

        if current == goal:
            # Reconstruct and return the path
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1], g_score[goal]

        # Check all neighbors of the current node
        for neighbor, distance in graph[current].items():
            # Tentative g_score to the neighbor
            tentative_g = g_score[current] + distance

            if tentative_g < g_score[neighbor]:
                # This path is better, so update
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + heuristic[neighbor]
                if (f_score[neighbor], neighbor) not in open_set:
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return None, None  # No path found

# Main: Set start and goal cities
start_city = 'Peshawar'
goal_city = 'Lahore'
path, total_distance = a_star(graph, start_city, goal_city)

if path:
    print(f"Shortest path from {start_city} to {goal_city}: {' -> '.join(path)}")
    print(f"Total distance: {total_distance} km")
else:
    print(f"No path found from {start_city} to {goal_city}")