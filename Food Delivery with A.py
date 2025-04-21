import heapq
import time

# ---------------------- GRID ----------------------
city_grid = [
    ['R', '.', '.', 'X', '.'],
    ['.', 'X', '.', '.', '.'],
    ['.', '.', '.', 'X', 'C']
]

rows, cols = len(city_grid), len(city_grid[0])

def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def find_pos(symbol):
    for r in range(rows):
        for c in range(cols):
            if city_grid[r][c] == symbol:
                return (r, c)

def reconstruct(came_from, start, goal):
    current = goal
    path = [current]
    while current != start:
        current = came_from.get(current)
        if current is None:
            return []
        path.append(current)
    path.reverse()
    return path

start = find_pos('R')
goal = find_pos('C')

# ---------------------- GBFS ----------------------
def gbfs_food(start, goal):
    open_set = [(manhattan(start, goal), start)]
    came_from = {}
    visited = set()
    nodes_explored = 0

    while open_set:
        _, current = heapq.heappop(open_set)
        nodes_explored += 1
        if current == goal:
            break
        visited.add(current)
        for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
            nr, nc = current[0] + dr, current[1] + dc
            if 0 <= nr < rows and 0 <= nc < cols:
                if (nr, nc) not in visited and city_grid[nr][nc] != 'X':
                    came_from[(nr, nc)] = current
                    heapq.heappush(open_set, (manhattan((nr, nc), goal), (nr, nc)))
    return reconstruct(came_from, start, goal), nodes_explored

# ---------------------- A* ----------------------
def astar_food(start, goal):
    open_set = [(manhattan(start, goal), 0, start)]
    came_from = {}
    g_score = {start: 0}
    nodes_explored = 0

    while open_set:
        _, cost, current = heapq.heappop(open_set)
        nodes_explored += 1
        if current == goal:
            break
        for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
            nr, nc = current[0] + dr, current[1] + dc
            if 0 <= nr < rows and 0 <= nc < cols and city_grid[nr][nc] != 'X':
                neighbor = (nr, nc)
                tentative_g = g_score[current] + 1
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    came_from[neighbor] = current
                    f = tentative_g + manhattan(neighbor, goal)
                    heapq.heappush(open_set, (f, tentative_g, neighbor))
    return reconstruct(came_from, start, goal), nodes_explored

# ---------------------- RUN ----------------------
# GBFS
start_time = time.time()
gbfs_path, gbfs_nodes = gbfs_food(start, goal)
gbfs_time = (time.time() - start_time) * 1000

# A*
start_time = time.time()
astar_path, astar_nodes = astar_food(start, goal)
astar_time = (time.time() - start_time) * 1000

# Visualisasi Grid
vis_grid = [row[:] for row in city_grid]
for r, c in gbfs_path:
    if vis_grid[r][c] not in ('R', 'C'):
        vis_grid[r][c] = '*'

print("Grid Path (GBFS):")
for row in vis_grid:
    print(' '.join(row))

# Grid A* (opsional tampilkan juga)
vis_astar = [row[:] for row in city_grid]
for r, c in astar_path:
    if vis_astar[r][c] not in ('R', 'C'):
        vis_astar[r][c] = '*'

print("\nGrid Path (A*):")
for row in vis_astar:
    print(' '.join(row))

# ---------------------- OUTPUT ----------------------
print("\nComparing result (based on Time in millisecond)")
print("Assignment\tGBFS\t\tA star")
print(f"3\t\t{gbfs_time:.2f} ms\t{astar_time:.2f} ms\n")

print("Comparing result (based on number of nodes)")
print("Assignment\tGBFS\tA star")
print(f"3\t\t{gbfs_nodes}\t{astar_nodes}")
