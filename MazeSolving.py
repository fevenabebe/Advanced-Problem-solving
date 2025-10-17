from pyamaze import maze, agent, textLabel
from queue import PriorityQueue
from tabulate import tabulate
import time
import random

def h(cell1, cell2):
    """Manhattan distance heuristic for A* and Greedy Best-First Search."""
    return abs(cell1[0] - cell2[0]) + abs(cell1[1] - cell2[1])

def move(current_cell, direction):
    """Return the new cell based on the current cell and direction."""
    if direction == 'E':
        return (current_cell[0], current_cell[1] + 1)
    elif direction == 'W':
        return (current_cell[0], current_cell[1] - 1)
    elif direction == 'N':
        return (current_cell[0] - 1, current_cell[1])
    elif direction == 'S':
        return (current_cell[0] + 1, current_cell[1])

def calculate_path_statistics(path):
    """Calculate statistics for the path such as length and turns."""
    length = len(path)
    turns = 0
    previous_direction = None

    for (cell1, cell2) in zip(list(path.keys()), list(path.values())):
        if cell1[0] == cell2[0]:  # Same row (E/W movement)
            current_direction = 'E' if cell2[1] > cell1[1] else 'W'
        else:  # Same column (N/S movement)
            current_direction = 'S' if cell2[0] > cell1[0] else 'N'

        if previous_direction and current_direction != previous_direction:
            turns += 1
        previous_direction = current_direction

    return length, turns

def DFS(m, start, goal):
    """Depth-First Search Algorithm."""
    explored = [start]
    frontier = [start]
    dfsPath = {}

    while frontier:
        currCell = frontier.pop()
        if currCell == goal:
            break

        for d in 'ESNW':
            if m.maze_map[currCell][d]:
                childCell = move(currCell, d)

                if childCell in explored:
                    continue

                explored.append(childCell)
                frontier.append(childCell)
                dfsPath[childCell] = currCell

    fwdPath = {}
    cell = goal
    while cell != start:
        fwdPath[dfsPath[cell]] = cell
        cell = dfsPath[cell]

    return fwdPath

def BFS(m, start, goal):
    """Breadth-First Search Algorithm."""
    frontier = [start]
    explored = [start]
    bfsPath = {}

    while frontier:
        currCell = frontier.pop(0)
        if currCell == goal:
            break

        for d in 'ESNW':
            if m.maze_map[currCell][d]:
                childCell = move(currCell, d)

                if childCell in explored:
                    continue

                frontier.append(childCell)
                explored.append(childCell)
                bfsPath[childCell] = currCell

    fwdPath = {}
    cell = goal
    while cell != start:
        fwdPath[bfsPath[cell]] = cell
        cell = bfsPath[cell]

    return fwdPath

def AStar(m, start, goal):
    """A* Algorithm."""
    g_score = {cell: float('inf') for cell in m.grid}
    g_score[start] = 0
    f_score = {cell: float('inf') for cell in m.grid}
    f_score[start] = h(start, goal)

    open_set = PriorityQueue()
    open_set.put((f_score[start], start))
    aPath = {}

    while not open_set.empty():
        currCell = open_set.get()[1]
        if currCell == goal:
            break

        for d in 'ESNW':
            if m.maze_map[currCell][d]:
                childCell = move(currCell, d)

                new_cost = g_score[currCell] + 1
                temp_f_score = new_cost + h(childCell, goal)

                if temp_f_score < f_score[childCell]:
                    g_score[childCell] = new_cost
                    f_score[childCell] = temp_f_score
                    open_set.put((temp_f_score, childCell))
                    aPath[childCell] = currCell

    fwdPath = {}
    cell = goal
    while cell != start:
        if cell not in aPath:
            return {}
        fwdPath[aPath[cell]] = cell
        cell = aPath[cell]

    return fwdPath

def greedy_best_first_search(m, start, goal):
    """Greedy Best-First Search Algorithm."""
    open_set = PriorityQueue()
    open_set.put((h(start, goal), start))
    came_from = {}
    explored = set()  # Use a set to track explored nodes

    while not open_set.empty():
        currCell = open_set.get()[1]
        if currCell == goal:
            break

        explored.add(currCell)  # Mark the current cell as explored

        for d in 'ESNW':
            if m.maze_map[currCell][d]:
                childCell = move(currCell, d)
                if childCell not in explored and childCell not in came_from:
                    came_from[childCell] = currCell
                    open_set.put((h(childCell, goal), childCell))

    # Reconstruct path
    fwdPath = {}
    cell = goal
    while cell in came_from:
        fwdPath[came_from[cell]] = cell
        cell = came_from[cell]

    return fwdPath

def dijkstra(m, start, goal):
    """Dijkstra's Algorithm."""
    g_score = {cell: float('inf') for cell in m.grid}
    g_score[start] = 0

    open_set = PriorityQueue()
    open_set.put((g_score[start], start))
    dijkstraPath = {}

    while not open_set.empty():
        currCell = open_set.get()[1]
        if currCell == goal:
            break

        for d in 'ESNW':
            if m.maze_map[currCell][d]:
                childCell = move(currCell, d)
                new_cost = g_score[currCell] + 1

                if new_cost < g_score[childCell]:
                    g_score[childCell] = new_cost
                    open_set.put((new_cost, childCell))
                    dijkstraPath[childCell] = currCell

    fwdPath = {}
    cell = goal
    while cell != start:
        if cell not in dijkstraPath:
            return {}
        fwdPath[dijkstraPath[cell]] = cell
        cell = dijkstraPath[cell]

    return fwdPath

def main():
    print("Choose maze generation method:")
    print("1: Generate random maze")
    print("2: Define number of rows and columns")

    generation_choice = input("Enter the number of your choice: ")

    if generation_choice == '1':
        rows = random.randint(5, 20)  # Random rows between 5 and 20
        cols = random.randint(5, 20)  # Random columns between 5 and 20
        print(f"Generated maze with {rows} rows and {cols} columns.")
    elif generation_choice == '2':
        rows = int(input("Enter number of rows for the maze: "))
        cols = int(input("Enter number of columns for the maze: "))
    else:
        print("Invalid choice")
        return

    m = maze(rows, cols)
    m.CreateMaze()

    start = (m.rows, m.cols)
    goal = (1, 1)

    print("Choose an algorithm:")
    print("1: Depth-First Search (DFS)")
    print("2: Breadth-First Search (BFS)")
    print("3: A* Algorithm")
    print("4: Greedy Best-First Search")
    print("5: Dijkstra's Algorithm")
    
    choice = input("Enter the number of your choice: ")

    # Start timing the execution
    start_time = time.time()

    if choice == '1':
        path = DFS(m, start, goal)
    elif choice == '2':
        path = BFS(m, start, goal)
    elif choice == '3':
        path = AStar(m, start, goal)
    elif choice == '4':
        path = greedy_best_first_search(m, start, goal)
    elif choice == '5':
        path = dijkstra(m, start, goal)
    else:
        print("Invalid choice")
        return

    # Stop timing the execution
    end_time = time.time()
    execution_time = end_time - start_time

    # Create an agent for visualization
    a = agent(m, start[0], start[1])

    # Trace the path if found
    if path:
        m.tracePath({a: dict(path)})
        
        # Calculate path statistics
        length, turns = calculate_path_statistics(path)
        
        # Prepare data for table visualization
        table_data = [["Cell", "Predecessor"]] + [[str(cell), str(predecessor)] for cell, predecessor in path.items()]
        
        # Print the sorted path as a table
        print("\nPath found:")
        print(tabulate(table_data, headers="firstrow", tablefmt="grid"))
        print(f"\nPath Length: {length} steps")
        print(f"Number of Turns: {turns}")
        print(f"Execution Time: {execution_time:.4f} seconds")
    else:
        print("No path found!")

    # Run the maze visualization
    m.run()

if __name__ == '__main__':
    main()