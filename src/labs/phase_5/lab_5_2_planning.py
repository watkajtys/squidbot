"""
Lab 9.5: Path Planning (A*)
Goal: Find the shortest path through a 2D Voxel Grid.
Refer to docs/theory/Theory_9.5_Path_Planning_and_Search.md
"""

class GridPlanner:
    def __init__(self, grid):
        self.grid = grid # A 2D array where 0=empty, 1=wall

    def get_neighbors(self, node):
        """Return adjacent walkable cells."""
        pass

    def heuristic(self, a, b):
        """Manhattan or Euclidean distance."""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def a_star(self, start, goal):
        """
        Implement the A* search algorithm.
        Returns: A list of (x, y) coordinates.
        """
        # open_list = []
        # closed_list = set()
        
        # TODO: Implement the search logic
        
        return []
