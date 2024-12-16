import pygame
import sys
import math
import numpy as np
from scipy.spatial import Delaunay
from typing import List, Tuple
from shapely.geometry import Polygon, Point
from collections import defaultdict
from queue import Queue, PriorityQueue
import heapq
import asyncio

# Initialize Pygame
pygame.init()

# Set up the display
WIDTH = 800
HEIGHT = 600
screen = pygame.display.set_mode((WIDTH, HEIGHT), pygame.RESIZABLE)
pygame.display.set_caption("Delaunay Demo")

async def main():
    running = True
    while running:
        # Handle events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            
        # Clear screen
        screen.fill((255, 255, 255))  # White background
        
        # Draw here
        pygame.draw.circle(screen, (0, 0, 255), (WIDTH//2, HEIGHT//2), 30)
        
        # Update display
        pygame.display.flip()
        
        # Control frame rate
        await asyncio.sleep(0)  # Required for Pygbag

asyncio.run(main())
# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
VERTEX_COLOR = (100, 149, 237)        # Cornflower blue
TRIANGLE_COLOR = (255, 140, 0, 128)   # Semi-transparent orange
EDGE_COLOR = (178, 34, 34)            # Firebrick red
GRID_COLOR = (220, 220, 220)          # Light grey

# Screen dimensions and grid
WIDTH, HEIGHT = 800, 600
GRID_SIZE = 20

class DelaunayDemo:
    def __init__(self):
        self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
        pygame.display.set_caption("Delaunay Triangulation Demo")
        
        self.polygons = []          # List of polygons (first is boundary)
        self.current_polygon = []    # Current polygon being drawn
        self.building_polygon = False
        self.triangulation = None    # Store Delaunay triangulation
        self.valid_triangles = []    # Store only the valid triangles
        self.triangle_graph = defaultdict(list)  # Store adjacency information
        self.start_point = None
        self.end_point = None
        self.path = []  # Store path of triangles
        self.placing_start = False
        self.placing_end = False
        
        # Add bounding box
        self.add_bounding_box()

    def add_bounding_box(self):
        """Add initial bounding box."""
        margin = GRID_SIZE
        box = [
            (margin, margin),
            (WIDTH - margin, margin),
            (WIDTH - margin, HEIGHT - margin),
            (margin, HEIGHT - margin)
        ]
        self.polygons.append(box)

    def snap_to_grid(self, pos):
        """Snap position to grid."""
        x = round(pos[0] / GRID_SIZE) * GRID_SIZE
        y = round(pos[1] / GRID_SIZE) * GRID_SIZE
        return (x, y)

    def is_near_first_point(self, pos, threshold=20):
        """Check if position is near the first point of current polygon."""
        if not self.current_polygon:
            return False
        first_point = self.current_polygon[0]
        dist = math.hypot(pos[0] - first_point[0], pos[1] - first_point[1])
        return dist < threshold

    def compute_triangulation(self):
        """Compute triangulation using scipy's Delaunay."""
        if len(self.polygons) < 1:
            return

        # Collect vertices from boundary
        points = []
        for polygon in self.polygons:
            points.extend(polygon)
        points = np.array(points)

        # Compute Delaunay triangulation
        tri = Delaunay(points)
        
        # Filter triangles
        boundary = Polygon(self.polygons[0])
        obstacles = [Polygon(poly) for poly in self.polygons[1:]]
        
        self.valid_triangles = []
        valid_indices = []  # Keep track of valid triangle indices
        
        for i, simplex in enumerate(tri.simplices):
            triangle_points = points[simplex]
            centroid = np.mean(triangle_points, axis=0)
            
            # Check if centroid is inside boundary and outside obstacles
            if boundary.contains(Point(centroid)):
                valid = True
                for obstacle in obstacles:
                    if obstacle.contains(Point(centroid)):
                        valid = False
                        break
                if valid:
                    self.valid_triangles.append(triangle_points)
                    valid_indices.append(i)

        # Compute adjacency graph
        self.compute_triangle_graph(tri, valid_indices)
        
        # Recompute path if start and end points exist
        if self.start_point and self.end_point:
            self.find_path()

    def compute_triangle_graph(self, tri, valid_indices):
        """Create adjacency graph of valid triangles."""
        self.triangle_graph = defaultdict(list)
        
        # Create a mapping from original indices to valid triangle indices
        index_map = {orig: new for new, orig in enumerate(valid_indices)}
        
        # Get neighbors from Delaunay triangulation
        neighbors = tri.neighbors
        
        for new_idx, orig_idx in enumerate(valid_indices):
            # Get neighbors of current triangle
            for neighbor_idx in neighbors[orig_idx]:
                # Check if neighbor is valid and not -1 (which indicates no neighbor)
                if neighbor_idx in index_map:
                    self.triangle_graph[new_idx].append(index_map[neighbor_idx])
        
        print("\nTriangle Adjacency Graph:")
        for tri_idx, neighbors in self.triangle_graph.items():
            print(f"Triangle {tri_idx} is adjacent to triangles: {neighbors}")

    def find_containing_triangle(self, point):
        """Find which triangle contains the given point."""
        point = Point(point[0], point[1])
        for i, triangle in enumerate(self.valid_triangles):
            triangle_poly = Polygon(triangle)
            if triangle_poly.contains(point):
                return i
        return None

    def find_path(self):
        """Find path between start and end triangles using BFS."""
        if not self.start_point or not self.end_point:
            print("Need both start and end points!")
            return

        start_tri = self.find_containing_triangle(self.start_point)
        end_tri = self.find_containing_triangle(self.end_point)

        if start_tri is None or end_tri is None:
            print("Start or end point not in any triangle!")
            return

        print(f"Finding path from triangle {start_tri} to {end_tri}")

        # BFS
        queue = Queue()
        queue.put(start_tri)
        came_from = {start_tri: None}

        while not queue.empty():
            current = queue.get()
            
            if current == end_tri:
                break

            for next_tri in self.triangle_graph[current]:
                if next_tri not in came_from:
                    queue.put(next_tri)
                    came_from[next_tri] = current

        # Reconstruct path
        self.path = []
        current = end_tri
        while current is not None:
            self.path.append(current)
            current = came_from.get(current)
        self.path.reverse()
        
        print(f"Found path: {self.path}")

    def handle_click(self, pos):
        """Handle mouse clicks for polygon creation and point placement."""
        if self.placing_start:
            self.start_point = self.snap_to_grid(pos)
            self.placing_start = False
            if self.start_point and self.end_point:
                self.find_path()
        elif self.placing_end:
            self.end_point = self.snap_to_grid(pos)
            self.placing_end = False
            if self.start_point and self.end_point:
                self.find_path()
        elif self.building_polygon:
            snapped_pos = self.snap_to_grid(pos)
            
            if len(self.current_polygon) >= 3 and self.is_near_first_point(snapped_pos):
                self.polygons.append(self.current_polygon[:])
                self.current_polygon = []
                self.building_polygon = False
                self.compute_triangulation()
            else:
                self.current_polygon.append(snapped_pos)

    def draw(self):
        """Draw everything to the screen."""
        self.screen.fill(WHITE)
        
        # Draw grid
        for x in range(0, WIDTH, GRID_SIZE):
            pygame.draw.line(self.screen, GRID_COLOR, (x, 0), (x, HEIGHT))
        for y in range(0, HEIGHT, GRID_SIZE):
            pygame.draw.line(self.screen, GRID_COLOR, (0, y), (WIDTH, y))

        # Draw triangles
        for i, triangle in enumerate(self.valid_triangles):
            color = TRIANGLE_COLOR
            if i in self.path:
                # Highlight path triangles
                color = (144, 238, 144, 128)  # Light green
            pygame.draw.polygon(self.screen, color, triangle)
            pygame.draw.polygon(self.screen, EDGE_COLOR, triangle, 2)

        # Draw polygons
        for polygon in self.polygons:
            pygame.draw.lines(self.screen, BLACK, True, polygon, 2)
            for point in polygon:
                pygame.draw.circle(self.screen, VERTEX_COLOR, point, 4)

        # Draw current polygon
        if self.current_polygon:
            if len(self.current_polygon) >= 2:
                pygame.draw.lines(self.screen, BLACK, False, self.current_polygon, 2)
            for point in self.current_polygon:
                pygame.draw.circle(self.screen, VERTEX_COLOR, point, 4)

        # Draw start and end points
        if self.start_point:
            pygame.draw.circle(self.screen, (255, 0, 0), self.start_point, 6)  # Red
        if self.end_point:
            pygame.draw.circle(self.screen, (0, 255, 0), self.end_point, 6)  # Green

        pygame.display.flip()

    async def run(self):
        """Main game loop."""
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 1:  # Left click
                        self.handle_click(event.pos)
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_p:  # Start new polygon
                        self.building_polygon = True
                        self.current_polygon = []
                    elif event.key == pygame.K_s:  # Place start point
                        self.placing_start = True
                        self.placing_end = False
                    elif event.key == pygame.K_e:  # Place end point
                        self.placing_end = True
                        self.placing_start = False
                    elif event.key == pygame.K_r:  # Reset
                        self.polygons = [self.polygons[0]]  # Keep boundary
                        self.current_polygon = []
                        self.building_polygon = False
                        self.valid_triangles = []
                        self.start_point = None
                        self.end_point = None
                        self.path = []

            self.draw()
            await asyncio.sleep(0)  # Allow browser to update

        pygame.quit()

if __name__ == "__main__":
    demo = DelaunayDemo()
    asyncio.run(demo.run())
