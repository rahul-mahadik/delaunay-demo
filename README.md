# Delaunay Triangulation Pathfinding Demo

This project demonstrates Delaunay triangulation and graph-based pathfinding over valid triangles within a polygonal domain. It provides an interactive interface to visualize computational geometry concepts like Delaunay triangulation, convex hulls, and centroid-based filtering.

---

## Computational Geometry Behind the Demo

### 1. **Delaunay Triangulation**
#### **Intuition**
Delaunay triangulation is a method to divide a set of points into triangles such that no point lies inside the circumcircle of any triangle. This maximizes the minimum angle of all triangles, producing a "well-shaped" mesh that avoids skinny triangles.

#### **What Happens Here?**
- We use **Bowyer-Watson's algorithm** (via Scipy's `Delaunay`) to compute the triangulation. This method:
  1. Starts with a large "super-triangle" that contains all points.
  2. Iteratively adds points, adjusting the triangulation to maintain the Delaunay property.
  3. Removes the super-triangle and any connected edges to leave the valid triangulation.
  
- **Key Properties**:
  - **Well-formed triangles**: Ensures numerical stability for centroid computation.
  - **Convex hull**: The triangulation inherently computes the convex hull of the point set.

- **Runtime**: \(O(n \log n)\), primarily due to sorting during convex hull computation.

---

### 2. **Valid Triangle Filtering**
#### **Intuition**
After generating the triangulation, not all triangles are useful for pathfinding. A triangle is considered "valid" if:
1. Its **centroid** lies inside the **boundary polygon**.
2. Its **centroid** lies outside any **obstacle polygons**.

#### **What Happens Here?**
- The **centroid** of each triangle is used as a representative point to check containment.
  - Containment is tested using **polygon-point inclusion checks** with the Shapely library.
  - Triangles that pass these checks are retained as valid.

- **Why Centroid?**
  - The centroid acts as a geometric proxy for the triangle's "belonging" to the domain.
  - Simplifies the inclusion tests compared to testing all vertices or edges.

- **Runtime**: \(O(t \cdot o)\), where \(t\) is the number of triangles and \(o\) is the number of obstacles.

---

### 3. **Graph Construction**
#### **Intuition**
Triangles in the Delaunay triangulation form a **dual graph**, where:
- Each triangle is a **node**.
- An **edge** connects two nodes if their corresponding triangles share a boundary edge.

#### **What Happens Here?**
- A **triangle adjacency graph** is constructed:
  - Nodes represent valid triangles.
  - Edges link adjacent valid triangles.
  
- This graph allows efficient traversal during pathfinding, reducing the problem to a **graph search**.

- **Runtime**: \(O(v + e)\), where \(v\) is the number of valid triangles and \(e\) is the number of edges between them.

---

### 4. **Pathfinding (BFS)**
#### **Intuition**
Pathfinding over the triangulated domain is achieved using **Breadth-First Search (BFS)**:
- Treat the triangle graph as a roadmap.
- Start from the triangle containing the start point and traverse until the triangle containing the endpoint is reached.

#### **What Happens Here?**
1. **Start and End Triangles**:
   - Locate which triangles contain the start and end points using **point-in-polygon** tests.
2. **BFS**:
   - Begin at the start triangle.
   - Explore neighbors level by level until the end triangle is found.
   - Track the path using a `came_from` dictionary.

- **Output**: A sequence of triangles representing the path.

- **Runtime**: \(O(v + e)\), as BFS visits all nodes and edges in the graph.

---

## How These Algorithms Fit Together
1. **Delaunay Triangulation**: Constructs a geometric structure to represent the domain.
2. **Triangle Filtering**: Extracts valid regions for traversal.
3. **Graph Construction**: Represents the connectivity of valid triangles.
4. **Pathfinding**: Finds a valid route between two points using the triangle graph.

---

## How to Use the Demo

### 1. **Setup**
   - Ensure you have Python installed.
   - Install dependencies using the provided `requirements.txt` file:
     ```bash
     pip install -r requirements.txt
     ```
   - Run the script:
     ```bash
     python demo.py
     ```

### 2. **Controls**
   - **Drawing Polygons**:
     - Press **`P`** to begin drawing a new polygon.
     - Left-click to place vertices.
     - Close the polygon by clicking near the first vertex.
   - **Start and End Points**:
     - Press **`S`** to place the start point, then click on the canvas.
     - Press **`E`** to place the end point, then click on the canvas.
   - **Resetting**:
     - Press **`R`** to reset the demo. This clears all user-defined polygons and points, but retains the bounding box.

### 3. **Visualization**
   - **Triangles**:
     - Valid triangles are shown in **semi-transparent orange**.
     - Path triangles are highlighted in **light green**.
   - **Polygons**:
     - Boundaries and obstacles are outlined in **black**.
     - Vertices are represented as **blue dots**.
   - **Start and End Points**:
     - The start point is marked in **red**, and the end point in **green**.
## Potential Applications
1. **Robotics**: Plan robot paths in polygonal environments.
2. **GIS**: Navigate geographic regions with obstacles.
3. **Computer Graphics**: Mesh generation and collision detection.
4. **Educational Tools**: Interactive learning for computational geometry concepts.
