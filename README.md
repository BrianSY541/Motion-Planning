# 3D Motion Planning with A\* and RRT

## 🔍 Project Overview

This project addresses the classical problem of motion planning in a continuous three-dimensional space populated with axis-aligned rectangular obstacles. The goal is to compute a collision-free path from a start to a goal configuration while analyzing the trade-offs between path optimality and computational performance. The project explores two complementary classes of planning algorithms: a custom-built, search-based Weighted A\* planner and several sampling-based planners (RRT, RRTConnect, and PRM) from the Open Motion Planning Library (OMPL).

## 🛠️ Technical Components

### 1️⃣ AABB Collision Checking

  * A robust and efficient line-segment vs. axis-aligned bounding box (AABB) collision checker was implemented based on the **slab method**.
  * The checker determines if a path segment intersects an obstacle by calculating and intersecting the segment's parametric intervals across the three "slabs" that form the bounding box.

### 2️⃣ Search-Based Planner: Weighted A\*

  * A search-based planner was developed using a **Weighted A\*** algorithm, which inflates the heuristic to accelerate the search at the cost of guaranteed optimality.
  * The free space is discretized into a uniform grid, and the planner searches for a path between voxel centers.
  * The heuristic weight, **ε (epsilon)**, is a key parameter used to tune the trade-off between planning time and path quality.

### 3️⃣ Sampling-Based Planners: OMPL

  * Three classical sampling-based planners—**RRT, RRTConnect, and PRM**—were integrated using the Open Motion Planning Library (OMPL) to benchmark against the custom A\* implementation.
  * **RRT (Rapidly-exploring Random Tree)** grows a tree from the start state by randomly sampling the space.
  * **RRTConnect** accelerates the search by simultaneously growing two trees, one from the start and one from the goal, and attempting to connect them.
  * **PRM (Probabilistic Roadmap)** builds a graph by sampling configurations in the free space and connecting nearby nodes, which can then be used to answer multiple path queries efficiently.

## 📂 Project Structure

```
.
├── code/
│   ├── astar.py              # Custom Weighted A* planner implementation
│   ├── collision_checker.py  # Slab-based AABB collision checker
│   ├── main.py               # Main script to run tests and visualizations
│   ├── Planner.py            # Wrapper class for all planning algorithms
│   └── maps/                 # Directory with map files
├── plot/
│   └── ...                   # Saved plots of resulting trajectories
└── report/
    └── ECE276B_Project2_Report.pdf # Project report
```

## 📈 Results & Performance

  * Across seven benchmark environments, the **Weighted A\*** planner with **ε = 2.5** was found to be the most effective choice for single-query tasks, achieving a dramatic reduction in planning time while only minimally impacting path length.
  * **RRTConnect** consistently found high-quality, short paths, particularly in complex maps like the "maze," but at a significantly higher computational cost.
  * **PRM** produced the longest paths in cluttered environments and had the highest runtime for a single query due to its roadmap construction phase.
  * The analysis highlights a key trade-off: informed, heuristic search methods like A\* are highly efficient in low-dimensional Euclidean spaces, while sampling-based methods offer advantages in more complex or high-dimensional problems.

## 🛠️ Technologies

  * **Python** for implementation.
  * **NumPy** for efficient numerical operations and vector calculations.
  * **Matplotlib** for 3D visualization of environments, obstacles, and paths.
  * **OMPL (Open Motion Planning Library)** for state-of-the-art sampling-based planning algorithms.

## 📚 Documentation

Detailed implementation, algorithm analysis, and results are available in the project report: [`report/ECE276B_Project2_Report.pdf`](https://www.google.com/search?q=report/ECE276B_Project2_Report.pdf).

---

## 📧 Contact
- **Brian (Shou-Yu) Wang**  
  - Email: briansywang541@gmail.com  
  - LinkedIn: [linkedin.com/in/sywang541](https://linkedin.com/in/sywang541)
  - GitHub: [BrianSY541](https://github.com/BrianSY541)

---

**Project developed as part of ECE 276B: Planning & Learning in Robotics at UC San Diego.**
