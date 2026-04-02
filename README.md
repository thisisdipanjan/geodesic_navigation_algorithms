# Search-Based Global Planners for Nav2 (ROS 2)

This repository provides a comprehensive set of **search-based global planners** implemented as **Nav2 plugins** in ROS 2. It includes both **uninformed (blind)** and **informed (heuristic-based)** search algorithms, enabling experimentation, benchmarking, and understanding of classical planning strategies in robotic navigation.

---

## Implemented Algorithms

### Uninformed Search

* Breadth First Search (BFS)
* Depth First Search (DFS)
* Iterative Deepening Search (IDS)
* Uniform Cost Search (UCS)
* Dijkstra (cost-based baseline)

### Informed Search

* A* Search
* Greedy Best-First Search
* Beam Search
* Hybrid A*
* Iterative Deepening A* (IDA*)

---

## Overview

These planners operate on the **Nav2 costmap**, which represents the environment as a 2D grid:

* Each cell → node in a graph
* Neighboring cells → edges
* Costmap values → traversal cost / obstacle information

### Uninformed Search

* No heuristic guidance
* Explores the search space systematically
* Typically slower but conceptually simple

### Informed Search

* Uses heuristic functions to guide exploration
* Balances cost (`g(n)`) and heuristic (`h(n)`)
* Much more efficient for large environments

---

## Architecture

All planners implement the `nav2_core::GlobalPlanner` interface and are:

* Compiled into shared libraries (`.so`)
* Registered via `plugin.xml`
* Dynamically loaded by the Nav2 `planner_server`

---

Each package builds a **shared plugin library** containing multiple planner implementations.

---

## Execution Flow

1. Nav2 receives a navigation goal
2. `planner_server` selects a planner plugin
3. Plugin is loaded dynamically using `pluginlib`
4. The selected planner executes `createPlan(start, goal)`
5. A path (`nav_msgs/Path`) is returned
6. The controller executes the path

---

## Algorithm Characteristics

### Uninformed Search

| Algorithm | Complete | Optimal       | Notes                      |
| --------- | -------- | ------------- | -------------------------- |
| BFS       | yes        | yes (unit cost) | Level-order exploration    |
| DFS       | no        | no             | Fast but unreliable        |
| IDS       | yes        | yes             | DFS with depth limits      |
| UCS       | yes        | yes             | Cost-aware (like Dijkstra) |
| Dijkstra  | yes       | Slow             | No heuristic             |

---

### Informed Search

| Algorithm   | Optimal | Speed            | Notes                    |
| ----------- | ------- | ---------------- | ------------------------ |
| A*          | yes       | Fast             | Standard optimal planner |
| Greedy      | no       | Very Fast        | Heuristic-only           |
| Beam Search | no      | Faster           | Limited branching        |
| Hybrid A*   | yes       | Moderate         | Kinematic constraints    |
| IDA*        | yes       | Memory efficient | Iterative deepening      |

---

## Heuristics

Common heuristics used in informed planners:

* Euclidean Distance
* Manhattan Distance
* Costmap-aware heuristics (optional extensions)

---

## Limitations

* Grid-based planning produces non-smooth paths
* Performance depends on costmap resolution
* Poor heuristics can degrade informed search performance
* DFS and Beam Search may produce suboptimal paths

---

## Build Instructions

```bash
colcon build
source install/setup.bash
```

---

## Usage

Launch Nav2 normally and select planners via parameter configuration.
You can test using:

* RViz navigation goals
* ROS 2 action interface

---

## Future Work

* Path smoothing (splines, gradient methods)
* Heuristic tuning and adaptive strategies
* Benchmarking across different environments
* Integration with dynamic obstacles
* GPU-accelerated planning

