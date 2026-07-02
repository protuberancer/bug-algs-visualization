# Mobile Object Trajectory Generator on a Polygon

Implementation and comparative analysis of the **Bug** family of algorithms (Bug0, Bug1, Bug2) for mobile robot path planning in a two-dimensional space with obstacles.

## Description

The Bug family of algorithms are simple and effective methods for planning a mobile robot's path in an unknown environment, using only local information about the surroundings (without building a full map). The robot moves straight toward the goal, and upon encountering an obstacle, switches to a boundary-following mode.

Three classic algorithms are implemented in this project:

- **Bug0** — moves toward the goal; upon hitting an obstacle, follows its boundary until the path to the goal is clear again.
- **Bug1** — fully circumnavigates the obstacle while remembering the point closest to the goal, then returns to that point and continues toward the goal.
- **Bug2** — follows the obstacle boundary until it intersects the line connecting the start and the goal (the M-line), then continues moving along that line.

The polygon is defined as a plane bounded by a closed polyline, and obstacles are static polygons inside it.

## Features

- Generates a mobile object's trajectory on a polygon with arbitrary obstacles
- Obstacle avoidance in both clockwise and counterclockwise directions
- Visualization of the motion process and the resulting trajectory using `matplotlib`
- Tested on several scenarios: polygon with no obstacles, with one obstacle, with multiple obstacles, and with narrow passages

## Repository Structure

```
.
├── bug0.py          # Bug0 algorithm implementation
├── bug1.py          # Bug1 algorithm implementation
├── bug2.py          # Bug2 algorithm implementation
├── images/          # Trajectory visualization screenshots (1.png ... 16.png)
├── README.md
└── README_ru.md
```

## Requirements

- Python 3.8+
- `matplotlib`

Install dependencies:

```bash
pip install matplotlib
```

## Usage

Each algorithm is run as a separate script:

```bash
python bug0.py
python bug1.py
python bug2.py
```

Inside each script you can configure:

- the polygon boundary and obstacle coordinates;
- the start and goal points of the route;
- the robot's step size (`stepSize`) and dimensions (`robotSize`);
- the obstacle-following direction (`rotation`: `1` — clockwise, `-1` — counterclockwise).

After running, a plot shows the robot's motion from the start point to the goal while avoiding obstacles.

## Core Methods

| Method | Purpose |
|---|---|
| `find_nearest_side_index(point, path)` | finds the index of the polygon side closest to a point |
| `find_side_vector(side_index, path)` | computes the direction vector of a given polygon side |
| `find_vector_to_nearest_side(point, path)` | returns the direction of travel along the nearest polygon side |
| `at_vertex(robot_pos, obstacle)` | checks whether the robot is at an obstacle vertex |
| `main()` | main loop: environment setup, movement toward the goal, obstacle avoidance, visualization |

## Testing

The algorithms were tested on four types of polygons:

1. Simple polygon with no obstacles
2. Polygon with one obstacle
3. Polygon with multiple obstacles
4. Polygon with narrow passages

### Visualization Results

**1. Simple polygon with no obstacles**

![Bug0, Bug1, Bug2. Polygon 1](images/1.png)

**2. Polygon with one obstacle**

| Bug0 (clockwise) | Bug0 (counterclockwise) |
|---|---|
| ![Bug0 clockwise. Polygon 2](images/2.png) | ![Bug0 counterclockwise. Polygon 2](images/3.png) |

![Bug1. Polygon 2](images/4.png)

| Bug2 (clockwise) | Bug2 (counterclockwise) |
|---|---|
| ![Bug2 clockwise. Polygon 2](images/5.png) | ![Bug2 counterclockwise. Polygon 2](images/6.png) |

**3. Polygon with multiple obstacles**

| Bug0 (counterclockwise) | Bug0 (clockwise) |
|---|---|
| ![Bug0 counterclockwise. Polygon 3](images/7.png) | ![Bug0 clockwise. Polygon 3](images/8.png) |

![Bug1. Polygon 3](images/9.png)

| Bug2 (clockwise) | Bug2 (counterclockwise) |
|---|---|
| ![Bug2 clockwise. Polygon 3](images/10.png) | ![Bug2 counterclockwise. Polygon 3](images/11.png) |

**4. Polygon with narrow passages**

| Bug0 (clockwise) | Bug0 (counterclockwise) |
|---|---|
| ![Bug0 clockwise. Polygon 4](images/12.png) | ![Bug0 counterclockwise. Polygon 4](images/13.png) |

![Bug1. Polygon 4](images/14.png)

| Bug2 (clockwise) | Bug2 (counterclockwise) |
|---|---|
| ![Bug2 clockwise. Polygon 4](images/16.png) | ![Bug2 counterclockwise. Polygon 4](images/15.png) |

### Comparison by Criteria

Comparison by **efficiency**, **reliability** (resistance to looping), and **memory consumption**:

| Criterion | Bug0 | Bug1 | Bug2 |
|---|---|---|---|
| Efficiency | 1 | 3 | 2 |
| Reliability | 3 | 1 | 2 |
| Memory consumption | 1 | 3 | 2 |

*(1 — best result, 3 — worst)*

**Conclusions:** Bug0 is the fastest thanks to its simple logic, but risks looping in complex environments. Bug1 is the most reliable (looping is impossible), but requires the most memory and produces a longer path due to full obstacle circumnavigation. Bug2 falls in between on all metrics.

## Future Plans

- Add heuristics to reduce the length of the resulting trajectory
- Integrate with other path-planning methods (e.g., potential fields)
- Support for dynamic obstacles

## References

1. V. Lumelsky, A. Stepanov. *Dynamic path planning for a mobile automaton with limited information on the environment*. IEEE Transactions on Automatic Control, 1986.
2. K. McGuire, G. Croon, K. Tuyls. *A Comparative Study of Bug Algorithms for Robot Navigation*. 2018.
3. Y. Zhu, T. Zhang, J. Song, X. Li. *A new Bug-type navigation algorithm for mobile robots*. IEEE ICRB, 2010.
4. S. Gupta, C. S. Asha, J. M. D'Souza. *Implementation and Comparison of BUG Algorithms on ROS*. INOCON, 2023.
5. Y. Zhu, Z. Tao, J. Song, X. Li. *A new bug-type navigation algorithm for mobile robots in unknown environments containing moving obstacles*. Industrial Robot, 2012.
6. K. McGuire et al. *Minimal navigation solution for a swarm of tiny flying robots to explore an unknown environment*. Science Robotics, 2019.
