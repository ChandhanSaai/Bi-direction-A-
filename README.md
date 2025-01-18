# Bidirectional A* Pathfinding with Obstacle Avoidance

This repository contains an implementation of the Bidirectional A* search algorithm for finding the shortest path between two points in a 2D space with obstacle avoidance. The obstacles are modeled as circles, and the algorithm dynamically visualizes the search process.

## Features

- Implements Bidirectional A* for efficient pathfinding.
- Dynamically visualizes the search and the shortest path on a 2D obstacle map.
- Handles circular obstacles with customizable positions and radii.
- Calculates movement costs using Euclidean distance.
- Supports eight-directional movement, including diagonals.

## Prerequisites

This implementation requires the following Python libraries:

- `numpy`
- `opencv-python`
- `math`

You can install the required dependencies using pip:

```bash
pip install numpy opencv-python
```

## How It Works

1. **Obstacle Map:**
   - A 2D map is initialized with user-defined dimensions.
   - Circular obstacles are drawn on the map.

2. **Bidirectional A* Search:**
   - Two priority queues are maintained for forward and backward searches.
   - Neighbors of each point are explored in both directions, and the costs are updated dynamically.
   - If the search fronts meet, the shortest path is reconstructed.

3. **Visualization:**
   - The exploration process is visualized dynamically using OpenCV.
   - The final path is highlighted on the map once found.

## Usage

### 1. Clone the Repository

```bash
git clone <repository_url>
cd <repository_name>
```

### 2. Run the Code

```bash
python bidirectional_a_star.py
```

### 3. Visualize the Path

- The search process and the final path will be displayed in an OpenCV window.
- Press any key to close the window after the visualization is complete.

## Customization

### Obstacle Configuration

You can define custom circular obstacles by modifying the `is_valid_point` function and the `cv2.circle` calls in the code:

```python
# Example obstacle definition
cv2.circle(obstacle_map, (224, 115), 80, (128, 128, 128), -1)  # (x, y), radius, color
```

### Start and Goal Points

Modify the start and goal points:

```python
start_point = (0, 300)
goal_point = (1200, 300)
```

### Map Dimensions

Change the dimensions of the map:

```python
width = 1200
height = 600
```

## Output

The code produces a visualization where:

- **Gray circles** represent obstacles.
- **Orange points** show the shortest path.
- **Green points** indicate explored nodes.

## Example Visualization

![Example Visualization](example_visualization.png)
