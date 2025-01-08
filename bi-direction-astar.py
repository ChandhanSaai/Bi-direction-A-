import numpy as np
import cv2
import math
from queue import PriorityQueue

# Function to map coordinates from (x, y) to (x, height - y)
def map_coordinates(x, y, width, height):
    return x, height - y

# Function to calculate the Euclidean distance between two points
def calculate_distance(p1, p2):
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

# Function to get valid neighbors of a given point
def get_valid_neighbors(point):
    global explored_count
    valid_neighbors = []
    x, y = point
    # Define the possible moves (including diagonals)
    moves = [(1, 0), (0, 1), (-1, 0), (0, -1), (1, 1), (-1, 1), (1, -1), (-1, -1)]
    for dx, dy in moves:
        new_point = (x + dx, y + dy)
        if is_valid_point(new_point):
            valid_neighbors.append(new_point)
            # Visualize the explored points on the obstacle map
            cv2.circle(obstacle_map, (new_point[0], new_point[1]), 2, (124, 163, 138), -1)
            if explored_count % 100 == 0:
                cv2.imshow("Obstacle Map", obstacle_map)
                cv2.waitKey(1)
            explored_count += 1
    return valid_neighbors

# Function to check if a point is inside a circle
def is_point_in_circle(point, center, radius):
    x, y = point
    cx, cy = center
    return (x - cx) ** 2 + (y - cy) ** 2 <= radius ** 2

# Function to check if a point is valid (not inside any obstacle circle)
def is_valid_point(point):
    if is_point_in_circle(point, (224, 115), 80):
        return False
    if is_point_in_circle(point, (890, 160), 75):
        return False
    if is_point_in_circle(point, (526, 420), 140):
        return False
    return True

# Function to calculate the movement cost between two points (Euclidean distance)
def movement_cost(current, next):
    return calculate_distance(current, next)

# Function to perform bidirectional A* search
def bidirectional_a_star(start, goal):
    # Initialize priority queues for forward and backward search
    forward_queue = PriorityQueue()
    backward_queue = PriorityQueue()
    forward_queue.put((0, start))
    backward_queue.put((0, goal))

    # Initialize cost and path dictionaries for forward and backward search
    forward_cost = {start: 0}
    backward_cost = {goal: 0}
    forward_path = {start: None}
    backward_path = {goal: None}
    intersection_point = None

    while not forward_queue.empty() and not backward_queue.empty():
        # Get the current nodes from both queues
        _, forward_current = forward_queue.get()
        _, backward_current = backward_queue.get()

        # Check if the current nodes intersect
        if forward_current in backward_cost:
            intersection_point = forward_current
            break
        if backward_current in forward_cost:
            intersection_point = backward_current
            break

        # Explore neighbors for forward search
        for neighbor in get_valid_neighbors(forward_current):
            new_cost = forward_cost[forward_current] + movement_cost(forward_current, neighbor)
            if neighbor not in forward_cost or new_cost < forward_cost[neighbor]:
                forward_cost[neighbor] = new_cost
                priority = new_cost + calculate_distance(neighbor, goal)
                forward_queue.put((priority, neighbor))
                forward_path[neighbor] = forward_current

        # Explore neighbors for backward search
        for neighbor in get_valid_neighbors(backward_current):
            new_cost = backward_cost[backward_current] + movement_cost(backward_current, neighbor)
            if neighbor not in backward_cost or new_cost < backward_cost[neighbor]:
                backward_cost[neighbor] = new_cost
                priority = new_cost + calculate_distance(neighbor, start)
                backward_queue.put((priority, neighbor))
                backward_path[neighbor] = backward_current

    # If an intersection point is found, reconstruct the path
    if intersection_point is not None:
        forward_part = []
        backward_part = []
        node = intersection_point
        while node != start:
            forward_part.append(node)
            node = forward_path[node]
        forward_part.append(start)
        forward_part.reverse()

        node = intersection_point
        while node != goal:
            backward_part.append(node)
            node = backward_path[node]
        backward_part.append(goal)

        return forward_part + backward_part[1:]
    return []

# Define the dimensions of the obstacle map
width = 1200
height = 600
explored_count = 0

# Create a white obstacle map
obstacle_map = np.ones((height, width, 3), dtype=np.uint8) * 255

# Draw the obstacle circles on the map
cv2.circle(obstacle_map, (224, 115), 80, (128, 128, 128), -1)
cv2.circle(obstacle_map, (890, 160), 75, (128, 128, 128), -1)
cv2.circle(obstacle_map, (526, 420), 140, (128, 128, 128), -1)

# Define the start and goal points
start_point = (0, 300)
goal_point = (1200, 300)

# Find the shortest path using bidirectional A* search
shortest_path = bidirectional_a_star(start_point, goal_point)

# Visualize the shortest path on the obstacle map
for point in shortest_path:
    cv2.circle(obstacle_map, (point[0], point[1]), 2, (234, 146, 79), -1)
    cv2.imshow("Obstacle Map", obstacle_map)
    cv2.waitKey(1)

# Display the final obstacle map
cv2.imshow("Obstacle Map", obstacle_map)

# Wait for a key press and close all windows
cv2.waitKey(0)
cv2.destroyAllWindows()