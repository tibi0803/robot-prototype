#Breadth-First Search (BFS) for Pathfinding:
from collections import deque

def bfs(graph, start, goal):
    queue = deque([(start, [start])])
    visited = set()
    
    while queue:
        (vertex, path) = queue.popleft()
        if vertex in visited:
            continue
        
        visited.add(vertex)
        
        for neighbor, direction in graph[vertex].items():
            if neighbor == goal:
                return path + [neighbor]
            
            queue.append((neighbor, path + [neighbor]))
    return None

path = bfs(graph, 'A', 'E')
print("Path from A to E:", path)

#Navigate the Path:
def navigate_path(path):
    current_position = path[0]
    for next_position in path[1:]:
        direction = graph[current_position][next_position]
        print(f"Move {direction} to {next_position}")
        # Add motor control logic here based on the direction
        if direction == 'forward':
            move_forward()
        elif direction == 'backward':
            move_backward()
        elif direction == 'left':
            turn_left()
            move_forward()
        elif direction == 'right':
            turn_right()
            move_forward()
        
        # Add intersection detection and counting here
        while not detect_intersection():
            follow_line()
        
        current_position = next_position
        utime.sleep(1)  # Pause briefly at each intersection

navigate_path(path)

#Main Navigation Logic:
path = bfs(graph, 'A', 'E')
print("Path from A to E:", path)
navigate_path(path)

#Reverse the Path:
path_to_E = bfs(graph, 'A', 'E')
print("Path from A to E:", path_to_E)

path_to_A = path_to_E[::-1]
print("Path from E to A:", path_to_A)

#Adjust Navigation Function:
def navigate_path(path, reverse=False):
    current_position = path[0]
    for next_position in path[1:]:
        if reverse:
            direction = get_reverse_direction(graph[next_position][current_position])
        else:
            direction = graph[current_position][next_position]
            
        print(f"Move {direction} to {next_position}")
        # Add motor control logic here based on the direction
        if direction == 'forward':
            move_forward()
        elif direction == 'backward':
            move_backward()
        elif direction == 'left':
            turn_left()
            move_forward()
        elif direction == 'right':
            turn_right()
            move_forward()
        
        # Add intersection detection and counting here
        while not detect_intersection():
            follow_line()
        
        current_position = next_position
        utime.sleep(1)  # Pause briefly at each intersection

def get_reverse_direction(direction):
    if direction == 'forward':
        return 'backward'
    elif direction == 'backward':
        return 'forward'
    elif direction == 'left':
        return 'right'
    elif direction == 'right':
        return 'left'
    return direction

#Integrate with Main Logic:
# Navigate from A to E
navigate_path(path_to_E)

# Navigate back from E to A
navigate_path(path_to_A, reverse=True)

#-----------------------------------------------------------------------------------------------
#"complete code"
from collections import deque
import utime

# Define motor control functions
def move_forward():
    enable_motor1.duty(512)
    enable_motor2.duty(512)
    pin1_motor1.value(1)
    pin2_motor1.value(0)
    pin1_motor2.value(1)
    pin2_motor2.value(0)

def move_backward():
    enable_motor1.duty(512)
    enable_motor2.duty(512)
    pin1_motor1.value(0)
    pin2_motor1.value(1)
    pin1_motor2.value(0)
    pin2_motor2.value(1)

def turn_left():
    enable_motor1.duty(512)
    enable_motor2.duty(512)
    pin1_motor1.value(0)
    pin2_motor1.value(1)
    pin1_motor2.value(1)
    pin2_motor2.value(0)
    utime.sleep(1)

def turn_right():
    enable_motor1.duty(512)
    enable_motor2.duty(512)
    pin1_motor1.value(1)
    pin2_motor1.value(0)
    pin1_motor2.value(0)
    pin2_motor2.value(1)
    utime.sleep(1)

# Define intersection detection
def detect_intersection():
    s1 = sensor1.read()
    s2 = sensor2.read()
    s3 = sensor3.read()
    s4 = sensor4.read()
    s5 = sensor5.read()

    if s1 < SENSOR_THRESHOLD and s2 < SENSOR_THRESHOLD and s3 < SENSOR_THRESHOLD and s4 < SENSOR_THRESHOLD and s5 < SENSOR_THRESHOLD:
        return True
    return False

# Define the graph
graph = {
    'A': {'B': 'forward'},
    'B': {'A': 'backward', 'C': 'right', 'D': 'left'},
    'C': {'B': 'left', 'E': 'forward'},
    'D': {'B': 'right', 'E': 'forward'},
    'E': {'C': 'backward', 'D': 'backward'}
}

# BFS for pathfinding
def bfs(graph, start, goal):
    queue = deque([(start, [start])])
    visited = set()
    
    while queue:
        (vertex, path) = queue.popleft()
        if vertex in visited:
            continue
        
        visited.add(vertex)
        
        for neighbor, direction in graph[vertex].items():
            if neighbor == goal:
                return path + [neighbor]
            
            queue.append((neighbor, path + [neighbor]))
    return None

# Navigate the path
def navigate_path(path, reverse=False):
    current_position = path[0]
    for next_position in path[1:]:
        if reverse:
            direction = get_reverse_direction(graph[next_position][current_position])
        else:
            direction = graph[current_position][next_position]
            
        print(f"Move {direction} to {next_position}")
        # Add motor control logic here based on the direction
        if direction == 'forward':
            move_forward()
        elif direction == 'backward':
            move_backward()
        elif direction == 'left':
            turn_left()
            move_forward()
        elif direction == 'right':
            turn_right()
            move_forward()
        
        # Add intersection detection and counting here
        while not detect_intersection():
            follow_line()
        
        current_position = next_position
        utime.sleep(1)  # Pause briefly at each intersection

def get_reverse_direction(direction):
    if direction == 'forward':
        return 'backward'
    elif direction == 'backward':
        return 'forward'
    elif direction == 'left':
        return 'right'
    elif direction == 'right':
        return 'left'
    return direction

# Main logic
path_to_E = bfs(graph, 'A', 'E')
print("Path from A to E:", path_to_E)
navigate_path(path_to_E)

path_to_A = path_to_E[::-1]
print("Path from E to A:", path_to_A)
navigate_path(path_to_A, reverse=True)
