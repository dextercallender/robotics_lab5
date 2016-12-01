from turtle import *
from math import *
import time
from sets import Set
import sys
#from gopigo import *
from matplotlib import path
import copy

start = ()
end = ()
dimensions = ()
obstacles = []
lower_rightmost_vertex = None
vertices = {}

def create_obstacles(input_file):
    '''
        Create the obstacles and world dimensions from the specified input file
    '''
    global start, end, obstacles, dimensions
    with open(input_file) as f:
        # Start
        str_start = f.readline().strip().split()
        start = Obstacle()
        start.vertices.append((float(str_start[0]), float(str_start[1])))
        # End
        str_end = f.readline().strip().split()
        end = Obstacle()
        end.vertices.append((float(str_end[0]), float(str_end[1])))
        # World Dimensions
        str_dim = f.readline().strip().split()
        dimensions = (float(str_dim[0]), float(str_dim[1]))
        # Obstacles
        num_obstacles = int(f.readline().strip())
        for i in range(0, num_obstacles):
            num_vertices = int(f.readline().strip())
            obstacle = Obstacle()
            for j in range(0, num_vertices):
                str_vertex = f.readline().strip().split()
                obstacle.vertices.append((float(str_vertex[0]),float(str_vertex[1])))
            obstacle.original_vertices = copy.deepcopy(obstacle.vertices)
            obstacles.append(obstacle)

class Obstacle:
    
    def __init__(self):
        '''
            Initialize an obstacle
        '''
        self.vertices =  []
        self.original_vertices = []

    def set_vertices(self, new_vertices):
        ''' Set the verticles of the obstacle '''
        self.vertices = new_vertices

def add_vertices(vertex1, vertex2):
    '''
        Add the two given vertices
    '''
    return (vertex1[0] + vertex2[0], vertex1[1] + vertex2[1])

def dist(vertex):
    '''
        Get the distance from this vertex to the lower_rightmost_vertex
    '''
    global lower_rightmost_vertex
    return sqrt((vertex[0] - lower_rightmost_vertex[0])**2 + (vertex[1] - lower_rightmost_vertex[1])**2)

def angle_comparison(vertex1, vertex2):
    '''
        Comparison for sorting by angle to the lower_rightmost_vertex
    '''
    global lower_rightmost_vertex
    vertex1_x = (vertex1[0]-lower_rightmost_vertex[0])
    vertex1_dist = dist(vertex1)
    vertex2_x = (vertex2[0]-lower_rightmost_vertex[0])
    vertex2_dist = dist(vertex2)
    vertex1_angle = vertex1_x / vertex1_dist
    vertex2_angle = vertex2_x / vertex2_dist
    vertex1
    # Sort by angle
    if vertex1_angle < vertex2_angle:
        return 1
    elif vertex1_angle > vertex2_angle:
        return -1
    # If angles to lower_rightmost_vertex is the same
    else:
        # Then favor closest vertex
        if dist(vertex1) < dist(vertex2):
            return -1
        elif dist(vertex1) > dist(vertex2):
            return 1
        return 0

def isLeftTurn(vertex1, vertex2, vertex3):
    '''
        Check if the given 3 vertices make a left turn
    '''
    value1 = (vertex2[0] - vertex1[0]) * (vertex3[1] - vertex1[1])
    value2 = (vertex2[1] - vertex1[1]) * (vertex3[0] - vertex1[0])
    if value1-value2 > 0:
        return True
    return False
    
def create_convex_hull(vertices):
    '''
        Create the convex hull from the given vertices
    '''
    global lower_rightmost_vertex
    stack = []
    if vertices and len(vertices) > 0:
        # Find lowest rightmost point
        lower_rightmost_vertex = (float("-inf"), float("inf"))
        for vertex in vertices:
            if vertex[1] < lower_rightmost_vertex[1]:
                lower_rightmost_vertex = vertex
            elif vertex[1] == lower_rightmost_vertex[1] \
                 and vertex[0] > lower_rightmost_vertex[0]:
                lower_rightmost_vertex = vertex
        vertices.remove(lower_rightmost_vertex)
        # Sort vertices by angle to lowest rightmost point
        vertices = list(vertices)
        vertices.sort(cmp=angle_comparison)
        # Add in the first and last points
        stack.append(vertices[len(vertices)-1])
        stack.append(lower_rightmost_vertex)
        # Loop through the vertices and add the ones that
        # will make a left turn
        for vertex in vertices:
            vertex2 = stack.pop()
            vertex1 = stack.pop()
            while not isLeftTurn(vertex1, vertex2, vertex):
                vertex2 = vertex1
                vertex1 = stack.pop()
            stack.append(vertex1)
            stack.append(vertex2)
            if not vertex in stack:
                stack.append(vertex)
    return stack

def grow_obstacles(robot_vertices):
    ''' Grow the obstacles by the given robot vertices '''
    global obstacles, dimensions
    # Loop through the vertices of each obstacle and 
    # add the robot to each one
    for obstacle in obstacles:
        new_vertices = Set([])
        for vertex in obstacle.vertices:
            for robot_vertex in robot_vertices:
                offset_vertex = add_vertices(vertex, robot_vertex)
                new_vertices.add(offset_vertex)
        # Create the convex hull from all the vertices
        obstacle.set_vertices(create_convex_hull(new_vertices))

class Vertex:
    def __init__(self, param_x, param_y, param_obstacle_num):
        ''' Initialize a vertex '''
        self.predecessor = None
        self.neighbors = []
        self.dist = float('inf')
        self.x = param_x
        self.y = param_y
        self.obstacle_num = param_obstacle_num

    def distance_to_neighbor(self, neighbor):
        ''' Distance between this vertex and the given neighbor '''
        return sqrt((self.x - neighbor.x)**2 + (self.y - neighbor.y)**2)

class Line_Segment:
    def __init__(self, param_x1, param_y1, param_x2, param_y2):
        ''' Initialize a line segment '''
        self.x1 = param_x1
        self.y1 = param_y1
        self.x2 = param_x2
        self.y2 = param_y2
        x_change = param_x2 - param_x1
        y_change = param_y2 - param_y1
        dist = sqrt((x_change)**2 + (y_change)**2)
        self.delta_x = x_change / dist
        self.delta_y = y_change / dist
        self.t = self.solve_t(param_x2, param_y2)
        self.slope = y_change / x_change if x_change != 0 else float('inf')
        self.b = param_y1 - self.slope * param_x1

    def solve_t(self, x2, y2):
        '''
            Given x and y solve for time to intersection
        '''
        return (x2 - self.x1) / self.delta_x \
               if self.delta_x != 0 else (y2 - self.y1) / self.delta_y

    def solve_x_y(self, t):
        '''
            Given a time to intersection solve for x and y
        '''
        result_x = self.delta_x * t + self.x1
        result_y = self.delta_y * t + self.y1
        return (result_x, result_y)
    
    def intersect_point(self, other_seg):
        '''
            Get the intersection between this line segment
            and the given line segment
        '''
        # Same slopes shouldn't intersect
        if other_seg.slope == self.slope:
            return None
        result_x = 0
        # Case if this segment is vertical
        if self.slope == float('inf'):
            result_x = self.x1
        # Case if this other segment is vertical
        elif other_seg.slope == float('inf'):
            result_x = other_seg.x1
        else:
            result_x = (self.b - other_seg.b) / (other_seg.slope - self.slope)
        result_y = self.slope * result_x + self.b if self.slope != float('inf') else other_seg.slope * result_x + other_seg.b
        return (result_x,result_y)

def inside_world_bounds(vertex):
    ''' Check if the given vertex is with the world dimensions/bounds '''
    return vertex[0] >= 0 and vertex[1] >= 0 \
           and vertex[0] <= dimensions[0] \
           and vertex[1] <= dimensions[1]

def construct_path_obstacles():
    '''
        Create path objects from the obstacles
    '''
    path_obstacles = []
    for obstacle in obstacles:
        path_obstacles.append(path.Path(obstacle.vertices))
    return path_obstacles

def inside_obstacle(vertex, path_obstacles, index):
    '''
        Check if the vertex is in any of the path obstacles
        with the exception of the obstacle at the given index
    '''
    for i in range(0, len(path_obstacles)):
        if i != index and path_obstacles[i].contains_points([vertex]):
            return True
    return False
    
def visibility_graph():
    ''' Draw the visibility graph '''
    global vertices, orange
    vertices = {}
    # Initialize variables
    path_obstacles = construct_path_obstacles()
    line_segs = []
    # Create a list of the line segments of the objects
    # and a dictionary of valid vertices
    for j in range(0, len(obstacles)):
        obstacle = obstacles[j]
        for i in range(0, len(obstacle.vertices)):
            # Add the line segment of the current vertex and the next vertex
            if len(obstacle.vertices) > 1:
                next_index = (i + 1) % len(obstacle.vertices)
                line_seg = Line_Segment(obstacle.vertices[i][0], obstacle.vertices[i][1], obstacle.vertices[next_index][0], obstacle.vertices[next_index][1])
                line_segs.append(line_seg)
            # Add the current vertex to the dictionary of vertices if it is new
            # not inside an object, or the world bounds
            if not obstacle.vertices[i] in vertices and not inside_obstacle(obstacle.vertices[i], path_obstacles, j) \
               and inside_world_bounds(obstacle.vertices[i]):
                vertices[obstacle.vertices[i]] = Vertex(obstacle.vertices[i][0], obstacle.vertices[i][1], j)
    # Attach each vertex to the ones before/after it in the obstacle
    for i in range(0, len(obstacles)):
        obstacle = obstacles[i]
        for j in range(0, len(obstacle.vertices)):
            if obstacle.vertices[j] in vertices:
                previous_index = (j - 1) % len(obstacle.vertices)
                next_index = (j + 1) % len(obstacle.vertices)
                if not previous_index == j and obstacle.vertices[previous_index] in vertices:
                    vertices[obstacle.vertices[j]].neighbors.append(vertices[obstacle.vertices[previous_index]])
                if not previous_index == next_index and not next_index == j and obstacle.vertices[next_index] in vertices:
                    vertices[obstacle.vertices[j]].neighbors.append(vertices[obstacle.vertices[next_index]])
    # For each vertex check all other vertices to see if they're visible
    for i in range(0, len(vertices.keys())):
        vertex1 = vertices.keys()[i]
        for j in range(i+1, len(vertices.keys())):
            vertex2 = vertices.keys()[j]
            line_seg = Line_Segment(vertex1[0], vertex1[1], vertex2[0], vertex2[1])
            valid = True
            # Make sure it's not the same obstacle
            if vertices[vertex1].obstacle_num == vertices[vertex2].obstacle_num:
                valid = False
            if valid:
                # Check if the line between the vertices intersects
                # the lines of any obstacle
                for obstacle_line_seg in line_segs:
                    # Get the intersection point
                    intersect = line_seg.intersect_point(obstacle_line_seg)
                    if intersect:
                        intersect_x = intersect[0]
                        intersect_y = intersect[1]
                        # Get the times to intersection for both line segments
                        obstacle_intersect_t = obstacle_line_seg.solve_t(intersect_x, intersect_y)
                        line_intersect_t = line_seg.solve_t(intersect_x, intersect_y)
                        # Variable for accommodate for floating point error
                        floating_error = 0.0000001
                        # Check that it lies on both line segments
                        if line_intersect_t > floating_error and \
                           line_intersect_t < line_seg.t - floating_error and \
                           obstacle_intersect_t > floating_error and \
                           obstacle_intersect_t < obstacle_line_seg.t - floating_error:
                            valid = False
                            break
            # If it's still valid, add the edge between the two vertices
            if valid:
                vertices[vertex1].neighbors.append(vertices[vertex2])
                vertices[vertex2].neighbors.append(vertices[vertex1])
                orange.setpos(vertex1[0], vertex1[1])
                orange.pendown()
                orange.setpos(vertex2[0], vertex2[1])
                orange.penup()
            
def dijkstra():
    ''' Run dijkstras '''
    # Start is the end
    if start.vertices[0] == end.vertices[0]:
        print "Total Distance To Goal: ", 0
        return [start.vertices[0]]
    # Initial start node
    vertices[start.vertices[0]].dist = 0
    unvisited = vertices.values()
    current_node = vertices[start.vertices[0]]
    # Continue while there are unvisited nodes
    while len(unvisited) > 0:
        # Mark this node as visited
        unvisited.remove(current_node)
        # Update distances of its neighbors
        for neighbor in current_node.neighbors:
            if neighbor in unvisited:
                potential_dist = current_node.distance_to_neighbor(neighbor) + current_node.dist
                if potential_dist <= neighbor.dist:
                    neighbor.dist = potential_dist
                    neighbor.predecessor = current_node
        # Get the next closest unvisited node 
        smallest_dist = float('inf')
        for node in unvisited:
            if node.dist <= smallest_dist:
                smallest_dist = node.dist
                current_node = node
    # Get the path by going through the predecessors
    path = []
    dist_to_start = vertices[end.vertices[0]].dist
    # End not reachable
    if dist_to_start == float('inf'):
        print "No Path Found"
        return []
    print "Total Distance To Goal: ", dist_to_start
    to_append = vertices[end.vertices[0]]
    while dist_to_start > 0:
        path.append((to_append.x, to_append.y))
        to_append = to_append.predecessor
        dist_to_start = to_append.dist
    path.append(start.vertices[0])
    path.reverse()
    # Plot the path
    purple = Turtle()
    purple.speed(0)
    purple.hideturtle()
    purple.color("purple")
    purple.penup()
    for location in path:
        purple.setpos(location[0], location[1])
        purple.pendown()
    return path

def write_to_file(path):
    ''' Write the given path to 'output.txt' '''
    first_vertex = None
    if len(path) > 1:
        first_vertex = path[0]
    with open('output.txt', 'w') as f:
        f.write(str(len(path)) + "\n")
        for vertex in path:
            f.write(str(vertex[0] - first_vertex[0]) + " " + str(vertex[1] - first_vertex[1]) + "\n")

def main():
    global obstacles, start, end, orange

    # Check parameters
    if len(sys.argv) < 2:
        print "Usage: lab5.py [map input file]"
        return

    # Create obstacles from input fo;e
    create_obstacles(sys.argv[1])
    
    # Create turtle window
    window = Screen()
    window.reset()
    window.setworldcoordinates(0, 0, max(dimensions) + 4, \
                               max(dimensions) + 4)
    # Draw the obstacles
    red = Turtle()
    red.speed(0)
    red.hideturtle()
    red.color("red")
    red.penup()
    for obstacle in obstacles:
        first_vertex = None
        for vertex in obstacle.vertices:
            if not first_vertex:
                first_vertex = vertex
            red.setpos(vertex[0], vertex[1])
            red.pendown()
        red.setpos(first_vertex[0], first_vertex[1])
        red.penup()
    # Draw the world boundaries
    red.setpos(0,0)
    red.pendown()
    red.setpos(dimensions[0], 0)
    red.setpos(dimensions[0], dimensions[1])
    red.setpos(0, dimensions[1])
    red.setpos(0,0)

    orange = Turtle()
    orange.speed(0)
    orange.hideturtle()
    orange.color("orange")
    orange.penup()
    
    # Choosing different vertices as reference vertex
    robot_size = sqrt(14**2 + 23**2)
    robot_vertices1 = [(0,0), (-1 * robot_size, 0), (-1 * robot_size, -1 * robot_size), (0, -1 * robot_size)]
    robot_vertices2 = [(0,0), (1 * robot_size, 0), (1 * robot_size, 1 * robot_size), (0, 1 * robot_size)]
    robot_vertices3 = [(0,0), (1 * robot_size, 0), (1 * robot_size, -1 * robot_size), (0, -1 * robot_size)]
    robot_vertices4 = [(0,0), (-1 * robot_size, 0), (-1 * robot_size, 1 * robot_size), (0, 1 * robot_size)]
    all_robot_vertices = [robot_vertices1, robot_vertices2, robot_vertices3, robot_vertices4]
    repeat = True
    repeat_index = 0
    # Try each of the reference vertices
    # in case one covers the goal/start
    while repeat:
        grow_obstacles(all_robot_vertices[repeat_index])
        
        # Draw the grown obstacles
        green = Turtle()
        green.speed(0)
        green.hideturtle()
        green.color("green")
        green.penup()
        for obstacle in obstacles:
            first_vertex = None
            for vertex in obstacle.vertices:

                if not first_vertex:
                    first_vertex = vertex
                green.setpos(vertex[0], vertex[1])
                green.pendown()
            green.setpos(first_vertex[0], first_vertex[1])
            green.penup()
        obstacles.extend([start,end])
       
        #Create the visibility graph
        visibility_graph()

        # Start or end was not covered by an obstacle, so no need to
        # try the next reference point on the robot
        if start.vertices[0] in vertices and end.vertices[0] in vertices:
            repeat = False
        # Start or end was obstructed by a grown obstacle
        # Reset everything for the next reference point
        else:
            obstacles.remove(start)
            obstacles.remove(end)
            for obstacle in obstacles:
                obstacle.vertices = copy.deepcopy(obstacle.original_vertices)
            green.clear()
            orange.clear()
        repeat_index += 1
        # Tried all reference points and didn't work
        if repeat_index > 3:
            print "No path found"
            return
    #Run Dijkstras Algorithm
    path = dijkstra()

    # Write to the path to a file
    write_to_file(path)
    
    window.exitonclick()
 
if __name__ == "__main__":
    main()

