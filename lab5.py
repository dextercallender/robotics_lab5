from turtle import *
from math import *
import time
from sets import Set
import sys
#from gopigo import *
from matplotlib import path

start = ()
end = ()
dimensions = ()
obstacles = []
lower_rightmost_vertex = None
bounds = []

def create_obstacles(input_file):
    # create the all the obstacle in the input file and fill them in the global obstacles []
    # also set the global start and end points from the input file
    global start, end, obstacles, dimensions
    with open(input_file) as f:
        str_start = f.readline().strip().split()
        start = Obstacle()
        start.add_vertex((float(str_start[0]), float(str_start[1])))
        str_end = f.readline().strip().split()
        end = Obstacle()
        end.add_vertex((float(str_end[0]), float(str_end[1])))
        str_dim = f.readline().strip().split()
        dimensions = (float(str_dim[0]), float(str_dim[1]))
        num_obstacles = int(f.readline().strip())
        for i in range(0, num_obstacles):
            num_vertices = int(f.readline().strip())
            obstacle = Obstacle()
            for j in range(0, num_vertices):
                str_vertex = f.readline().strip().split()
                obstacle.add_vertex((float(str_vertex[0]),float(str_vertex[1])))
            obstacles.append(obstacle)
        bounds.extend([ [(0,0), (0,dimensions[0])], [(0,0), (0,dimensions[1])], ])
        bounds.extend([ [(dimensions[0],0), (dimensions[0],dimensions[1])], [(0,dimensions[1]), (dimensions[0],dimensions[1])] ])

class Obstacle:
    vertices = Set([])
    neighbors = [] # [ [vertex1, neighbor1], [vertex1, neighbor2], [vertex2, neighbor1] ]
                     # where vertex1 and vertex2 are vertices in this obstacle
                     # and the neighbor1 and neighbor 2 are vertices in other objects
                     # basically a hashmap that maps a vertex in this obstacle to a valid neighbor vertex in another obstacle
    
    def __init__(self):
       self.vertices =  []
    
    def add_vertex(self, location):
        self.vertices.append(location)
    
    def set_vertices(self, param_vertices):
        self.vertices = param_vertices

    def get_vertices(self):
        return self.vertices

    def add_neighbor(self, vertex_followedby_neighbors):
        self.neighbors.append(vertex_followedby_neighbors)

    def get_neighbors(self):
        return self.neighbors

def flip_x_and_y(location):
    return (-1*location[0], -1*location[1])

def add_vertices(vertex1, vertex2):
    return (vertex1[0] + vertex2[0], vertex1[1] + vertex2[1])


def dist(vertex):
    global lower_rightmost_vertex
    return sqrt((vertex[0] - lower_rightmost_vertex[0])**2 + (vertex[1] - lower_rightmost_vertex[1])**2)

def angle_comparison(vertex1, vertex2):
    global lower_rightmost_vertex
    vertex1_x = (vertex1[0]-lower_rightmost_vertex[0])
    vertex1_dist = dist(vertex1)
    vertex2_x = (vertex2[0]-lower_rightmost_vertex[0])
    vertex2_dist = dist(vertex2)
    vertex1_angle = vertex1_x / vertex1_dist
    vertex2_angle = vertex2_x / vertex2_dist
    vertex1 
    if vertex1_angle < vertex2_angle:
        return 1
    elif vertex1_angle > vertex2_angle:
        return -1
    else:
        if dist(vertex1) < dist(vertex2):
            return -1
        elif dist(vertex1) > dist(vertex2):
            return 1
        return 0

def isLeftTurn(vertex1, vertex2, vertex3):
    value1 = (vertex2[0] - vertex1[0]) * (vertex3[1] - vertex1[1])
    value2 = (vertex2[1] - vertex1[1]) * (vertex3[0] - vertex1[0])
    if value1-value2 > 0:
        return True
    return False
    
def create_convex_hull(vertices):
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
    global obstacles, dimensions
    # rewrite the vertices for the objects
    reflected_vertices = Set([])
    for robot_vertex in robot_vertices:
        reflected_vertices.add(flip_x_and_y(robot_vertex))
    for obstacle in obstacles:
        new_vertices = Set([])
        for vertex in obstacle.vertices:
            for reflected_vertex in reflected_vertices:
                offset_vertex = add_vertices(vertex, reflected_vertex)
                new_vertices.add(offset_vertex)             
        obstacle.set_vertices(create_convex_hull(new_vertices))

def line(vertex1, vertex2):

    if( (vertex2[0]-vertex1[0]) == 0 ):
        return [ "undefined", "dne", vertex1, vertex2 ]

    slope = (vertex2[1]-vertex1[1]) / float( (vertex2[0]-vertex1[0]) )  # (y2-y1)/(x2-x1)
    intercept = vertex2[1] - (slope * vertex2[0])               # b = y2 - m * x2    
    return [slope, intercept, vertex1, vertex2]

def intersect(obstacle, path):
    if( (obstacle[0] == "undefined" and path[0] == "undefined") ):
        return False
    return ccw(obstacle[2], path[2], path[3]) != ccw(obstacle[3],path[2],path[3]) and ccw(obstacle[2],obstacle[3],path[2]) != ccw(obstacle[2],obstacle[3],path[3])

def ccw(A,B,C):
    return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

def visibility_graph( obstacles, start, end ):
    
    obstacle_lines = []
    path_obstacles = []
    # get all lines in all objects
    for i in range(0,len(obstacles)):
        vertices = obstacles[i].get_vertices()
        for j in range(0, len(vertices)):
            obstacle_lines.append( line(vertices[j], vertices[(j+1)%len(vertices)]) )
        path_obstacles.append(path.Path(vertices))
    # generate valid neighbors in each object by testing intersections with other object lines
    for i in range(0,len(obstacles)):
        vertices = obstacles[i].get_vertices()
        for j in range(0,len(vertices)):
            vertex = vertices[j]
            for k in range(0,len(obstacles)):
                if(k != i):
                    obstacle = obstacles[k]
                    other_obs_vertices = obstacle.get_vertices()
                    for l in range(0, len(other_obs_vertices)):
                        other_vertex = other_obs_vertices[l]
                        possible_path = line( vertex, other_vertex )
                        #does this possible path intersect one of the obstacle lines
                        does_intersect = False;
                        for m in range(0, len(obstacle_lines)):
                            obstacle_line = obstacle_lines[m]
                            if( intersect( obstacle_line, possible_path) ):
                                does_intersect = True;
                                
                        for m in range(0,len(bounds)):
                            boundary_line = line(bounds[m][0], bounds[m][1])
                            if( intersect(boundary_line, possible_path) ):
                                does_intersect = True;

                        if( does_intersect == False ):
                            obstacles[i].add_neighbor( [ vertex, other_vertex ] )

            skip = False
            for l in range(0, len(path_obstacles)):
                if l != i and path_obstacles[l].contains_points([vertex]):
                    skip = True
            if not skip:
                for k in range(0,len(obstacles)):
                    if(k != i):
                        obstacle = obstacles[k]
                        other_obs_vertices = obstacle.get_vertices()
                        for l in range(0, len(other_obs_vertices)):
                            other_vertex = other_obs_vertices[l]
                            possible_path = line( vertex, other_vertex )
                            #does this possible path intersect one of the obstacle lines
                            does_intersect = False;
                            for m in range(0, len(obstacle_lines)):
                                obstacle_line = obstacle_lines[m]
                                if( intersect( obstacle_line, possible_path) ):
                                    does_intersect = True;
                            if( does_intersect == False ):
                                obstacles[i].add_neighbor( [ vertex, other_vertex ] )
                

    # Plot all valid paths (ie neighbors in every obstacle object)
    orange = Turtle()
    orange.clear()
    orange.speed(0)
    orange.hideturtle()
    orange.color("orange")
    orange.penup()
    for i in range(0,len(obstacles)):
        neighbors = obstacles[i].get_neighbors()
        for j in range(0,len(neighbors)):
            orange.setpos(neighbors[j][0][0], neighbors[j][0][1])
            orange.pendown()
            orange.setpos(neighbors[j][1][0], neighbors[j][1][1])
            orange.penup() 

    time.sleep(10)

def dijkstras(obstacles):
    # use neighbors in each obstacle and perform dijkstras
    # there are duplicate paths in each obstacle.
    # consider making a global object paths and that contains all the neighbors in all obstacles with no duplicates
    # before you run dijkstras

    pass

def turn(angle):
    # angle in radians
    #$ 5.625 degrees per encoder pulse
    revolutions = math.degrees(radians) / float(5.625)

    enable_encoders()
    # positive angle to turn left
    if(angle > 0):
        enc_tgt(0,1,revolutions)
        left()
        while(read_enc_status() == 1):
            left()
    # negative angle to turn right
    if(angle < 0):
        enc_tgt(1,0,revolutions)
        right()
        while(read_enc_status() == 1):
            left()
    stop()
    disable_encoders()

def forward(distance):
    #wheel moves 20.4 cm in one full revolution
    revolutions = float(distance) / 20.4
    enable_encoders()
    enc_tgt(1,1,revolutions)
    while(read_enc_status() == 1 ):
        fwd()
    stop()
    disable_encoders()

def move(tupe_angle_distance):
    turn(tupe_angle_distance[0])
    forward(tupe_angle_distance[1])


def get_angles_and_dist(to_visit):
    '''
        Get angles and distances between vertices to visit
        Returns [(angle1, distance1), (angle2, distance2),...]
    '''
    result = []
    for i in range(0, len(to_visit) - 1):
        vertex1 = to_visit[i]
        vertex2 = to_visit[i+1]
        dist = sqrt((vertex2[0] - vertex1[0])**2 + (vertex2[1] - vertex1[1])**2)
        if dist == 0:
            result.append((0,0))
        else:
            y_diff = vertex2[1] - vertex1[1]
            angle = acos(y_diff/dist)
            if len(result) > 0:
                angle = angle - result[len(result)-1][0]
            result.append((angle, dist))
    return result
            

def main():
    global obstacles
    global start
    global end
    global bounds

    if len(sys.argv) < 2:
        print "Usage: lab5.py [map input file]"
        return
    
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
    # Grow the obstacles
    robot_size = sqrt(14**2 + 23**2)
    robot_vertices = [(0,0), (-1 * robot_size, 0), (-1 * robot_size, -1 * robot_size), (0, -1 * robot_size)]
    grow_obstacles(robot_vertices)
    # Print the grown obstacles
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
    visibility_graph(obstacles, start, end)

    #Run Dijkstras Algorithm
    dijkstras(obstacles)
    window.exitonclick()

if __name__ == "__main__":
    main()

