from turtle import *
from math import *
import time
from sets import Set

start = ()
end = ()
dimensions = ()
obstacles = []
lower_rightmost_vertex = None

def create_obstacles(input_file):
    # create the all the obstacle in the input file and fill them in the global obstacles []
    # also set the global start and end points from the input file
    pass 

class Obstacle:
    vertices = Set([])
    neighbors = [] # [ [vertex1, neighbor1], [vertex1, neighbor2], [vertex2, neighbor1] ]
                     # where vertex1 and vertex2 are vertices in this object
                     # and the neighbor1 and neighbor 2 are vertices in other objects
                     # basically a hashmap that maps a vertex in an object to a valid neighbor
    
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
        stack.append(vertices.pop())
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
            stack.append(vertex)
    return stack

def grow_obstacles(robot_vertices):
    global obstacles
    # rewrite the vertices for the objects
    reflected_vertices = Set([])
    for robot_vertex in robot_vertices:
        reflected_vertices.add(flip_x_and_y(robot_vertex))
    for obstacle in obstacles:
        new_vertices = Set([])
        for vertex in obstacle.vertices:
            turtle = Turtle()
            turtle.penup()
            turtle.hideturtle()
            turtle.color("black")
            first_vertex = None
            for reflected_vertex in reflected_vertices:
                offset_vertex = add_vertices(vertex, reflected_vertex)
                if not first_vertex:
                    first_vertex = offset_vertex
                turtle.setpos(offset_vertex[0], offset_vertex[1])
                turtle.pendown()
                new_vertices.add(offset_vertex)
            turtle.setpos(first_vertex[0], first_vertex[1])
            turtle.penup()                
        obstacle.set_vertices(create_convex_hull(new_vertices))

def line(vertex1, vertex2):

    if( (vertex2[0]-vertex1[0]) == 0 ):
        return [ "undefined", "dne", vertex1, vertex2 ]

    slope = (vertex2[1]-vertex1[1]) / float( (vertex2[0]-vertex1[0]) )  # (y2-y1)/(x2-x1)
    intercept = vertex2[1] - (slope * vertex2[0])               # b = y2 - m * x2    
    return [slope, intercept, vertex1, vertex2]

'''
attempted intersetion funciton
def intersect(obstacle, path):
    # format : [ slope, y int, point, point ]

    # undefined slope for one of the lines
    if( obstacle[0] == "undefined" and path[0] != "undefined" ):
        x_intersect = obstacle[2][0]
        y_intersect = (path[0] * x_intersect) + path[1]

        y_path_end1 = (path[0] * path[2][0]) + path[1]
        y_path_end2 = (path[0] * path[3][0]) + path[1]

        y_obstacle_end1 = obstacle[2][1]
        y_obstacle_end2 = obstacle[3][1]

    elif( path[0] == "undefined" and obstacle[0] != "undefined" ):
        x_intersect = path[2][0]
        y_intersect = (obstacle[0] * x_intersect) + obstacle[1]

        y_path_end1 = path[2][1]
        y_path_end2 = path[3][1]

        y_obstacle_end1 = (obstacle[0] * obstacle[2][0]) + obstacle[1]
        y_obstacle_end2 = (obstacle[0] * obstacle[3][0]) + obstacle[1]

    # if lines parallel 
    elif( (obstacle[0] == "undefined" and path[0] == "undefined") or (obstacle[0] - path[0] == 0) ):
        return False

    # normal case
    else:
        x_intersect = (path[1]-obstacle[1]) / float( (obstacle[0]-path[0]) )
        y_intersect = (obstacle[0] * x_intersect) + obstacle[1]

        y_path_end1 = (path[0] * path[2][0]) + path[1]
        y_path_end2 = (path[0] * path[3][0]) + path[1]

        y_obstacle_end1 = (obstacle[0] * obstacle[2][0]) + obstacle[1]
        y_obstacle_end2 = (obstacle[0] * obstacle[3][0]) + obstacle[1]

    # if the obstacle line segment actually crosses the path segment. NOT the intersection being on the path

    if( (y_intersect < y_path_end1 and y_intersect > y_path_end2) or (y_intersect > y_path_end1 and y_intersect < y_path_end2) ):
        if( (y_intersect < y_obstacle_end1 and y_intersect > y_obstacle_end2) or (y_intersect > y_obstacle_end1 and y_intersect < y_obstacle_end2) ):
            print True
            return True

    return False
'''

def intersect(obstacle, path):
    return ccw(obstacle[2], path[2], path[3]) != ccw(obstacle[3],path[2],path[3]) and ccw(obstacle[2],obstacle[3],path[2]) != ccw(obstacle[2],obstacle[3],path[3])

def ccw(A,B,C):
    return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

def visibility_graph( obstacles, start, end ):
    
    obstacle_lines = []

    # get all lines in all objects
    for i in range(0,len(obstacles)):
        vertices = obstacles[i].get_vertices()
        for j in range(0, len(vertices)):
            obstacle_lines.append( line(vertices[j], vertices[(j+1)%len(vertices)]) )

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

def main():
    global obstacles
    global start
    global end

    create_obstacles("input_file.txt")

    #Temp code for setting start and end
    start =  ( 0, 0 )
    end = ( 300, 300 )

    # Temp code for creating obstacles
    obstacle = Obstacle()
    obstacle.add_vertex((200,220))
    obstacle.add_vertex((221.213203436,241.213203436))
    obstacle.add_vertex((210.606601718,251.819805153))
    obstacle.add_vertex((189.393398282,230.606601718))
    obstacle2 = Obstacle()
    obstacle2.add_vertex((130,180))
    obstacle2.add_vertex((159.997181494,179.588779362))
    obstacle2.add_vertex((145.35287801,194.640170509))
    obstacle3 = Obstacle()
    obstacle3.add_vertex((150,120))
    obstacle3.add_vertex((194.995772241,119.383169043))
    obstacle3.add_vertex((173.029317014,141.960255763))
    obstacle4 = Obstacle()
    obstacle4.add_vertex((230,170))
    obstacle4.add_vertex((251.501987353,190.920433549))
    obstacle4.add_vertex((230.503960307,191.208287996))
    obstacles.extend([obstacle, obstacle2, obstacle3, obstacle4])
    
    # Create turtle window
    window = Screen()
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
   
    #Create the visibility graph
    visibility_graph(obstacles, start, end)

    #Run Dijkstras Algorithm
    dijkstras(obstacles)
    #window.exitonclick()

if __name__ == "__main__":
    main()

