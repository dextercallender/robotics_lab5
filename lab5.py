from turtle import *
from math import *
import time

start = ()
end = ()
dimensions = ()
obstacles = []

def create_objects(input_file):
    # for each object in input file
    obj = Obstacle( #list vertices for each object )  
    return obj

def class Obstacle:
    vertices = []
    neighbors = [[]] #valid neighbors for each vertex in the obstacle
    Obstacle( vertices ):
       vertices =  

def grow_obstacles((robot_width,robot_height)):
    # rewrite the vertices for the objects

def visibility_graph( obstacles, start, end ):
    
    #check valid neighbors for each vertex in each obstacle
    #do not check two vertex within the same shape
    #check if line intersects OTHER objects
    # for every line you draw, check if it intersects any line on any other object.
    # if not its valid
    # *make sure the intersection is inbetween the values of two vertices you're looking at
    # see code from last lab. getClosestDist()
    # plot

def dijkstras(obstacles):
    # use neighbors in each obstacle and perform dijkstras

if __name__ == "__main__":
    create_objects("input_file.txt")
    grow_obstacles(obstacles)
    visibility_graph( obstacles, start, end)
    dijkstras(obstacles)

