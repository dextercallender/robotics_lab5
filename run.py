#from gopigo import *
from math import *
import math

path = []

def turn(angle):
    # angle in radians
    #$ 5.625 degrees per encoder pulse
    revolutions = math.degrees(angle) / float(5.625)

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
        dist = sqrt((vertex2.x - vertex1.x)**2 + (vertex2.y - vertex1.y)**2)
        if dist == 0:
            result.append((0,0))
        else:
            y_diff = vertex2.y - vertex1.y
            angle = acos(y_diff/dist)
            if len(result) > 0:
                angle = angle - result[len(result)-1][0]
            result.append((angle, dist))
    return result

class Point:
    x = 0.0
    y = 0.0
    
    def __init__(self, _x, _y):
       self.vertices =  []
       self.x = _x
       self.y = _y
    
def make_path(input_file):
    global path
    with open(input_file) as f:
        number_of_points = int( f.readline().strip().split()[0] )
        for i in range(0, number_of_points ):
            point = f.readline().strip().split(",")
            path.append( Point( float(point[0]), float(point[1])) )    
        
def main():
    global path    
    make_path("optimal_path.txt")

    result = get_angles_and_dist(path)
    for i in range(0, len(result)):
        move(result[i])
    
if __name__ == "__main__":
    main()
