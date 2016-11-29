from gopigo import *

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
