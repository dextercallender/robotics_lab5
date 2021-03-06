from gopigo import *
from math import *
import math
from time import *
from find_cone import *

path = []

def turn(angle):
        '''
                Turn the given angle, where left is positive and right is negative
        '''
	# angle in radians
	# 11.25 degrees per encoder pulse
	
	revolutions = abs(math.degrees(angle) / float(11.25))

	# positive angle to turn left
	enable_encoders()
	angle -= 0.2
	if(angle > 0):
		#enable_encoders()
		enc_tgt(0,1,int(round(revolutions*1.2)))
		left_rot()
		while(read_enc_status() == 1):
			left_rot()
	# negative angle to turn right
	elif(angle < 0):
		#enable_encoders()
		enc_tgt(1,0,int(round(revolutions*1.2)))
		right_rot()
		while(read_enc_status() == 1):
			right_rot()
	stop()
	disable_encoders()

def forward(distance):
        '''
                Move forward the given distance
        '''
	#wheel moves 20.4 cm in one full revolution
	revolutions = abs((float(distance) / 20.4) * 18)
	enable_encoders()
	enc_tgt(1,1,int(round(revolutions*1.2)))
	reading1 = us_dist(15)
	reading2 = us_dist(15)
	reading3 = us_dist(15)
	# Try to keep going if we're not going to hit anything
	while(read_enc_status() == 1 and reading1  > 5 and reading2 > 5 and reading3 > 5):
		fwd()
		reading1 = us_dist(15)
		reading2 = us_dist(15)
		reading3 = us_dist(15)
		if(not(reading1 > 5 and reading2 > 5 and reading3 > 5)):
			print "Obstacle ahead! Ending current forward movement."
	stop()
	disable_encoders()

def move(tupe_angle_distance):
        '''
                Do turn and then movement forward
        '''
	sleep(1)
	turn(tupe_angle_distance[0])
	sleep(1)
	forward(tupe_angle_distance[1])

def get_angles_and_dist(to_visit):
	'''
		Get angles and distances between vertices to visit
		Returns [(angle1, distance1), (angle2, distance2),...]
	'''
	result = []
	prev_angle = 0
	abs_angle = 0
	angle = 0
	for i in range(len(to_visit) - 1):
		vertex1 = to_visit[i]
		vertex2 = to_visit[i+1]
		dist = sqrt((vertex2.x - vertex1.x)**2 + (vertex2.y - vertex1.y)**2)
		if dist == 0:
			result.append((0,0))
		else:
			x_diff = vertex2.x - vertex1.x
			y_diff = vertex2.y - vertex1.y	
			angle = atan( y_diff/x_diff ) - abs_angle
			#prev_angle = angle
			abs_angle += angle
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
        '''
                Parse input file
        '''
	with open(input_file) as f:
		number_of_points = int( f.readline().strip().split()[0] )
		for i in range( number_of_points ):
			point = f.readline().strip().split(" ")
			path.append( Point( float(point[0]), float(point[1])) )    

def main():
	global path
	make_path("output.txt")
	result = get_angles_and_dist(path)

	for i in range(0, len(result)):
		move(result[i])
	find_cone()

if __name__ == "__main__":
	main()
