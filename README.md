Lab 4 Readme
Group 10
Team members: Dexter Callender III (dec2148), Daniel Hong (sh3266), Yanrong Wo (yw2513)

Part 1 - Growing obstacles, visibility graph, dijkstras
	File:
		lab5_part1.py
	Usage:
		python lab5_part1.py [input map file]
		- A turtle window will open up and plot the original obstacles, world bounds, start, and end in red. Then it will plot the grown obstacles in green. Then it will plot the visibility graph in orange. Finally, it will plot the shortest path in purple.
		- The program then writes this path to "output.txt"
		- When the program is done, click on the turtle window to end the program. 
	Basic Algorithm:
		1) Parse the input file and draw the parsed obstacles, dimensions, start, and end
		2) Grow the obstacles by the largest possible size of the robot: sqrt(14^2 + 23^2) by sqrt(14^2 + 23^2). With all the new points, we calculate the convex hull with graham's algorithm. Draw the convex hull as the grown obstacles.
		3) Create the visibility graph by doing the following: create a list of all the line segments of the obstacles. Create vertex objects for each vertex that is not inside another obstacle or outside the world bounds. For each vertex loop through all other vertices and see if that is a valid visible line segment. It is valid if it does not intersect any of the obstacles line segments and if it is not a vertex of the same object that is adjacent to the current vertex. Add each of the visible line segments into the graph
		4) Check if the start and goal are not in the graph, repeat from step 2 with a different reference point. 
		5) Run Dijkstra's to find the shortest path in the visibility graph.
		6) Write the vertices that make up the shortest path to a file 'output.txt'
	Additional Information:
		Shape and size of robot: sqrt(14^2 + 23^2) by sqrt(14^2 + 23^2)
		Reference Point: We start by trying the bottom right corner as the reference point. If the goal/start is obstructed, we then try top right, then top left, then bottom right.
Part 2 - Roborace
	File: 
		lab5_part2.py
		find_cone.py 
	Usage:
		python lab4_part2.py
	Basic Algorithm:
		1) Parse the path from "output.txt"
		2) Calculate the distance/angle change needed between each two adjcent vertices in the path.
		3) Have the robot turn and then move as specified by the angles/distance in step 2. 
				- While moving forward, check the ultrasound sensor distance and see if it is less than 5cm. If so stop the current movement forward and proceed with the next set of angle/distances. (We take 3 ultrasound readings to try to make up for inaccurate readings.)
		4) The robot should now be at the goal, so attempt to find the cone. Most likely the cone will be placed in front of the robot, so turn 90 + 11.25 degrees to the left. Then scan 360 degrees by increments of 11.25 to the right for the cone. If the cone is found (area of the orange is greater than 100 and the x-center of the image is in the object), then start moving forward towards it. Adjust to the left and right as needed (i.e. if the x-center of the image is to the left of the left bound of the object, turn right.) Stop moving forward when area of the orange section is greater than 25000 pixels or less than 100 pixels (lost the cone). If the cone is not found, just print that it wasn't found. 
	Additional Information:
		In addition to the finding cone code, find_cone also has a select_color() function that allows an image to be taken, a rectangle of color to be chosen, and will output the mean color and standard deviation. This will be used to get the color of the cone prior to the roborace. 

