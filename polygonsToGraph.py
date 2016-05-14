import math, io
import matplotlib.pyplot as plt
import matplotlib


step_x = 35.0
step_y = 30.0

class Graph:
	
	def __init__(self):
		self.nodes = []
		self.start = -1
		self.goal = -1
		
	def getNodeById(self, n_id):
		return next((x for x in self.nodes if x.n_id == n_id), None)

class Point:
	
	def __init__(self, x, y):
		self.x = x
		self.y = y
		
class Node:
	
	def __init__(self, n_id, point):
		self.n_id = n_id
		self.center = point
		self.neighbors = []
		
class Polygon:
	
	def __init__(self, points):
		self.points = points
	
def getBorders(polygons):
	if (len(polygons) == 0):
		return {'left_x': 0, 'right_x': 0, 'top_y': 0, 'bottom_y': 0}
	if (len(polygons[0].points) < 4):
		return {'left_x': 0, 'right_x': 0, 'top_y': 0, 'bottom_y': 0}
	left_x = polygons[0].points[0].x
	right_x = polygons[0].points[0].x
	bottom_y = polygons[0].points[0].y
	top_y = polygons[0].points[0].y
	for poly in polygons:
		for point in poly.points:
			if (point.x < left_x):
				left_x = point.x
			if (point.x > right_x):
				right_x = point.x
			if (point.y > bottom_y):
				bottom_y = point.y
			if (point.y < top_y):
				top_y = point.y
	return {'left_x': (left_x), 'right_x': (right_x), 'top_y': (top_y), 'bottom_y': (bottom_y)}

def createGrid(polygons, borders):	
	grid = []
	i = borders['top_y']
	while i <= borders['bottom_y']:
		row = []
		j = borders['left_x']
		while j <= borders['right_x']:
			point = Point(j + step_x/2.0, i + step_y/2.0)
			row.append(Node(-1, point))
			j += step_x
		grid.append(row)
		i += step_y
	#nullify the occupied cells
	for poly in polygons:
		left_cell = int((min(poly.points, key=lambda point: point.x).x - borders['left_x'])/ step_x)  
		right_cell = int((max(poly.points, key=lambda point: point.x).x  - borders['left_x'])/ step_x)
		bottom_cell = int((max(poly.points, key=lambda point: point.y).y  - borders['top_y'])/ step_y)
		top_cell = int((min(poly.points, key=lambda point: point.y).y  - borders['top_y'])/ step_y)
		for i in range(top_cell, bottom_cell + 1):
			for j in range(left_cell, right_cell + 1):
				grid[i][j] = None
	n_id = 0
	for i in range(0, len(grid)):
		for j in range(0, len(grid[i])):
			if (grid[i][j] != None):
				grid[i][j].n_id = n_id
				n_id += 1
	return grid					

def findNeighbors(i, j, grid):
	neighbors = []
	if (i + 1 < len(grid) and grid[i+1][j] != None):
		neighbors.append(grid[i+1][j].n_id)
	if (i + 1 < len(grid) and j + 1 < len(grid[i]) and grid[i+1][j+1] != None):
		neighbors.append(grid[i+1][j+1].n_id)
	if (i + 1 < len(grid) and j - 1 >= 0 and grid[i+1][j-1] != None):
		neighbors.append(grid[i+1][j-1].n_id)
	if (i - 1 >= 0 and grid[i-1][j] != None):
		neighbors.append(grid[i-1][j].n_id)
	if (i - 1 >= 0 and j + 1 < len(grid[i]) and grid[i-1][j+1] != None):
		neighbors.append(grid[i-1][j+1].n_id)
	if (i - 1 >= 0 and j - 1 >= 0 and grid[i-1][j-1] != None):
		neighbors.append(grid[i-1][j-1].n_id)
	if (j + 1 < len(grid[i]) and grid[i][j+1] != None):
		neighbors.append(grid[i][j+1].n_id)
	if (j - 1 >= 0 and grid[i][j-1] != None):
		neighbors.append(grid[i][j-1].n_id)
			
	return neighbors
			
def translateToGraph(polygons):
	graph = Graph()
	X = []
	Y = []

	if (len(polygons) == 0):
		print "no polygons"
		return graph
	if (len(polygons[0].points) < 4):
		print "no polygons"
		return graph
	borders = getBorders(polygons)
	grid = createGrid(polygons, borders)
	for i in range(0, len(grid)):
		for j in range(0, len(grid[i])):
			if (grid[i][j]):
				graph.nodes.append(grid[i][j])
				neighbors = findNeighbors(i, j, grid)
				grid[i][j].neighbors = neighbors
				X.append(grid[i][j].center.x)
				Y.append(grid[i][j].center.y)

				
	for i in range(0, len(grid)):
		if (grid[i][0] != None):
			print "start assigned"
			graph.start = grid[i][0].n_id
			break
	for i in range(0, len(grid)):
		if (grid[i][len(grid[i])-1] != None):
			print "goal assigned"
			graph.goal = grid[i][len(grid[i])-1].n_id
			break
				
	return graph

####for testing				
def getPolygonsFromFile(filename):
	polygons = []
	f = open(filename, 'r')	
	for line in f:
		points = []
		coordinates = line.split(' ')
		i = 0
		while i + 1 < len(coordinates):
			points.append(Point(float(coordinates[i]), float(coordinates[i+1])))
			i += 2
		polygons.append(Polygon(points))
	return polygons

####for testing		
def visualizeGraph(graph, poly):
	#draw nodes
	nx = []
	ny = []
	start = None
	goal = None
	for node in graph.nodes:
		if (node.n_id == graph.start):
			start = node.center
		elif (node.n_id == graph.goal):
			goal = node.center
		else:
			nx.append(node.center.x)
			ny.append(node.center.y)
	plt.scatter(nx, ny, c=['g']*len(graph.nodes))
	
	#draw edges
	for node in graph.nodes:
		for n in node.neighbors:
			n_center = next((x for x in graph.nodes if x.n_id == n), None).center
			plt.plot([node.center.x, n_center.x], [node.center.y, n_center.y], c='g')
	
	#draw plolygons
	px = []
	py = []
	for p in poly:
		sum_x = 0
		sum_y = 0
		for point in p.points:
			px.append(point.x)
			py.append(point.y)
			sum_x += point.x
			sum_y += point.y
		px.append(sum_x/4.0)
		py.append(sum_y/4.0)
	plt.scatter(px, py, c=['b']*len(px))
	
	#draw start & goal
	plt.scatter([start.x], [start.y], c=['r'])
	plt.scatter([goal.x], [goal.y], c=['r'])

	plt.show()
	return 

####for testing		
def visualizeResult(graph, path, poly):
	#draw nodes
	nx = []
	ny = []
	start = None
	goal = None
	for node in graph.nodes:
		if (node.n_id == graph.start):
			start = node.center
		elif (node.n_id == graph.goal):
			goal = node.center
		else:
			nx.append(node.center.x)
			ny.append(node.center.y)
	plt.scatter(nx, ny, c=['g']*len(graph.nodes))
	
	#draw edges
	for node in graph.nodes:
		for n in node.neighbors:
			n_center = next((x for x in graph.nodes if x.n_id == n), None).center
			plt.plot([node.center.x, n_center.x], [node.center.y, n_center.y], c='g')
	
	#draw plolygons
	px = []
	py = []
	for p in poly:
		sum_x = 0
		sum_y = 0
		for point in p.points:
			px.append(point.x)
			py.append(point.y)
			sum_x += point.x
			sum_y += point.y
		px.append(sum_x/4.0)
		py.append(sum_y/4.0)
	plt.scatter(px, py, c=['b']*len(px))
	
	#draw start & goal
	plt.scatter([start.x], [start.y], c=['r'])
	plt.scatter([goal.x], [goal.y], c=['r'])

	#draw path
	path_x = []
	path_y = []
	for node in path:
		path_x.append(node.center.x)
		path_y.append(node.center.y)

	plt.plot(path_x , path_y, c='r')

	plt.show()
	return 

def visualizeTree(graph, poly):
	#draw nodes
	nx = []
	ny = []
	for node in graph.nodes:
			nx.append(node.center.x)
			ny.append(node.center.y)
	plt.scatter(nx, ny, c=['g']*len(graph.nodes))
	
	#draw edges
	for node in graph.edges:
		plt.plot([node[0].center.x, node[1].center.x], [node[0].center.y, node[1].center.y], c='g')
	
	plt.show()
	return

def visualizeResultFromNodes(graph, path, poly):
	#draw nodes
	nx = []
	ny = []
	start = None
	goal = None
	for node in graph.nodes:
		if (node.n_id == graph.start):
			start = node.center
		elif (node.n_id == graph.goal):
			goal = node.center
		else:
			nx.append(node.center.x)
			ny.append(node.center.y)
	plt.scatter(nx, ny, c=['g']*len(graph.nodes))
	
	#draw edges
	for node in graph.nodes:
		for n in node.neighbors:
			n_center = next((x for x in graph.nodes if x.n_id == n), None).center
			plt.plot([node.center.x, n_center.x], [node.center.y, n_center.y], c='g')
	
	#draw plolygons
	px = []
	py = []
	for p in poly:
		sum_x = 0
		sum_y = 0
		for point in p.points:
			px.append(point.x)
			py.append(point.y)
			sum_x += point.x
			sum_y += point.y
		px.append(sum_x/4.0)
		py.append(sum_y/4.0)
	plt.scatter(px, py, c=['b']*len(px))
	
	#draw start & goal
	plt.scatter([start.x], [start.y], c=['r'])
	plt.scatter([goal.x], [goal.y], c=['r'])

	#draw path
	prev_node = None
	for node in path:
		if (prev_node):
			plt.plot([prev_node.center.x, node.center.x], [prev_node.center.y, node.center.y], c='r')
		prev_node = node
	plt.show()
	return

####for testing				
#poly = 	getPolygonsFromFile('polygons2016-05-06134555706541.txt')
#graph = translateToGraph(poly)
#visualizeGraph(graph, poly)
