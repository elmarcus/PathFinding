

class ImagetoGraph:
	
	def __init__(self):
		self.start = Node()
		self.goal  = Node()

class Node:
	def __init__(self):
		self.neighbors = []
		self.x
		self.y
		self.visited=False
		self.distanceTraveled=0
	def getPriority(self, heuristic):
		return self.distanceTraveled+heuristic

