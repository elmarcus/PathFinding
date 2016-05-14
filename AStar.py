import Queue
import math
from sets import Set

class PriorityNode():
	def __init__(self, node, h, dsf, parent):
		self.parent = parent
		self.n = node
		self.dsf = dsf
		self.priority = h + dsf

class PriorityQueue:
	def __init__(self):
		self.nodes = []

	def insert(self, new_node):
		for i in range(0, len(self.nodes)):
			if (new_node.priority < self.nodes[i].priority):
				self.nodes.insert(i, new_node)
				return
		self.nodes.append(new_node)
	
	def pop(self):
		return self.nodes.pop(0)
		
			

class AStar:
	def __init__(self, graph):
		self.graph = graph
		self.queue = PriorityQueue()
		self.solution = []
		self.visitedNodes = Set()

	def isVisited(self, n_id):
		visitedNodesArray = list(self.visitedNodes)
		for visitedNode in visitedNodesArray:
			if (visitedNode.n.n_id == n_id):
				return True
		return False
	
	def getVisitedById(self, n_id):
		for visitedNode in list(self.visitedNodes):
			if (visitedNode.n.n_id == n_id):
				return visitedNode
		return None

	def getDistance(self, node1, node2):
		xDiff = abs(node1.center.x - node2.center.x)
		yDiff = abs(node1.center.y - node2.center.y)	
		return math.sqrt(xDiff*xDiff+yDiff*yDiff)

	def findHeuristic(self, currentNode):
		return self.getDistance(currentNode, self.graph.getNodeById(self.graph.goal))

	def RfindPath(self, node, graph, queue):
		if (node.n.n_id == self.graph.goal):
			return node
		for ne in node.n.neighbors:
			next_node = graph.getNodeById(ne)
			if (not self.isVisited(next_node.n_id)):
				h = self.findHeuristic(next_node)
				d = self.getDistance(node.n, next_node)
				priority_node = PriorityNode(next_node, h, node.dsf + d, node.n.n_id)
				queue.insert(priority_node)
		next_node = queue.pop()
		self.visitedNodes.add(next_node)
		return self.RfindPath(next_node, graph, queue)
	
	def traverseSolution(self, goal):
		node = self.graph.getNodeById(self.graph.goal)
		self.solution.insert(0, node)
		next = self.getVisitedById(goal.parent)
		while (next):
			self.solution.insert(0, self.graph.getNodeById(next.n.n_id))
			next = self.getVisitedById(next.parent)

	def run(self):
		node = self.graph.nodes[self.graph.start]
		h = self.findHeuristic(node)
		priority_node = PriorityNode(node, h, 0.0, None)
		self.queue.insert(priority_node)
		self.visitedNodes.add(priority_node)
		goal = self.RfindPath(priority_node, self.graph, self.queue)
		self.traverseSolution(goal)
		return self.solution

		



	































