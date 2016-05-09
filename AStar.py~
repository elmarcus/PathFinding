import ImageToGraph.py
import TraversePath.py

class AStar:
	def __init__(self, graph):
		self.graph = graph
		self.queue = Queue.PriorityQueue(0)
		self.solution = []

	def findHeuristic(self, currentNode):
		xDiff = abs(currentNode.x - self.graph.goal.x)
		yDiff = abs(currentNode.y - self.graph.goal.y)
		return xDiff+yDiff


	def RfindPath(self, node):
		if(node.visited==True):
			return None
		if(node == self.graph.goal)
		{return node}

		node = self.queue.get()
		for x in node.neighbors:
			x.distanceTraveled= node.distanceTraveled+1
			queue.put(x.getPriority(self.findHeuristic(x)) , x)		
			node.visited=True	
			temp = RfindPath(x)
			
			if(temp is not None):
				self.solution.append(temp)
				return temp

	def run(self):
		Node node = self.graph.start
		self.queue.put(0, node)
		RfindPath(node)
		self.solution.reverse
		path = TraversePath(self.solution)

		



	































