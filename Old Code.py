	def RfindPath(self, node):
		if(node in self.visitedNodes):
			return None
		if(node is self.graph.nodes[self.graph.goal]):
			return node

		node = self.queue.get()[1]
		
		for x in range(0, len(node.neighbors)):
			neighborNode = self.graph.nodes[node.neighbors[x]]
			neighborNode.distanceTraveled= node.distanceTraveled+1
			self.queue.put((self.findHeuristic(neighborNode)+neighborNode.distanceTraveled , neighborNode))		
			self.visitedNodes.add(node)
			if (neighborNode in self.visitedNodes):
				continue

			temp = self.RfindPath(neighborNode)
			
			if(temp is not None):
				self.solution.append(temp)
				print "node added to solution!!!"
				return temp
