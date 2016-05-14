from sets import Set
import random
import math
class tree:
	def __init__(self):
		self.edges = []
		self.nodes = []

	def add_edge(self, f, s):
		self.edges.append((f,s))

	def add_vertex(self, n):
		self.nodes.append(n)

class RRT:
	
	def __init__(self, graph):
		self.graph = graph

	def find_path(self, G, n, path_list):
		for i in range(0, len(G.edges)):
			if(G.edges[i][1] is n):
				path_list.append(G.edges[i][0])
				del G.edges[i]
				self.find_path(G, path_list[len(path_list) -1], path_list)
				break
		
		return path_list

	def rand_conf(self, visited_set, k):
		while(True):
			i = random.randint(1,k - 1)
			if(not i in visited_set):
				visited_set.add(i)
				return self.graph.nodes[i]

	def nearest_vertex(self, q_rand, G):
		min_val = float("inf")
		min_node = None
		for n in G.nodes:
			if(math.sqrt(math.pow(q_rand.center.x - n.center.x,2) + math.pow(q_rand.center.y - n.center.y, 2)) < min_val):
				min_val = math.sqrt(math.pow(q_rand.center.x - n.center.x,2) + math.pow(q_rand.center.y - n.center.y, 2))
				min_node = n

		return min_node

	def new_conf(self, q_near, q_rand):
		min_val = float("inf")
		min_node = None

		for n in q_near.neighbors:

			if(math.sqrt(math.pow(q_rand.center.x - self.graph.nodes[n].center.x,2) + math.pow(q_rand.center.y - self.graph.nodes[n].center.y,2)) < min_val):

				min_val = math.sqrt(math.pow(q_rand.center.x - self.graph.nodes[n].center.x,2) + math.pow(q_rand.center.y - self.graph.nodes[n].center.y,2))
				min_node = self.graph.nodes[n]


		return min_node

	def run(self):
		init_node = self.graph.nodes[self.graph.start]
		visited_set = Set()
		path = []
		G = tree()
		G.add_vertex(init_node)
		visited_set.add(init_node)

		print "GRAPH SIZE: " + str(len(self.graph.nodes))
		for n in range(1, len(self.graph.nodes)):
			q_rand = self.rand_conf(visited_set, len(self.graph.nodes))
			#print "iteration " + str(n)
			
			c = 0
			while(True):
				q_near = self.nearest_vertex(q_rand, G)
				q_new = self.new_conf(q_near, q_rand)


				c += 1
				if (q_new in visited_set):
					break

				visited_set.add(q_new)
				G.add_vertex(q_new)
				G.add_edge(q_near, q_new)

				if(q_new.center.x == q_rand.center.x and q_new.center.y == q_rand.center.y):
					break			
			
			if(not q_new):
				print "none-type error"
				return None
			if(q_new.center.x == self.graph.nodes[self.graph.goal].center.x and q_new.center.y == self.graph.nodes[self.graph.goal].center.y):
				path.append(self.graph.nodes[self.graph.goal])
				path = self.find_path(G, self.graph.nodes[self.graph.goal], path)
				break
		path.reverse
		return path

