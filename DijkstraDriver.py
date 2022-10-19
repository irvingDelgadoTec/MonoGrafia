import sys
'''
Reading/Importing/Parsing input graph from Adj Matrix file "test.txt
Given a graph and a source vertex in graph, find shortest paths from source to all vertices in the given graph.
'''

with open('/Users/idelgados/Documents/test') as f:
    lines = f.read().splitlines()



emptyGraph = []
for l in lines:
    emptyGraph.append( list(map(int, l.split(','))) )


#Applying Dijkstra algorithm 

    


class Graph():

	def __init__(self, vertices):
		self.V = vertices
		self.graph = [[0 for column in range(vertices)]
					for row in range(vertices)]

	def printSolution(self, dist):

		#Following is the printing of all cheapest cost to every edge. 
		print("Vertex \tDistance from Source")
		for node in range(self.V):
			print(node, "\t", dist[node])


	def getShortestPath(self,distances,previous,targetIndex):
		path = []
		if(distances[targetIndex]==sys.maxsize):
			return path

		at = targetIndex
		while at is not None :
			path.append(at)
			at = previous[at]
		path.reverse()

		return path


	# A utility function to find the vertex with
	# minimum distance value, from the set of vertices
	# not yet included in shortest path tree
	def minDistance(self, dist, sptSet):

		# Initialize minimum distance for next node
		min = sys.maxsize

		# Search not nearest vertex not in the
		# shortest path tree
		for u in range(self.V):
			if dist[u] < min and sptSet[u] == False:
				min = dist[u]
				min_index = u

		return min_index

	# Function that implements Dijkstra's single source
	# shortest path algorithm for a graph represented
	# using adjacency matrix representation
	def dijkstra(self, src, targetNode):

		dist = [sys.maxsize] * self.V
		prev = [None] * self.V

		#print(prev)

		dist[src] = 0
		sptSet = [False] * self.V

		for cout in range(self.V):

			# Pick the minimum distance vertex from
			# the set of vertices not yet processed.
			# x is always equal to src in first iteration
			x = self.minDistance(dist, sptSet)

			# Put the minimum distance vertex in the
			# shortest path tree
			sptSet[x] = True

			# Update dist value of the adjacent vertices
			# of the picked vertex only if the current
			# distance is greater than new distance and
			# the vertex in not in the shortest path tree
			for y in range(self.V):
				newDist = dist[x] + self.graph[x][y]
				if self.graph[x][y] > 0 and sptSet[y] == False and dist[y] > newDist:
					prev[y] = x
					dist[y] = newDist

		self.printSolution(dist)
		print(self.getShortestPath(dist,prev,targetNode))


# Driver's code
if __name__ == "__main__":
	g = Graph(9)
	g.graph = [[0, 4, 0, 0, 0, 0, 0, 8, 0],
		[4, 0, 8, 0, 0, 0, 0, 11, 0],
		[0, 8, 0, 7, 0, 4, 0, 0, 2],
		[0, 0, 7, 0, 9, 14, 0, 0, 0],
		[0, 0, 0, 9, 0, 10, 0, 0, 0],
		[0, 0, 4, 14, 10, 0, 2, 0, 0],
		[0, 0, 0, 0, 0, 2, 0, 1, 6],
		[8, 11, 0, 0, 0, 0, 1, 0, 7],
		[0, 0, 2, 0, 0, 0, 6, 7, 0]
		]

	g.dijkstra(0,0)
