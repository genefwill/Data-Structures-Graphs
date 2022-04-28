# Course: CS261 - Data Structures
# Author: Genevieve Will
# Assignment: Assignment 6
# Description: Program to implement a directed graph with methods to
# add_vertex, add_edge, remove_edge, get_vertices, get_edges,
# is_valid_path, dfs, bfs, and has_cycle

class DirectedGraph:
    """
    Class to implement directed weighted graph
    - duplicate edges not allowed
    - loops not allowed
    - only positive edge weights
    - vertex names are integers
    """

    def __init__(self, start_edges=None):
        """
        Store graph info as adjacency matrix
        DO NOT CHANGE THIS METHOD IN ANY WAY
        """
        self.v_count = 0
        self.adj_matrix = []

        # populate graph with initial vertices and edges (if provided)
        # before using, implement add_vertex() and add_edge() methods
        if start_edges is not None:
            v_count = 0
            for u, v, _ in start_edges:
                v_count = max(v_count, u, v)
            for _ in range(v_count + 1):
                self.add_vertex()
            for u, v, weight in start_edges:
                self.add_edge(u, v, weight)

    def __str__(self):
        """
        Return content of the graph in human-readable form
        DO NOT CHANGE THIS METHOD IN ANY WAY
        """
        if self.v_count == 0:
            return 'EMPTY GRAPH\n'
        out = '   |'
        out += ' '.join(['{:2}'.format(i) for i in range(self.v_count)]) + '\n'
        out += '-' * (self.v_count * 3 + 3) + '\n'
        for i in range(self.v_count):
            row = self.adj_matrix[i]
            out += '{:2} |'.format(i)
            out += ' '.join(['{:2}'.format(w) for w in row]) + '\n'
        out = f"GRAPH ({self.v_count} vertices):\n{out}"
        return out

    # ------------------------------------------------------------------ #

    def add_vertex(self) -> int:
        """
        Method to add vertex to directed graph
        """
        self.v_count = self.v_count + 1
        self.adj_matrix = [[0 for x in range(self.v_count)] for y in range(self.v_count)]
        return self.v_count

    def add_edge(self, src: int, dst: int, weight=1) -> None:
        """
        Method to add edge to the graph, connecting to the two vertices
        will add a weight if given
        :param src: First vertex to connect
        :param dst: Second vertex to connect
        :param weight: Weight for edge if given, otherwise it is 1
        """
        if src == dst:
            pass
        elif src >= 0 and dst >= 0:
            if src <= self.v_count -1 and dst <= self.v_count-1:
                self.adj_matrix[src][dst] = weight

    def remove_edge(self, src: int, dst: int) -> None:
        """
        Method to remove the edge between given vertices
        :param src: First vertex for removed edge
        :param dst: Second vertex for removed edge
        """
        if src >= 0 and dst >= 0:
            if src <= self.v_count - 1 and dst <= self.v_count - 1:
                self.adj_matrix[src][dst] = 0

    def get_vertices(self) -> []:
        """
        Method that returns a list of vertices of the graph
        :return: list of all vertices
        """
        vertices = []
        for i in range(self.v_count):
            vertices.append(i)
        return vertices

    def get_edges(self) -> []:
        """
        Method to find all edges in a graph and return a list of them all
        :return: List tuples of all edges in the graph
        """
        edges = []
        for vertex in self.get_vertices():
            edge_list = self.adj_matrix[vertex]
            for i in range(len(edge_list)):
                if edge_list[i] != 0:
                    tuple = (vertex,)
                    tuple += (i, edge_list[i])
                    edges.append(tuple)
        return edges

    def is_valid_path(self, path: []) -> bool:
        """
        Method that takes a list of vertices and checks if there is a valid
        path between those vertices
        :param path: List of vertices to check
        :return: True if path is valid, otherwise False
        """
        visited = []
        all_vertices = self.get_vertices()
        for current_vertex, next_vertex in zip(path, path[1:]):
            curr_edge_list = self.adj_matrix[current_vertex]
            if current_vertex in all_vertices and curr_edge_list[next_vertex] != 0:
                if self.adj_matrix[current_vertex] != 0:
                    visited.append(True)
            else:
                return False
        return False not in visited

    def dfs(self, v_start, v_end=None) -> []:
        """
        Method that performs a depth-first search (DFS) in the graph
        and returns a list of vertices visited in the search
        :param v_start: Vertex for search to start at
        :param v_end: Vertex for search to end at, if given
        :return: List of vertices visited in search
        """
        if v_start < 0 or v_start >= self.v_count:
            return []
        visited = [False for i in range(self.v_count)]
        stack = []
        dfs_list = []
        stack.append(v_start)
        while stack:
            s = stack.pop()
            if not visited[s]:
                dfs_list.append(s)
                visited[s] = True
            if s == v_end:
                return dfs_list
            for node in reversed(range(self.v_count)):
                if not visited[node] and self.adj_matrix[s][node] > 0:
                    stack.append(node)
        return dfs_list

    def bfs(self, v_start, v_end=None) -> []:
        """
        Method that performs a breadth-first search (BFS) in the graph
        and returns a list of vertices visited in the search
        :param v_start: Vertex for search to start at
        :param v_end: Vertex for search to end at, if given
        :return: List of vertices visited in search
        """
        if v_start < 0 or v_start >= self.v_count:
            return []
        visited = [False] * self.v_count
        bfs_list = []
        queue = []
        visited[v_start] = True
        queue.append(v_start)
        while queue:
            u = queue.pop(0)
            bfs_list.append(u)
            if u == v_end:
                return bfs_list
            for i in range(self.v_count):
                if visited[i] is False and self.adj_matrix[u][i] > 0:
                    queue.append(i)
                    visited[i] = True
        return bfs_list

    def has_cycle(self):
        """
        Method to check if graph has at least one cycle
        :return: True if graph has cycle, otherwise False
        """
        visited = [False] * self.v_count
        rec_stack = [False] * self.v_count
        for node in range(self.v_count):
            if visited[node] is False:
                if self.has_cycle_rec(node, visited, rec_stack):
                    return True
        return False

    def has_cycle_rec(self, u, visited, rec_stack):
        """
        Helper function for has_cycle
        :param u: node to check
        :param visited: List of visited vertex
        :param rec_stack: recursive stack
        :return: True if there is a cycle, otherwise False
        """
        visited[u] = True
        rec_stack[u] = True
        for v in range(self.v_count):
            if self.adj_matrix[u][v] == 0:
                continue
            if not visited[v]:
                if self.has_cycle_rec(v, visited, rec_stack):
                    return True
            elif rec_stack[v]:
                return True
        rec_stack[u] = False
        return False

    def dijkstra(self, src: int) -> []:
        """
        Method that implements the Dijkstra's algorithm to compute the
        shortest path in the graph
        :param src: Starting vertex for algorithm
        :return: List of value for each vertex in graph with value at
        index zero being the shortest path
        """
        dist = [99999999] * self.v_count
        dist[src] = 0
        spt_set = [False] * self.v_count

        for i in range(self.v_count):
            u = self.get_min(dist, spt_set)
            spt_set[u] = True
            for v in range(self.v_count):
                if self.adj_matrix[u][v] > 0 and spt_set[v] is False and dist[v] > (dist[u] + self.adj_matrix[u][v]):
                    dist[v] = dist[u] + self.adj_matrix[u][v]

        for i in range(len(dist)):
            if dist[i] == 99999999:
                dist[i] = float('inf')
        return dist

    def get_min(self, dist, spt_set):
        min = 99999999
        min_index = 0
        for v in range(self.v_count):
            if dist[v] < min and spt_set[v] is False:
                min = dist[v]
                min_index = v

        return min_index


if __name__ == '__main__':

    print("\nPDF - method add_vertex() / add_edge example 1")
    print("----------------------------------------------")
    g = DirectedGraph()
    print(g)
    for _ in range(5):
        g.add_vertex()
    print(g)

    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    for src, dst, weight in edges:
        g.add_edge(src, dst, weight)
    g.add_edge(1,5,2)
    print(g)


    print("\nPDF - method get_edges() example 1")
    print("----------------------------------")
    g = DirectedGraph()
    print(g.get_edges(), g.get_vertices(), sep='\n')
    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    g = DirectedGraph(edges)
    print(g.get_edges(), g.get_vertices(), sep='\n')


    print("\nPDF - method is_valid_path() example 1")
    print("--------------------------------------")
    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    g = DirectedGraph(edges)
    test_cases = [[0, 1, 4, 3], [1, 3, 2, 1], [0, 4], [4, 0], [], [2]]
    for path in test_cases:
        print(path, g.is_valid_path(path))


    print("\nPDF - method dfs() and bfs() example 1")
    print("--------------------------------------")
    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    g = DirectedGraph(edges)
    for start in range(5):
        print(f'{start} DFS:{g.dfs(start)} BFS:{g.bfs(start)}')


    print("\nPDF - method has_cycle() example 1")
    print("----------------------------------")
    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    g = DirectedGraph(edges)

    edges_to_remove = [(3, 1), (4, 0), (3, 2)]
    for src, dst in edges_to_remove:
        g.remove_edge(src, dst)
        print(g.get_edges(), g.has_cycle(), sep='\n')

    edges_to_add = [(4, 3), (2, 3), (1, 3), (4, 0)]
    for src, dst in edges_to_add:
        g.add_edge(src, dst)
        print(g.get_edges(), g.has_cycle(), sep='\n')
    print('\n', g)


    print("\nPDF - dijkstra() example 1")
    print("--------------------------")
    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    g = DirectedGraph(edges)
    for i in range(5):
        print(f'DIJKSTRA {i} {g.dijkstra(i)}')
    g.remove_edge(4, 3)
    print('\n', g)
    for i in range(5):
        print(f'DIJKSTRA {i} {g.dijkstra(i)}')
