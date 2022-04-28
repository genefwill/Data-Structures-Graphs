# Course: 261 Data Structures
# Author: Genevieve Will
# Assignment: Assignment # 6
# Description: Program to implement an undirected graph with methods to
# add_vertex, add_edge, remove_edge, remove_vertex, get_vertices, get_edges
# is_valid_path, dfs, bfs, count_connected_components and has_cycle


import heapq
from collections import deque


class UndirectedGraph:
    """
    Class to implement undirected graph
    - duplicate edges not allowed
    - loops not allowed
    - no edge weights
    - vertex names are strings
    """

    def __init__(self, start_edges=None):
        """
        Store graph info as adjacency list
        DO NOT CHANGE THIS METHOD IN ANY WAY
        """
        self.adj_list = dict()

        # populate graph with initial vertices and edges (if provided)
        # before using, implement add_vertex() and add_edge() methods
        if start_edges is not None:
            for u, v in start_edges:
                self.add_edge(u, v)

    def __str__(self):
        """
        Return content of the graph in human-readable form
        DO NOT CHANGE THIS METHOD IN ANY WAY
        """
        out = [f'{v}: {self.adj_list[v]}' for v in self.adj_list]
        out = '\n  '.join(out)
        if len(out) < 70:
            out = out.replace('\n  ', ', ')
            return f'GRAPH: {{{out}}}'
        return f'GRAPH: {{\n  {out}}}'

    # ------------------------------------------------------------------ #

    def add_vertex(self, v: str) -> None:
        """
        Method to add new vertex to graph
        :param v: vertex to be added to graph
        """
        if v not in self.adj_list:
            self.adj_list[v] = []
        
    def add_edge(self, u: str, v: str) -> None:
        """
        Method to add a new edge to the graph. Takes two vertex as
        parameter and creates edges between them
        :param u: First vertex to connect
        :param v: Second vertex to connect
        """
        if u != v:
            if u or v not in self.adj_list:
                if u not in self.adj_list:
                    self.add_vertex(u)
                if v not in self.adj_list:
                    self.add_vertex(v)
        if u and v in self.adj_list:
            if u not in self.adj_list[v]:
                if u != v:
                    self.adj_list[v].append(u)
            if v not in self.adj_list[u]:
                if v != u:
                    self.adj_list[u].append(v)

    def remove_edge(self, v: str, u: str) -> None:
        """
        Method to remove and edge between two provided vertices.
        :param v: First vertex of edge to be removed
        :param u: Second vertex of edge to be removed
        """
        if v.islower() or u.islower():
            return
        elif u not in self.adj_list:
            return
        elif v not in self.adj_list:
            return
        elif u and v in self.adj_list:
            if u in self.adj_list[v]:
                self.adj_list[v].remove(u)
                self.adj_list[u].remove(v)
            if v in self.adj_list[u]:
                self.adj_list[v].remove(u)
                self.adj_list[u].remove(v)
        else:
            return

    def remove_vertex(self, v: str) -> None:
        """
        Method to remove a vertex from graph.
        :param v: Vertex to be removed
        """
        if v in self.adj_list:
            self.adj_list.pop(v)
            for i in self.adj_list:
                list1 = self.adj_list[i]
                if v in list1:
                    list1.remove(v)

    def get_vertices(self) -> []:
        """
        Method to get all vertices in a graph and return a list of them.
        :return: List of all vertices
        """
        vertices = []
        for i in self.adj_list:
            vertices.append(i)
        return vertices

    def get_edges(self) -> []:
        """
        Method to get all edges in a graph.
        :return: List of tuples of all edges in graph
        """
        edges = []
        for node in self.adj_list:
            for neighbour in self.adj_list[node]:
                if (node, neighbour) and (neighbour, node) not in edges:
                    edges.append((node, neighbour))
        return edges

    def is_valid_path(self, path: []) -> bool:
        """
        Method that takes a list of vertices and checks whether it is a valid path or not
        :param path: List of vertices for a potential path
        :return: True if valid path, otherwise False
        """
        path = deque(path)
        while path:
            current = path.popleft()
            if current not in self.adj_list:
                return False
            if path:
                next = path[0]
                visited = []
                for i in self.adj_list[current]:
                    visited.append(i)
                if next not in visited:
                    return False
            else:
                return True
        return True

    def dfs(self, v_start, v_end=None) -> []:
        """
        Method to perform a depth-first search (DFS) of the graph
        from given start vertices and returns a list of vertices visited in search.
        :param v_start: Vertex to start search at
        :param v_end: If given, vertex to end search at
        :return: List of all vertices visited in search
        """
        visited = []
        stack = [v_start]
        while stack:
            dfs = []
            current = stack.pop()
            if current not in self.adj_list:
                return visited
            if current not in visited:
                visited.append(current)
                if current == v_end:
                    return visited
                for i in self.adj_list[current]:
                    dfs.append(i)
                    dfs = sorted(dfs, reverse=True)
                for j in dfs:
                    stack.append(j)
        return visited

    def bfs(self, v_start, v_end=None) -> []:
        """
        Method to perform a breadth-first search (BFS) of the graph
        from given start vertices and returns a list of vertices visited in search.
        :param v_start: Vertex to start search at
        :param v_end: If given, vertex to end search at
        :return: List of all vertices visited in search
        """
        visited = []
        queue = []
        visited.append(v_start)
        queue.append(v_start)
        if v_start not in self.adj_list:
            return []
        if v_start == v_end:
            return visited
        while queue:
            bfs = []
            s = queue.pop(0)
            if s not in self.adj_list:
                return visited
            for neighbour in self.adj_list[s]:
                bfs.append(neighbour)
                bfs = sorted(bfs)
            for i in bfs:
                if i not in visited:
                    visited.append(i)
                    queue.append(i)
                if i == v_end:
                    return visited
        return visited

    def count_connected_components(self):
        """
        Method that counts the number of connected components in the graph
        :return: Number of connected components in the graph
        """
        unvisited = self.get_vertices()
        queue = []
        count = 0
        while len(unvisited) > 0:
            count += 1
            v = next(iter(unvisited))
            unvisited.remove(v)
            queue.append(v)
            while len(queue) > 0:
                v = queue.pop()
                for w in self.adj_list[v]:
                    if w in unvisited:
                        unvisited.remove(w)
                        queue.append(w)
        return count

    def has_cycle(self):
        """
        Method that checks if the graph has at least one cycle in the graph
        :return: True if graph has a cycle, otherwise False
        """
        vertex_count = len(self.get_vertices())
        visited = [False] * vertex_count

        for i in self.adj_list:
            value = list(self.adj_list)
            value = value.index(i)
            if not visited[value] and self.has_cycle_helper(self.adj_list, i, value, vertex_count, visited):
                return True
        return False

    def has_cycle_helper(self, adj, vertex, s, vertex_count, visited):
        """
        Helper method for has cycle
        :param graph: Graph to be checked
        :param vertex: vertex that is being checked
        :param s: index of vertex
        :param visited: List of whether a vertex has been visited or not
        :return: True if vertex is visited and there is an adjacent vertex
        that has already been visited, otherwise false
        """
        parent = [-1] * vertex_count
        q = deque()
        visited[s] = True
        q.append(vertex)

        while q:
            u = q.pop()
            list2 = list(adj)
            u2 = list2.index(u)
            for v in adj[u]:
                v2 = list2.index(v)
                if not visited[v2]:
                    visited[v2] = True
                    q.append(v)
                    parent[v2] = u2
                elif parent[u2] != v2:
                    return True

        return False
       

   


if __name__ == '__main__':

    print("\nPDF - method add_vertex() / add_edge example 1")
    print("----------------------------------------------")
    g = UndirectedGraph()
    print(g)

    for v in 'ABCDE':
        g.add_vertex(v)
    print(g)

    g.add_vertex('A')
    print(g)

    for u, v in ['AB', 'AC', 'BC', 'BD', 'CD', 'CE', 'DE', ('B', 'C')]:
        g.add_edge(u, v)
    print(g)


    print("\nPDF - method remove_edge() / remove_vertex example 1")
    print("----------------------------------------------------")
    g = UndirectedGraph(['AB', 'AC', 'BC', 'BD', 'CD', 'CE', 'DE'])
    g.remove_vertex('DOES NOT EXIST')
    g.remove_edge('A', 'B')
    g.remove_edge('X', 'B')
    g.remove_edge('B', 'c')
    print(g)
    g.remove_vertex('D')
    print(g)


    print("\nPDF - method get_vertices() / get_edges() example 1")
    print("---------------------------------------------------")
    g = UndirectedGraph()
    print(g.get_edges(), g.get_vertices(), sep='\n')
    g = UndirectedGraph(['AB', 'AC', 'BC', 'BD', 'CD', 'CE'])
    print(g.get_edges(), g.get_vertices(), sep='\n')


    print("\nPDF - method is_valid_path() example 1")
    print("--------------------------------------")
    g = UndirectedGraph(['AB', 'AC', 'BC', 'BD', 'CD', 'CE', 'DE'])
    test_cases = ['ABC', 'ADE', 'ECABDCBE', 'ACDECB', '', 'D', 'Z']
    for path in test_cases:
        print(list(path), g.is_valid_path(list(path)))


    print("\nPDF - method dfs() and bfs() example 1")
    print("--------------------------------------")
    edges = ['AE', 'AC', 'BE', 'CE', 'CD', 'CB', 'BD', 'ED', 'BH', 'QG', 'FG']
    g = UndirectedGraph(edges)
    test_cases = 'ABCDEGH'
    for case in test_cases:
        print(f'{case} DFS:{g.dfs(case)} BFS:{g.bfs(case)}')
    print('-----')
    for i in range(1, len(test_cases)):
        v1, v2 = test_cases[i], test_cases[-1 - i]
        print(f'{v1}-{v2} DFS:{g.dfs(v1, v2)} BFS:{g.bfs(v1, v2)}')


    print("\nPDF - method count_connected_components() example 1")
    print("---------------------------------------------------")
    edges = ['AE', 'AC', 'BE', 'CE', 'CD', 'CB', 'BD', 'ED', 'BH', 'QG', 'FG']
    g = UndirectedGraph(edges)
    test_cases = (
        'add QH', 'remove FG', 'remove GQ', 'remove HQ',
        'remove AE', 'remove CA', 'remove EB', 'remove CE', 'remove DE',
        'remove BC', 'add EA', 'add EF', 'add GQ', 'add AC', 'add DQ',
        'add EG', 'add QH', 'remove CD', 'remove BD', 'remove QG')
    for case in test_cases:
        command, edge = case.split()
        u, v = edge
        g.add_edge(u, v) if command == 'add' else g.remove_edge(u, v)
        print(g.count_connected_components(), end=' ')
    print()


    print("\nPDF - method has_cycle() example 1")
    print("----------------------------------")
    edges = ['AE', 'AC', 'BE', 'CE', 'CD', 'CB', 'BD', 'ED', 'BH', 'QG', 'FG']
    g = UndirectedGraph(edges)
    test_cases = (
        'add QH', 'remove FG', 'remove GQ', 'remove HQ',
        'remove AE', 'remove CA', 'remove EB', 'remove CE', 'remove DE',
        'remove BC', 'add EA', 'add EF', 'add GQ', 'add AC', 'add DQ',
        'add EG', 'add QH', 'remove CD', 'remove BD', 'remove QG',
        'add FG', 'remove GE')
    for case in test_cases:
        command, edge = case.split()
        u, v = edge
        g.add_edge(u, v) if command == 'add' else g.remove_edge(u, v)
        print('{:<10}'.format(case), g.has_cycle())
