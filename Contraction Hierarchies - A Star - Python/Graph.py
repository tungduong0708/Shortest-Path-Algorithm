import networkx as nx
from pyproj import Transformer
import math
import heapq
import json
import sys
import collections
import time

class Graph:
    def __init__(self):
        self.G = nx.MultiDiGraph()
        self.times_all = {}
        self.shortest_path_time = {}
        self.shortest_adj = {}
        self.shortest_cnt = {}
        self.stop_info = {}
        self.node_order = {}
        self.order_of = {}
    
    def make_graph_demo(self):
        nodes = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K']
        self.G.add_nodes_from(nodes)

        # Define edges with weights
        edges = [
            ('A', 'C', 5), ('A', 'B', 3),
            ('B', 'C', 3), ('B', 'D', 5),
            ('C', 'D', 2), ('C', 'J', 2),
            ('D', 'E', 7), ('D', 'J', 4), 
            ('E', 'F', 6), ('E', 'J', 3),
            ('F', 'H', 2), 
            ('G', 'H', 3), ('G', 'F', 4),
            ('H', 'I', 3), ('H', 'J', 2), 
            ('I', 'J', 4), ('I', 'G', 5), 
            ('J', 'K', 3),
            ('K', 'A', 3), ('K', 'I', 6)
        ]

        # Add edges with weights and shortcut_node attribute
        for u, v, w in edges:
            self.G.add_edge(u, v, weight=w, shortcut_node=0)
            self.G.add_edge(v, u, weight=w, shortcut_node=0)
            if u not in self.times_all:
                self.times_all[u] = {}
            if v not in self.times_all:
                self.times_all[v] = {}
            self.times_all[u][v] = w
            self.times_all[v][u] = w

        # Define coordinates for each node (latitude, longitude)
        coordinates = {
            'A': (10.0, 10.0),
            'B': (20.0, 10.0),
            'C': (15.0, 12.0),
            'D': (18.0, 14.0),
            'E': (23.0, 15.0),
            'F': (26.0, 13.0),
            'G': (30.0, 11.0),
            'H': (28.0, 9.0),
            'I': (25.0, 7.0),
            'J': (17.0, 8.0),
            'K': (12.0, 6.0),
        }

        # Store the stop_info for each node
        self.stop_info = coordinates

    def get_distance(self, x1, y1, x2, y2):
        return math.sqrt(((x1 - x2) ** 2) + ((y1 - y2) ** 2))
    
    def get_path(self, current_vertex, parents):
        path = []
        if parents.get(current_vertex, 0) == 0:
            return path
        while parents[current_vertex] != current_vertex:
            path.append(current_vertex)
            current_vertex = parents[current_vertex]
        path.append(current_vertex)
        return list(reversed(path))

    def a_star(self, source_node, target_node): # Load stop info before use
        close_list = set()
        open_list = []
        g = {}
        f = {}
        h = {}
        parents = {}

        def heuristic(cur_id, target_id):
            h[cur_id] = self.get_distance(self.stop_info[cur_id][0], self.stop_info[cur_id][1], 
                                    self.stop_info[target_id][0], self.stop_info[target_id][1])

        g[source_node] = 0
        heuristic(source_node, target_node)
        f[source_node] = g[source_node] + h[source_node]
        parents[source_node] = source_node
        heapq.heappush(open_list, (f[source_node], source_node))
        while open_list:
            _, n = heapq.heappop(open_list)
            if n in close_list:
                continue
            if n == target_node:
                path = self.get_path(n, parents)
                return f[target_node], path
            
            close_list.add(n)
            
            for m in self.G.neighbors(n):
                g_new = g[n] + self.times_all[n][m]
                if m not in h:
                    heuristic(m, target_node)
                f_new = g_new + h[m]

                if m not in f or f_new < f[m]:
                    g[m] = g_new
                    f[m] = f_new
                    parents[m] = n
                    heapq.heappush(open_list, (f_new, m))
        
        return 0, []
    
    def get_shortest_path_astar(self, source_node, target_node):
        t, p = self.a_star(source_node, target_node)
        if not p:
            print(f'No p found from {source_node} to {target_node}!')
            return 0, p
        print(t, p)
        return t, p
  
    # Contraction Hieracrhies
    def get_node_order(self):
        node_pq = []
        for v in list(self.G.nodes()):
            val = len(list(self.G.neighbors(v))) + len(list(self.G.predecessors(v)))
            heapq.heappush(node_pq, (val, v))
        i = 1
        while node_pq:
            _, v = heapq.heappop(node_pq)
            self.node_order[i] = v
            self.order_of[v] = i
            i += 1

    def local_dijkstra_without_v(self, u, v, P_max):
        vertices = list(self.G.nodes)
        visited = set()
        pq = [(0, u)]
        D = {v: float('inf') for v in vertices}
        visited.add(v)
        D[u] = 0
        while pq:
            cost, n = heapq.heappop(pq)
            if n in visited:
                continue
            if cost > P_max:
                break
            visited.add(n)

            for neighbor in list(self.G.neighbors(n)):
                if neighbor in self.order_of:
                    continue
                old_cost = D[neighbor]
                new_cost = D[n] + self.times_all[n][neighbor]
                if new_cost < old_cost:
                    D[neighbor] = new_cost
                    heapq.heappush(pq, (new_cost, neighbor))
        return D

    def edge_difference(self, v):
        dif = - len(list(self.G.neighbors(v))) - len(list(self.G.predecessors(v)))
        for u in list(self.G.predecessors(v)):
            if u in self.order_of:
                continue
            P = {}
            for w in list(self.G.neighbors(v)):
                if w in self.order_of:
                    continue
                P[w] = self.times_all[u][v] + self.times_all[v][w]
            if not P:
                continue
            P_max = max(P.values())

            D = self.local_dijkstra_without_v(u, v, P_max)
            
            for w in list(self.G.neighbors(v)):
                if w in self.order_of:
                    continue
                if D[w] > P[w]:
                    dif += 1
        return dif
    
    def get_node_order_edge_difference(self):
        node_pq = []
        for v in list(self.G.nodes()):
            dif = self.edge_difference(v)
            heapq.heappush(node_pq, (dif, v))
        return node_pq
    
    def preprocess(self):
        node_pq = self.get_node_order_edge_difference()
        order = 0
        while node_pq:
            # Calculate edge difference again to update the pq and get the next node
            _, v = heapq.heappop(node_pq)

            new_dif = self.edge_difference(v)
            if node_pq and new_dif > node_pq[0][0]:
                heapq.heappush(node_pq, (new_dif, v))
                continue

            order += 1
            if order % 500 == 0:
                print(f"..........Contracting {order}/4397 nodes..........")
            self.order_of[v] = order
            self.node_order[order] = v

            for u in list(self.G.predecessors(v)):
                if u in self.order_of:
                    continue
                P = {}
                for w in list(self.G.neighbors(v)):
                    if w in self.order_of:
                        continue
                    P[w] = self.times_all[u][v] + self.times_all[v][w]
                if not P:
                    continue
                P_max = max(P.values())

                D = self.local_dijkstra_without_v(u, v, P_max)

                for w in list(self.G.neighbors(v)):
                    if w in self.order_of:
                        continue
                    
                    if D[w] > P[w]:
                        if self.G.has_edge(u, w):
                            self.G.get_edge_data(u, w)[0]['shortcut_node'] = v
                        else:
                            self.G.add_edge(u, w, shortcut_node=v)
                        self.times_all[u][w] = P[w]
        print('Preprocess Done!')

    def bidirectional_dijkstra(self, source_node, target_node):
        vertices = list(self.G.nodes())
        visited_start = set()
        visited_end = set()
        parents1 = {}
        parents2 = {}
        dist1 = {v: float('inf') for v in vertices}
        dist2 = {v: float('inf') for v in vertices}

        parents1[source_node] = source_node
        parents2[target_node] = target_node
        dist1[source_node] = 0
        dist2[target_node] = 0
        pq_start = [(0, source_node)]
        pq_end = [(0, target_node)]
        while pq_start or pq_end:
            if pq_start:
                _, current_vertex = heapq.heappop(pq_start)
                if current_vertex in visited_start:
                    continue
                visited_start.add(current_vertex)

                for neighbor in self.G.neighbors(current_vertex):
                    if self.order_of[neighbor] <= self.order_of[current_vertex]:
                        continue

                    new_cost = dist1[current_vertex] + self.times_all[current_vertex][neighbor]
                    if new_cost < dist1[neighbor]:
                        parents1[neighbor] = current_vertex
                        dist1[neighbor] = new_cost
                        heapq.heappush(pq_start, (new_cost, neighbor))
            if pq_end:
                _, current_vertex = heapq.heappop(pq_end)
                if current_vertex in visited_end:
                    continue
                visited_end.add(current_vertex)

                for neighbor in self.G.predecessors(current_vertex):
                    if self.order_of[neighbor] <= self.order_of[current_vertex]:
                        continue

                    new_cost = dist2[current_vertex] + self.times_all[neighbor][current_vertex]
                    if new_cost < dist2[neighbor]:
                        parents2[neighbor] = current_vertex
                        dist2[neighbor] = new_cost
                        heapq.heappush(pq_end, (new_cost, neighbor))

        L = [v for v in self.G.nodes if dist1[v] != float('inf') and dist2[v] != float('inf')]
        if not L:
            return 0, []

        shortest_time = math.inf
        common_node = 0
        for v in L:
            if shortest_time > dist1[v] + dist2[v]:
                shortest_time = dist1[v] + dist2[v]
                common_node = v

        def generate_shortcut(start_node, end_node):
            shortcut_node = self.G.get_edge_data(start_node, end_node)[0]['shortcut_node']
            if shortcut_node != 0:
                return generate_shortcut(start_node, shortcut_node) + [shortcut_node] + generate_shortcut(shortcut_node, end_node)
            else:
                return []

        shortest_path = []
        path1 = []
        cur_node = common_node

        while parents1[cur_node] != cur_node:
            tmp_node = parents1[cur_node]
            path = []
            if self.G.get_edge_data(tmp_node, cur_node)[0]['shortcut_node'] != 0:
                path = generate_shortcut(tmp_node, cur_node)
            path1 = path + path1
            path1 = [tmp_node] + path1
            cur_node = tmp_node

        cur_node = common_node
        path2 = []
        while parents2[cur_node] != cur_node:
            path2.append(cur_node)
            tmp_node = parents2[cur_node]
            path = []
            if self.G.get_edge_data(cur_node, tmp_node)[0]['shortcut_node'] != 0:
                path = generate_shortcut(cur_node, tmp_node)
            path2 += path
            cur_node = tmp_node
        path2.append(cur_node)
        
        shortest_path = path1 + path2
        return shortest_time, shortest_path

    def get_shortest_path_CH(self, source_node, target_node):
        t1 = time.time()
        self.preprocess()
        t2 = time.time()
        print("Preprocessing time:", t2 - t1)

        t, p = self.bidirectional_dijkstra(source_node, target_node)
        t3 = time.time()
        print('Query time: ', t3 - t2)
        if not p:
            print(f'No path found from {source_node} to {target_node}!')
            return 0, p
        print(t, p)
        return t, p
    
    def CH_all_pairs(self):
        t1 = time.time()
        self.preprocess()
        t2 = time.time()
        print('Preprocess time: ', t2 - t1)
        vertices = list(self.G.nodes)
        cnt = 1
        for u in vertices:
            if cnt == 16:
                break
            print(cnt)
            cnt += 1
            for v in vertices:
                t, p = self.bidirectional_dijkstra(u, v)
        t3 =  time.time()
        print(t3 - t2, (t3 - t2) / (4397*15))