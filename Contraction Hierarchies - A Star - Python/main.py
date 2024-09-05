from Graph import *

import time
import openai
import json
import networkx as nx
from pyproj import Transformer
from rtree import *

if __name__ == '__main__':
    graph = Graph()
    time1 = time.time()
    graph.make_graph_demo()
    time2 = time.time()
    graph.get_shortest_path_CH('B', 'G')
    time3 = time.time()
    graph.get_shortest_path_astar('B', 'G')
    time4 = time.time()

    print("Time making graph: ", time2 - time1)
    print('Executing Contraction Hierachies time: ', time3 - time2)
    print('Executing A* time: ', time4 - time3)