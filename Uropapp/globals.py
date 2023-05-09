# C:\Users\natha\Urop\Uropapp\globals.py

import osmnx as ox
from osmnx import utils_geo
import networkx as nx

# Define a global graph variable to be used throughout the application
G = None

def init_graph(center, radius):
    return ox.graph_from_point(center, dist=radius, network_type='all')
