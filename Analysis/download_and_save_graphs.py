import osmnx as ox
import pickle

def download_graph(center, radius, city_name):
    G = ox.graph_from_point(center, dist=radius, network_type='drive')
    with open(f"{city_name}_graph.pkl", "wb") as f:
        pickle.dump(G, f)
    print(G.number_of_nodes(), G.number_of_edges())

locations = [
    ("Taipei", (25.046179, 121.517437)),
    ("Taichung", (24.162078, 120.649230)),
    ("Kaohsiung", (22.620710, 120.312467)),
    ("Pingtung", (22.003684,120.747165)),
    ("Hualien", (23.335521, 121.315165)),
    ("Taitung", (23.122363, 121.215128))
]

radius = 6000

for city_name, center in locations:
    download_graph(center, radius, city_name)
    print(f"{city_name} graph downloaded and saved.")
    
