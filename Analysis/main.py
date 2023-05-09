import osmnx as ox
import networkx as nx
import pickle
import random
import time
import heapq
import geopy.distance
from datetime import datetime
import csv


def crop_graph(G, start_node, end_node):
    start_x, start_y = G.nodes[start_node]['x'], G.nodes[start_node]['y']
    end_x, end_y = G.nodes[end_node]['x'], G.nodes[end_node]['y']

    center_x = (start_x + end_x) / 2
    center_y = (start_y + end_y) / 2
    radius = ox.distance.great_circle_vec(start_y, start_x, end_y, end_x) / 1.5

    def is_within_circle(node, center_x, center_y, radius):
        node_x, node_y = node['x'], node['y']
        return ox.distance.great_circle_vec(node_y, node_x, center_y, center_x) <= radius

    cropped_nodes = [node for node in G.nodes if is_within_circle(G.nodes[node], center_x, center_y, radius)]
    return G.subgraph(cropped_nodes)

def load_graph(city_name):
    with open(f"{city_name}_graph.pkl", "rb") as f:
        G = pickle.load(f)
    print("finish load graph")
    return G

def select_point_short(G, center, radius=4000, distance_threshold=1000, num_pairs=100):
    point_pairs = []
    while len(point_pairs) < num_pairs:
        random_node_1 = random.choice(list(G.nodes))
        random_node_2 = random.choice(list(G.nodes))
        point1 = (G.nodes[random_node_1]['y'], G.nodes[random_node_1]['x'])
        point2 = (G.nodes[random_node_2]['y'], G.nodes[random_node_2]['x'])
        center_distance_1 = geopy.distance.distance(center, point1).m
        center_distance_2 = geopy.distance.distance(center, point2).m
        distance = geopy.distance.distance(point1, point2).m
        if distance <= distance_threshold and center_distance_1 <= radius and center_distance_2 <= radius and distance>100 and nx.has_path(G, random_node_1, random_node_2):
            point_pairs.append((random_node_1, random_node_2))
    return point_pairs

def select_point_long(G, center, distance_threshold=5000, num_pairs=100):
    point_pairs = []
    while len(point_pairs) < num_pairs:
        random_node_1 = random.choice(list(G.nodes))
        random_node_2 = random.choice(list(G.nodes))
        point1 = (G.nodes[random_node_1]['y'], G.nodes[random_node_1]['x'])
        point2 = (G.nodes[random_node_2]['y'], G.nodes[random_node_2]['x'])
        distance = geopy.distance.distance(point1, point2).m
        if distance >= distance_threshold and nx.has_path(G, random_node_1, random_node_2):
            point_pairs.append((random_node_1, random_node_2))
    return point_pairs

def dijkstras(G, start_node, end_node):
    start_time = time.perf_counter()
    costs = {node: float('inf') for node in G.nodes}
    paths = {node: [] for node in G.nodes}
    costs[start_node] = 0
    paths[start_node] = [start_node]
    visited = set()
    heap = [(costs[start_node], start_node)]
    while heap:
        (cost, node) = heapq.heappop(heap)
        visited.add(node)
        
        for neighbor, data in G[node].items():
            if neighbor not in visited:
                new_cost = cost + data[0]['length']
                if new_cost < costs[neighbor]:
                    costs[neighbor] = new_cost
                    paths[neighbor] = paths[node] + [neighbor]
                    heapq.heappush(heap, (new_cost, neighbor))
    elapsed_time = time.perf_counter() - start_time
    return {"time": elapsed_time, "cost": costs[end_node], "path": paths[end_node]}

def nx_dijkstra(G, start_node, end_node):
    start_time = time.perf_counter()
    length, path = nx.single_source_dijkstra(G, start_node, end_node, weight='length')
    elapsed_time = time.perf_counter() - start_time
    return {"time": elapsed_time, "cost": length, "path": path}

def bidirectional_dijkstra(G, start_node, end_node):
    start_time = time.perf_counter()
    forward_costs = {node: float('inf') for node in G.nodes}
    backward_costs = {node: float('inf') for node in G.nodes}
    forward_paths = {node: [] for node in G.nodes}
    backward_paths = {node: [] for node in G.nodes}
    forward_costs[start_node] = 0
    forward_paths[start_node] = [start_node]
    backward_costs[end_node] = 0
    backward_paths[end_node] = [end_node]
    forward_visited = set()
    backward_visited = set()
    forward_heap = [(0, start_node)]
    backward_heap = [(0, end_node)]
    best_cost = float('inf')
    best_path = None
    while forward_heap and backward_heap:
        forward_cost, forward_node = heapq.heappop(forward_heap)
        forward_visited.add(forward_node)
        for f_neighbor, f_data in G[forward_node].items():
            if f_neighbor not in forward_visited:
                f_cost = f_data[0]['length'] + forward_cost
                if f_cost < forward_costs[f_neighbor]:
                    forward_costs[f_neighbor] = f_cost
                    forward_paths[f_neighbor] = forward_paths[forward_node] + [f_neighbor]
                    heapq.heappush(forward_heap, (f_cost, f_neighbor))
        if forward_node in backward_visited:
            total_cost = forward_costs[forward_node] + backward_costs[forward_node]
            if total_cost < best_cost:
                best_cost = total_cost
                best_path = forward_paths[forward_node] + backward_paths[forward_node][::-1]
        backward_cost, backward_node = heapq.heappop(backward_heap)
        backward_visited.add(backward_node)
        for b_neighbor, b_data in G[backward_node].items():
            if b_neighbor not in backward_visited:
                b_cost = b_data[0]['length'] + backward_cost
                if b_cost < backward_costs[b_neighbor]:
                    backward_costs[b_neighbor] = b_cost
                    backward_paths[b_neighbor] = backward_paths[backward_node] + [b_neighbor]
                    heapq.heappush(backward_heap, (b_cost, b_neighbor))
        if backward_node in forward_visited:
                total_cost = forward_costs[backward_node] + backward_costs[backward_node]
                if total_cost < best_cost:
                    best_cost = total_cost
                    best_path = forward_paths[backward_node] + backward_paths[backward_node][::-1]
        if forward_heap and backward_heap and (forward_heap[0][0] + backward_heap[0][0]) >= best_cost:
            break
    elapsed_time = time.perf_counter() - start_time
    return {'time': elapsed_time, 'cost': best_cost, 'path': best_path}

def nx_bidirectional_dijkstra(G, start_node, end_node):
    start_time = time.perf_counter()
    length, path = nx.bidirectional_dijkstra(G, start_node, end_node, weight='length')
    elapsed_time = time.perf_counter() - start_time
    return {'time': elapsed_time, 'cost': length, 'path': path}

def A_star(G, start_node, end_node):
    def heuristic(node):
        y1, x1 = G.nodes[node]['y'], G.nodes[node]['x']
        y2, x2 = G.nodes[end_node]['y'], G.nodes[end_node]['x']
        return ox.distance.great_circle_vec(y1, x1, y2, x2)

    start_time = time.perf_counter()
    heuristic_costs = {}
    costs = {node: float('inf') for node in G.nodes()}
    paths = {node: [] for node in G.nodes()}
    costs[start_node] = 0
    heap = [(heuristic(start_node), start_node)] 
    path = []
    visited = set()
    while heap:
        (cost, node) = heapq.heappop(heap)
        visited.add(node)
        if node == end_node:
            path = paths[node] + [node]
            break
        for neighbor, data in G[node].items():
            if neighbor not in visited:
                new_cost = costs[node] + data[0]['length'] 
                if new_cost < costs[neighbor]:
                    costs[neighbor] = new_cost
                    paths[neighbor] = paths[node] + [neighbor]
                    if neighbor not in heuristic_costs:
                        heuristic_costs[neighbor] = heuristic(neighbor)
                    heapq.heappush(heap, (new_cost + heuristic_costs[neighbor], neighbor))
    elapsed_time = time.perf_counter() - start_time
    return {'time': elapsed_time, 'cost': costs[end_node], 'path': path}

def nx_A_star(G, start_node, end_node):
    def heuristic(node1, node2):
        y1, x1 = G.nodes[node1]['y'], G.nodes[node1]['x']
        y2, x2 = G.nodes[node2]['y'], G.nodes[node2]['x']
        return ox.distance.great_circle_vec(y1, x1, y2, x2)

    start_time = time.perf_counter()
    path = nx.astar_path(G, start_node, end_node, weight='length', heuristic=lambda u, v: heuristic(u, end_node))
    length = sum(G[path[i]][path[i + 1]][0]['length'] for i in range(len(path) - 1))
    elapsed_time = time.perf_counter() - start_time
    return { 'time': elapsed_time, 'cost': length, 'path': path}

def run_algorithms(points, G, size="large", city="", mode=""):
    algorithms = [bidirectional_dijkstra , nx_bidirectional_dijkstra,  dijkstras, nx_dijkstra , A_star, nx_A_star]
    results = {}
    print("Running algorithms for " + city + " " + mode)
    for algo in algorithms:
        results[algo.__name__] = { "costs": [], "times": []}

    for point_pair in points:
        if size == "small":
            start_x, start_y = G.nodes[point_pair[0]]['x'], G.nodes[point_pair[0]]['y']
            end_x, end_y = G.nodes[point_pair[1]]['x'], G.nodes[point_pair[1]]['y']
            center_x = (start_x + end_x) / 2
            center_y = (start_y + end_y) / 2
            radius = ox.distance.great_circle_vec(start_y, start_x, end_y, end_x) 
            G_cropped = ox.graph_from_point((center_y,center_x), dist=radius, network_type='drive')
        else:
            G_cropped = G

        temp_results = {}
        for algo in algorithms:
            try:
                result = algo(G_cropped, point_pair[0], point_pair[1])
            except Exception as e:
                print(e)
                break
            else:
                if result['cost'] == float('inf'):
                    print("inf")
                    break
                temp_results[algo.__name__] = result

        if len(temp_results) == len(algorithms):
            for algo_name, result in temp_results.items():
                results[algo_name]["costs"].append(result["cost"])
                results[algo_name]["times"].append(result["time"])


    # Generate a timestamped filename for the CSV file
    csv_filename = f"algorithm_results_{city}_{mode}.csv"

    # Write times to a CSV file
    with open(csv_filename, "w", newline="") as csvfile:
        fieldnames = ['point_index'] + [f"{algo.__name__}_time" for algo in algorithms] + [f"{algo.__name__}_cost" for algo in algorithms]
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()

        for i in range(len(points)):
            row = {"point_index": i}
            for algo in algorithms:
                algo_name = algo.__name__
                try:
                    row[f"{algo_name}_time"] = results[algo_name]["times"][i]
                    row[f"{algo_name}_cost"] = results[algo_name]["costs"][i]
                except IndexError:
                    continue
            writer.writerow(row)

    return results

G_Taipei = load_graph("Taipei")
G_Taichung = load_graph("Taichung")
G_Kaohsiung = load_graph("Kaohsiung")
G_Pingtung = load_graph("Pingtung")
G_Hualien = load_graph("Hualien")
G_Taitung = load_graph("Taitung")
graphs = [
    ("Taipei", (25.046179, 121.517437), G_Taipei),
    ("Taichung", (24.162078, 120.649230), G_Taichung),
    ("Kaohsiung", (22.620710, 120.312467), G_Kaohsiung),
    ("Pingtung", (22.003684, 120.747165), G_Pingtung),
    ("Hualien", (23.335521, 121.315165), G_Hualien),
    ("Taitung", (23.122363, 121.215128), G_Taitung)
]

complex_graphs = [G_Taipei, G_Taichung, G_Kaohsiung]
single_graphs = [G_Pingtung, G_Hualien, G_Taitung]

modes = ["long_distance", "short_distance", "short_distance_small_graph"]


for graph_name, coord, G in graphs:
    pointsl = select_point_long(G, coord)
    pointss = select_point_short(G, coord)
    for mode in modes:
        size = "small" if mode == "short_distance_small_graph" else "large"
        result = run_algorithms(
            pointsl if mode == "long_distance" else pointss,
            G,
            size=size,
            city=graph_name,
            mode=mode,
        )
    print(graph_name, "finish")


