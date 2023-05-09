import osmnx as ox
import networkx as nx
import time
import heapq
import math

def dijkstras(G, start_node, end_node):
    start_time = time.time()
    # Initialize the costs and paths dictionaries for each node in G.
    costs = {node: float('inf') for node in G.nodes}
    paths = {node: [] for node in G.nodes}
    # Set the cost of the start node to 0 and the path to just the start node.
    costs[start_node] = 0
    paths[start_node] = [start_node]
    # Initialize the visited set to keep track of visited nodes.
    visited = set()
    # Initialize the heap to store the nodes in order of increasing cost.
    heap = [(costs[start_node], start_node)]
    # Start the Dijkstra's algorithm.
    while heap:
        # Extract the node with the lowest cost from the heap.
        (cost, node) = heapq.heappop(heap)
        # Mark the current node as visited.
        visited.add(node)
        # Check all the neighbor nodes that have not been visited yet.
        for beside_node in G.adj[node]:
            # Get the cost of reaching the neighbor node from the current node.
            neighbor_cost = G.adj[node][beside_node][0]['length']
            # If the neighbor node has not been visited yet.
            if beside_node not in visited:
                # Get the current cost and the new cost of reaching the neighbor node.
                cur_cost = costs[beside_node]
                new_cost = cost + neighbor_cost
                # If the new cost is less than the current cost, update the cost and path dictionaries.
                if new_cost < cur_cost:
                    costs[beside_node] = new_cost
                    paths[beside_node] = paths[node] + [beside_node]
                    # Add the updated neighbor node to the heap.
                    heapq.heappush(heap, (new_cost, beside_node))
    elapsed_time = time.time() - start_time
    return {"time": elapsed_time, "cost": costs[end_node], "path": paths[end_node]}

def nx_dijkstra(G, start_node, end_node):
    start_time = time.time()
    length, path = nx.single_source_dijkstra(G, start_node, end_node, weight='length')
    elapsed_time = time.time() - start_time
    return {"time": elapsed_time, "cost": length, "path": path}

def bidirectional_dijkstra(G, start_node, end_node):

    start_time = time.time()
    # Initialize the costs and paths dictionarys for the forward and backward searches.
    forward_costs = {node: float('inf') for node in G.nodes}
    backward_costs = {node: float('inf') for node in G.nodes}
    forward_paths = {node: [] for node in G.nodes}
    backward_paths = {node: [] for node in G.nodes}
    # Set the cost of the start node to 0 for the forward search and the end node to 0 for the backward search.
    forward_costs[start_node] = 0
    backward_costs[end_node] = 0
    # Initialize sets to keep track of visited nodes for the forward and backward searches.
    forward_visited = set()
    backward_visited = set()
    # Initialize the heap for the forward and backward searches.
    forward_heap = [(0, start_node)]
    backward_heap = [(0, end_node)]
    # Initialize variables to keep track of the best cost and best path found so far.
    best_cost = float('inf')
    best_path = None
    # Start the bidirectional Dijkstra's algorithm.
    while forward_heap and backward_heap:
        # Extract the node with the lowest cost from the forward heap and check its neighbors.
        forward_cost, forward_node = heapq.heappop(forward_heap)
        if forward_node not in forward_visited:
            forward_visited.add(forward_node)
            for neighbor, data in G[forward_node].items():
                # Calculate the cost of reaching the neighbor node from the current node.
                cost = data[0]['length'] + forward_cost
                if cost < forward_costs[neighbor]:
                    # Update the cost and path for the neighbor node if the new cost is lower than the previous cost.
                    forward_costs[neighbor] = cost
                    forward_paths[neighbor] = forward_paths[forward_node] + [forward_node]
                    # Add the updated neighbor node to the forward heap.
                    heapq.heappush(forward_heap, (cost, neighbor))
            if forward_node in backward_visited:
                # If the forward node has been visited in the backward search, combine the paths and check if it is the best path found so far.
                path = forward_paths[forward_node] + [forward_node] + backward_paths[forward_node][::-1]
                cost = forward_costs[forward_node] + backward_costs[forward_node]
                if cost < best_cost:
                    best_cost = cost
                    best_path = path

        # Extract the node with the lowest cost from the backward heap and check its neighbors.
        backward_cost, backward_node = heapq.heappop(backward_heap)
        if backward_node not in backward_visited:
            backward_visited.add(backward_node)
            for neighbor, data in G[backward_node].items():
                # Calculate the cost of reaching the neighbor node from the current node.
                cost = data[0]['length'] + backward_cost
                if cost < backward_costs[neighbor]:
                    # Update the cost and path for the neighbor node if the new cost is lower than the previous cost.
                    backward_costs[neighbor] = cost
                    backward_paths[neighbor] = backward_paths[backward_node] + [backward_node]
                    # Add the updated neighbor node to the backward heap.
                    heapq.heappush(backward_heap, (cost, neighbor))

            if backward_node in forward_visited:
                # If the backward node has been visited in the forward search, combine the paths and check if it is the best path found so far.
                path = forward_paths[backward_node] + [backward_node] + backward_paths[backward_node][::-1]
                cost = forward_costs[backward_node] + backward_costs[backward_node]
                if cost < best_cost:
                    best_cost = cost
                    best_path = path
        # Check if the minimum cost found in both the forward and backward searches exceeds the current best cost.
        if forward_heap[0][0] + backward_heap[0][0] >= best_cost:
            # If the minimum cost exceeds the best cost, stop the algorithm and return the current best path.
            break
    elapsed_time = time.time() - start_time
    return {'time': elapsed_time, 'cost': best_cost, 'path': best_path}




def nx_bidirectional_dijkstra(G, start_node, end_node):
    start_time = time.time()
    length, path = nx.bidirectional_dijkstra(G, start_node, end_node, weight='length')
    elapsed_time = time.time() - start_time
    return {'time': elapsed_time, 'cost': length, 'path': path}

def A_star(G, start_node, end_node):
    start_time = time.time()
   # Define the heuristic function, which estimates the cost of the cheapest path from a node to the end_node
    def heuristic(node):
        return abs(G.nodes[node]['x'] - G.nodes[end_node]['x']) + abs(G.nodes[node]['y'] - G.nodes[end_node]['y'])

    # Initialize the costs and paths dictionaries for each node in G
    costs = {node: float('inf') for node in G.nodes()}
    paths = {node: [] for node in G.nodes()}
    # Set the cost of the start node to 0 and the path to an empty list
    costs[start_node] = 0
    heap = [(0, start_node)]
    path = []

    # Start the A* algorithm
    while heap:
        # Extract the node with the lowest cost from the heap
        current_cost, current_node = heapq.heappop(heap)
        # Check if the current node is the end_node, and if so, reconstruct the path and break the loop
        if current_node == end_node:
            while paths[current_node]:
                path.insert(0, paths[current_node])
                current_node = paths[current_node]
            path.insert(0, current_node)
            break
        # Check all the neighbor nodes of the current node
        for neighbor, data in G[current_node].items():
            # Calculate the cost of reaching the neighbor node from the current node
            cost = data[0]['length']
            # Estimate the cost of the cheapest path from the neighbor node to the end_node
            heuristic_cost = heuristic(neighbor)
            # Calculate the total cost of the path from the start_node to the neighbor node through the current node
            total_cost = current_cost + cost + heuristic_cost
            # Update the cost and path dictionaries for the neighbor node if the new cost is lower than the previous cost
            if total_cost < costs[neighbor]:
                costs[neighbor] = total_cost
                paths[neighbor] = current_node
                # Add the neighbor node to the heap with the total cost as the priority
                heapq.heappush(heap, (total_cost, neighbor))

    elapsed_time = time.time() - start_time
    return {'time': elapsed_time, 'cost': 'Infinity' if math.isinf(costs[end_node]) else costs[end_node], 'path': path}

def nx_A_star(G, start_node, end_node):
    start_time = time.time()
    path = nx.astar_path(G, start_node, end_node, weight='length')
    length = sum(G[path[i]][path[i + 1]][0]['length'] for i in range(len(path) - 1))
    elapsed_time = time.time() - start_time
    return { 'time': elapsed_time, 'cost': length, 'path': path}