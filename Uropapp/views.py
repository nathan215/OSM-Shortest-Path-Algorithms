from django.http import JsonResponse
from django.views import View
from django.shortcuts import render
import osmnx as ox
from io import BytesIO
import base64
from matplotlib.colors import to_rgba
import numpy as np
from .globals import G, init_graph
from .path_calculators import (
    dijkstras,
    nx_dijkstra,
    bidirectional_dijkstra,
    nx_bidirectional_dijkstra,
    A_star,
    nx_A_star,
)

# Convert the results to a dictionary
def results_to_dict(results):
    columns = ['own_dijkstra', 'nx_dijkstra', 'own_bidirectional_dijkstra', 'nx_bidirectional_dijkstra', 'own_A_star', 'nx_A_star']
    index = ['time_cost', 'length']
    data_dict = {key: {} for key in index}

    for method, data in results.items():
        data_dict['time_cost'][method] = data['time']
        data_dict['length'][method] = data['cost']

    return data_dict

# CalculatePathView is responsible for processing the POST request and calculating the paths using different algorithms
class CalculatePathView(View):
    def get(self, request):
        return render(request, "home.html")

    def post(self, request):
        global G, results
        start_y = float(request.POST["start_y"])
        start_x = float(request.POST["start_x"])
        end_y = float(request.POST["end_y"])
        end_x = float(request.POST["end_x"])

        # Calculate the center and radius for the bounding box
        center_x = (start_x + end_x) / 2
        center_y = (start_y + end_y) / 2
        radius = ox.distance.great_circle_vec(start_y, start_x, end_y, end_x) /1.5

        # Get graph from bounding box
        G = init_graph((center_y, center_x), radius)
        print("Graph loaded")
        # Get the start and end nodes
        start_node = ox.distance.nearest_nodes(G, X=start_x, Y=start_y)
        end_node = ox.distance.nearest_nodes(G, X=end_x, Y=end_y)

        # Calculate the paths using different algorithms
        dijkstra_result = dijkstras(G, start_node, end_node)
        nx_dijkstra_result = nx_dijkstra(G, start_node, end_node)
        bidirectional_dijkstra_result = bidirectional_dijkstra(G, start_node, end_node)
        nx_bidirectional_dijkstra_result = nx_bidirectional_dijkstra(G, start_node, end_node)
        A_star_result = A_star(G, start_node, end_node)
        nx_A_star_result = nx_A_star(G, start_node, end_node)

        # Store the results in a dictionary
        results = {
            'own_dijkstra': dijkstra_result,
            'nx_dijkstra': nx_dijkstra_result,
            'own_bidirectional_dijkstra': bidirectional_dijkstra_result,
            'nx_bidirectional_dijkstra': nx_bidirectional_dijkstra_result,
            'own_A_star': A_star_result,
            'nx_A_star': nx_A_star_result
        }
        # Convert the results to a dictionary
        index = ['time_cost', 'length']
        results_dict = {key: {} for key in index}
        for method, data in results.items():
            results_dict['time_cost'][method] = data['time']
            results_dict['length'][method] = data['cost']
        # Return the JSON response containing the results dictionary
        return JsonResponse({
            "results": results_dict
        })

# ShowPathsView is responsible for processing the POST request and rendering the selected paths on the map
class ShowPathsView(View):
    def post(self, request):
        global G, results
        # Get the selected path indexes from the POST request
        indexes = list(map(int, request.POST['indexes'].split(',')))
        # Extract the selected paths from the results dictionary
        selected_paths = [result['path'] for i, result in enumerate(results.values()) if i in indexes]
        # Create a list of paths containing nodes and edges
        paths = []
        for path in selected_paths:
            path_nodes = path
            path_edges = list(zip(path[:-1], path[1:]))
            paths.append({'nodes': path_nodes, 'edges': path_edges})
        colors = ['blue', 'red', 'green', 'orange', 'purple', 'yellow']
        alpha = 0.5
        # Plot the graph with default settings
        fig, ax = ox.plot_graph(G, node_size=0, edge_color='gray', bgcolor='white', show=False, close=False)
        # Iterate through all paths and plot them with different colors
        for i, path in enumerate(paths):
            color = colors[indexes[i]]
            route_edges = path['edges']
            ec = [to_rgba(color, alpha) if (u, v) in route_edges or (v, u) in route_edges else 'lightgray' for u, v, k in G.edges(keys=True)]
            ec_array = np.array(ec, dtype='O')
            ew = [4 if (u, v) in route_edges or (v, u) in route_edges else 0.2 for u, v, k in G.edges(keys=True)]
            ox.plot_graph(G, node_size=0, edge_color=ec_array, edge_linewidth=ew, bgcolor='white', ax=ax, show=False, close=False)

        # Save the figure to a BytesIO buffer
        buf = BytesIO()
        fig.savefig(buf, format='png')
        buf.seek(0)
        # Convert the buffer to a base64 encoded string
        image_base64 = base64.b64encode(buf.getvalue()).decode('ascii')
        buf.close()
        # Return the JSON response containing the image_base64
        response_data = {
            'image_base64': image_base64,
        }
        return JsonResponse(response_data)
