import osmnx as ox


def road_network_complexity(center_coord, radius=60000):

    G = ox.graph_from_point(center_coord, dist=radius, network_type='drive')
    print("finish download graph")
    num_nodes = len(G.nodes)
    num_edges = len(G.edges)

    return num_nodes, num_edges


#  Taipei = x: 121.517437, y: 25.046179  台北車站
#  Taichung = x: 120.649230, y: 24.162078 台中市政府
#  Kaohsiung = x: 120.322523, y: 22.640474 高雄市政府
#  Pingtung = 120.7471649 , 22.0036839 恆春鄉公所
#  Taitung= x: 121.215128, y: 23.122363 池上鄉公所
#  Hualien = (23.335521007664067, 121.31516510053311) 玉里鎮公所

print("Hualien: ", road_network_complexity((23.335521007664067, 121.31516510053311)))  # (9724, 25134)
print("Pingtung: ", road_network_complexity((22.003684,120.747165))) # (6683, 17061)
print("Taitung: ", road_network_complexity((23.122363, 121.215128)))  # (10618, 27122)

print("Taipei: ", road_network_complexity((25.046179, 121.517437)))  # (72749, 180611)
print("Kaohsiung: ", road_network_complexity((22.620710, 120.312467))) #(50862, 136478)
print("Taichung: ", road_network_complexity((24.162078, 120.649230))) # (73729, 190617)


