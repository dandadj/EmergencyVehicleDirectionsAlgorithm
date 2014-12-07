__author__ = 'Danny Brady'

import json


def Dijkstra(graph, source):
    unvisited_intersections = []
    dist = {}
    prev = {}
    dist[source] = 0
    for v in graph['intersections']:
        n = v['name']
        if n != source:
            dist[n] = float("inf")
            prev[n] = None
        unvisited_intersections.append(v)

    current_intersection = source
    while len(unvisited_intersections) > 0:
        u = None
        current_shotest_distance = float('inf')
        for i in unvisited_intersections:
            if dist[i['name']] < current_shotest_distance:
                u = i
                current_shotest_distance = dist[i['name']]
        unvisited_intersections.remove(u)

        for neighbor_of_u in [road for road in graph['roads'] if road['source'] == u['name'] or road['target'] == u['name']]:
            neighbor_name = neighbor_of_u['target'] if neighbor_of_u['target'] != u['name'] else neighbor_of_u['source']
            neighbor = [i for i in graph['intersections'] if i['name'] == neighbor_name][0]
            alt = dist[u['name']] + linkCostDistance(u['name'], neighbor['name'], graph)
            if alt < dist[neighbor['name']]:
                dist[neighbor['name']] = alt
                prev[neighbor['name']] = u
    return (dist, prev)


def intersectionWithLeastCost(source, unvisitedIntersections, graph, costFunc):
    best_cost = float('inf')
    best_intersection = None
    for v in unvisitedIntersections:
        cost = costFunc(source, v['name'], graph)
        if cost < best_cost:
            best_cost = cost
            best_intersection = v
    return best_intersection


def linkCostDistance(source, destination, graph):
    partial = [road for road in graph['roads'] if road['source'] == source or road['target'] == source]
    link = [road for road in partial if road['source'] == destination or road['target'] == destination][0]
    return float(link['distance']) / float(link['speed'])


if __name__ == "__main__":
    g = None
    with open("RoadNetwork.json") as json_file:
        g = json.load(json_file)
    dist, prev = Dijkstra(g, "Station 8")
    print dist