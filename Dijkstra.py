__author__ = 'Danny Brady'
import json

MEDIAN_ADJUSTMENT_FACTOR = {}
MEDIAN_ADJUSTMENT_FACTOR['soft'] = 0.8
MEDIAN_ADJUSTMENT_FACTOR['no'] = 0.9
MEDIAN_ADJUSTMENT_FACTOR['hard'] = 1

CONTROL_DEVICE_ADJUSTMENT_FACTOR = {}
CONTROL_DEVICE_ADJUSTMENT_FACTOR['preemptionSignal'] = 0.6
CONTROL_DEVICE_ADJUSTMENT_FACTOR['signal'] = 0.8
CONTROL_DEVICE_ADJUSTMENT_FACTOR['stop'] = 1


def Dijkstra(graph, source, costFunction):
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
            neighbor = intersectionFromName(neighbor_name, graph)
            alt = dist[u['name']] + costFunction(u['name'], neighbor['name'], graph)
            if alt < dist[neighbor['name']]:
                dist[neighbor['name']] = alt
                prev[neighbor['name']] = u
    return (dist, prev)


def intersectionFromName(name, graph):
    return [i for i in graph['intersections'] if i['name'] == name][0]


### Cost Functions ###

def linkCostDistance(source, destination, graph):
    partial = [road for road in graph['roads'] if road['source'] == source or road['target'] == source]
    link = [road for road in partial if road['source'] == destination or road['target'] == destination][0]
    return float(link['distance'])


def linkCostSpeedWeighted(source, destination, graph):
    partial = [road for road in graph['roads'] if road['source'] == source or road['target'] == source]
    link = [road for road in partial if road['source'] == destination or road['target'] == destination][0]
    return float(link['distance']) / float(link['speed'])


def linkCostEmergency(source, destination, graph):
    partial = [road for road in graph['roads'] if road['source'] == source or road['target'] == source]
    link = [road for road in partial if road['source'] == destination or road['target'] == destination][0]
    cost = float(link['distance']) / float(link['speed']) * (8 / float(link['lanes'])) * MEDIAN_ADJUSTMENT_FACTOR[link['median']]
    source_intersection = intersectionFromName(source, graph)
    destination_intersection = intersectionFromName(destination, graph)
    cost = cost * CONTROL_DEVICE_ADJUSTMENT_FACTOR[source_intersection['controlType']] * \
           CONTROL_DEVICE_ADJUSTMENT_FACTOR[destination_intersection['controlType']]
    return cost


if __name__ == "__main__":
    g = None
    with open("RoadNetwork.json") as json_file:
        g = json.load(json_file)
    #dist, prev = Dijkstra(g, "Station 8", linkCostDistance)
    #dist, prev = Dijkstra(g, "Station 8", linkCostSpeedWeighted)
    dist, prev = Dijkstra(g, "Station 8", linkCostEmergency)
    dest_intersection = "Greenbrier Dr @ Whitewood Rd"
    #dest_intersection = "Berkmar Dr @ Hilton Heights Rd"
    path = []
    path.append(intersectionFromName(dest_intersection, g))
    while dest_intersection != "Station 8":
        p = prev[dest_intersection]
        path.append(p)
        dest_intersection = p['name']
    path.reverse()
    for p in path:
        print p