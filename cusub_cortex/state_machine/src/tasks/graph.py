#!/usr/bin/env python

import networkx as nx
import numpy as np
import matplotlib.pyplot as plt
"""
Graph functions to assist with circling points

I want to pass in a list of points, a target index and a starting index, give me the indices needed to reach there in order
"""

def graphGetPath(target_pt, start_pt, path_pts):
    # Okay let's make a graph first
    # add the target pt first
    # add the path list closest to it & connect
    # connect the rest of the path pts in O(n^2)
    # connect the starting pt
    # Run depth first search from start to target
    # return those points
    G = nx.Graph()
    G.add_node(target_pt)
    G.add_node(start_pt)
    G.add_nodes_from(path_pts)
    closest_pt = graphFindClosestPt(target_pt, path_pts + [start_pt])
    G.add_edge(target_pt, closest_pt)
    plt.subplot(111)
    nx.draw(G, with_labels=True, font_weight="bold")
    plt.show()
    
    G = graphConnectPts(G, path_pts)
    closest_pt = graphFindClosestPt(start_pt, path_pts)
    G.add_edge(start_pt, closest_pt)
    plt.subplot(111)
    nx.draw(G, with_labels=True, font_weight="bold")
    plt.show()
    
    print(nx.shortest_path(G, start_pt, target_pt))

def graphConnectPts(graph, path_pts):
    for i in range(len(path_pts)):
        
        minPt = None
        minDist = np.Inf
        minPt2 = None
        minDist2 = np.Inf

        vec1 = np.array([path_pts[i].x,path_pts[i].y], dtype=np.float32)
        for j in range(i, len(path_pts)):
            if i == j:
                continue
            vec2 = np.array([path_pts[j].x, path_pts[j].y], dtype=np.float32)
            diff = vec1 - vec2
            dist = np.linalg.norm(diff)
            if dist < minDist:
                minPt2 = minPt
                minDist2 = minDist
                minPt = path_pts[j]
                minDist = dist
            elif dist < minDist2:
                minPt2 = path_pts[j]
                minDist2 = dist
        num_neighbors = len(list(graph.neighbors(path_pts[i])))
        if num_neighbors == 0:
            graph.add_edge(path_pts[i], minPt)
            graph.add_edge(path_pts[i], minPt2)
        elif num_neighbors == 1:
            graph.add_edge(path_pts[i], minPt)
    return graph

def graphFindClosestPt(target_pt, other_pts):
    vec1 = np.array([target_pt.x, target_pt.y], dtype=np.float32)
    dists=np.zeros((len(other_pts),1))
    for i in range(len(other_pts)):
        vec2 = np.array([other_pts[i].x,other_pts[i].y], dtype=np.float32)
        diff = vec1 - vec2
        dists[i] = np.linalg.norm(diff)
    return other_pts[np.argmin(dists)]
