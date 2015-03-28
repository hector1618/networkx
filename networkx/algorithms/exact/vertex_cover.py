# -*- coding: utf-8 -*-
"""
************
Vertex Cover
************

Given an undirected graph `G = (V, E)` and a function w assigning nonnegative
weights to its vertices, find a minimum weight subset of V such that each edge
in E is incident to at least one vertex in the subset.

http://en.wikipedia.org/wiki/Vertex_cover
"""
#   Copyright (C) 2015 by
#   Prafullkumar P Tale <hector1618@gmail.com>
#   All rights reserved.
#   BSD license.
from networkx.utils import *
import itertools
__all__ = ["min_weighted_vertex_cover"]
__author__ = """Prafullkumar P Tale (hector1618@gmail.com)"""

def findsubsets(S,m):
    return set(itertools.combinations(S, m))

@not_implemented_for('directed')
def is_vertex_cover(G, S):
    r""" Returns true if S is vertex cover for G. False otherwise.
    """
    # Check whether all the edges are coverd or not
    for u,v in G.edges_iter():
        if u not in S and v not in S:
            return False
    return True

@not_implemented_for('directed')
def brute_force_vertex_cover(G, W, weight=None):
    r"""Returns the vertex cover of weight less than or equal to W using brute force.

    Use vertex_cover function instead.
    """
    weight_func = lambda nd: nd.get(weight, 1)
    cost = dict((n, weight_func(nd)) for n, nd in G.nodes(data=True))
    
    if W >= 0 and not G.edges():
        return {}
    
    for j in range(G.order()):
        sets = findsubsets(S, j)
        for S in sets:
            if sum([cost[s] for s in S]) < W:
                if is_vertex_cover(G, S):
                    return S

@not_implemented_for('directed')
def vertex_cover(G, W, weight=None):
    r"""Returns the vertex cover of weight less than or equal to W. 
    """
    weight_func = lambda nd: nd.get(weight, 1)
    cost = dict((n, weight_func(nd)) for n, nd in G.nodes(data=True))
    partial_sol = {}

    # Apply reduction rules exhaustively
    is_reduction_rule_applicable = True
    while is_reduction_rule_applicable:
        is_reduction_rule_applicable = False
        # Return False if W <= 0 and G still has edge which is not coverd.
        if W < 0 and not G.edges():
            return False
        # Remove all isolated vertices.
        G.remove_nodes_from([u for u, d in G.degree_iter() if d == 0])
        
        for u, d in G.nodes_iter(data=True):
            weight_of_nbd = sum[cost[v] for v in G.neighbors_iter(u)]
            # If weight of nbd exceeds the allowed budget, include u in partial solution.
            if weight_of_nbd > W:
                partial_sol += [u]
                W -= cost[u]
                G.remove_node(u)
                is_reduction_rule_applicable = True

    return partial_sol + brute_force_vertex_cover(G, W, weight=None)

@not_implemented_for('directed')
def min_weighted_vertex_cover(G, weight=None):
    r"""Exact Algorithm Minimum Weighted Vertex Cover

    Find a minimum weighted vertex cover of a graph.

    Parameters
    ----------
    G : NetworkX graph
      Undirected graph

    weight : None or string, optional (default = None)
        If None, every edge has weight/distance/cost 1. If a string, use this
        edge attribute as the edge weight. Any edge attribute not present
        defaults to 1.

    Returns
    -------
    min_weighted_cover : set
      Returns a set of vertices whose weight sum is no more than 2 * OPT.

    Notes
    -----
    Local-Ratio algorithm for computing an approximate vertex cover.
    Algorithm greedily reduces the costs over edges and iteratively
    builds a cover. Worst-case runtime is `O(|E|)`.

    References
    ----------
    .. [1] Bar-Yehuda, R., & Even, S. (1985). A local-ratio theorem for
       approximating the weighted vertex cover problem.
       Annals of Discrete Mathematics, 25, 27–46
       http://www.cs.technion.ac.il/~reuven/PDF/vc_lr.pdf
    """
    weight_func = lambda nd: nd.get(weight, 1)
    cost = dict((n, weight_func(nd)) for n, nd in G.nodes(data=True))

    # while there are edges uncovered, continue
    for u,v in G.edges_iter():
        # select some uncovered edge
        min_cost = min([cost[u], cost[v]])
        cost[u] -= min_cost
        cost[v] -= min_cost

    return set(u for u in cost if cost[u] == 0)
