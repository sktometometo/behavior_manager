# -*- coding: utf-8 -*-

import networkx as nx


class GraphEdge:

    def __init__(self,
                 node_id_from=None,
                 node_id_to=None,
                 behavior_type=None,
                 args=None
                 ):
        self.node_id_from = node_id_from
        self.node_id_to = node_id_to
        self.behavior_type = behavior_type
        self.args = args

    def __str__(self):
        'from: \'{}\', to:\'{}\', behavior_type:\'{}\', args:{}'.format(
            self.node_id_from, self.node_id_to, self.behavior_type, self.args)

    def is_empty(self):

        return self.behavior_type is None


class GraphNode:

    def __init__(self,
                 node_id=None,
                 properties=None
                 ):
        self.node_id = node_id
        self.properties = properties

    def __str__(self):
        'node_id:{}, properties:{}'.format(self.node_id, self.properties)

    def is_empty(self):

        return self.node_id is None


class BehaviorGraph:

    def __init__(self, edge_dict={}, node_dict={}):

        self.edges = {}
        self.nodes = {}
        self.network = nx.DiGraph()

        for key, edge in edge_dict.items():
            self.add_edge(edge)

        for key, node in node_dict.items():
            self.add_node(node)

    def calcPath(self, node_id_from, node_id_to):

        try:
            node_id_list = nx.shortest_path(
                self.network, node_id_from, node_id_to)
        except nx.NetworkXNoPath as e:
            return None, '{}'.format(e)
        path = []
        for index in range(len(node_id_list)-1):
            path.append(self.edges[node_id_list[index], node_id_list[index+1]])
        return path, 'success'

    def add_node(self,
                 graph_node):
        self.nodes[graph_node.node_id] = graph_node

    def add_edge(self,
                 graph_edge):
        self.edges[graph_edge.node_id_from, graph_edge.node_id_to] = graph_edge
        self.network.add_edge(graph_edge.node_id_from,
                              graph_edge.node_id_to)

    def remove_node(self,
                    node_id):
        del self.nodes[node_id]

    def remove_edge(self,
                    node_id_from,
                    node_id_to):
        del self.edges[node_id_from, node_id_to]
        self.network.remove_edge(node_id_from, node_id_to)
