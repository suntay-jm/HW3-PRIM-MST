import numpy as np
import heapq
from typing import Union

class Graph:

    def __init__(self, adjacency_mat: Union[np.ndarray, str]):
        """
    
        Unlike the BFS assignment, this Graph class takes an adjacency matrix as input. `adjacency_mat` 
        can either be a 2D numpy array of floats or a path to a CSV file containing a 2D numpy array of floats.

        In this project, we will assume `adjacency_mat` corresponds to the adjacency matrix of an undirected graph.
    
        """
        if type(adjacency_mat) == str:
            self.adj_mat = self._load_adjacency_matrix_from_csv(adjacency_mat)
        elif type(adjacency_mat) == np.ndarray:
            self.adj_mat = adjacency_mat
        else: 
            raise TypeError('Input must be a valid path or an adjacency matrix')
        self.mst = None

    def _load_adjacency_matrix_from_csv(self, path: str) -> np.ndarray:
        with open(path) as f:
            return np.loadtxt(f, delimiter=',')

    def construct_mst(self):
        # add edges from starting node to heap
        # empty list for heap
        
        self.mst = np.zeros_like(self.adj_mat) # create an empty adjacency matrix with the same shape as self.adj_mat
        visited = set() # initialize the visited set and add the starting node to it
        visited.add(0) # can pick any start node, but starting with 0
        h = []


        # looping through neighbors of the starting node (0)
        for neighbor, weight in enumerate(self.adj_mat[0]): # enumerate for each neighbor + edge weight
          if weight > 0:
            # heappush uses the heap to push to and (weight, from_node, to_node)
            heapq.heappush(h, (weight, 0, neighbor))
        
        # process edges from heap
        while h:  # while there are still edges in the heap
          # pop the smallest edge from the heap
          # at this point h = [(5.0, 0, 1), (5.0, 0, 3)]
          weight, from_node, to_node = heapq.heappop(h)  # weight = 5, from_node = 0, to_node = 1
        
          # check if to_node is already visited
          if to_node in visited:  
              continue  # skip if node is in visited 
        
          # add edge to MST
          self.mst[from_node, to_node] = weight  
          self.mst[to_node, from_node] = weight  
        
          # mark the node as visited
          visited.add(to_node)  
        
          # add new edges from to_node to heap
          for neighbor, new_weight in enumerate(self.adj_mat[to_node]):  
              if new_weight > 0 and neighbor not in visited:  
                  heapq.heappush(h, (new_weight, to_node, neighbor))  
