import os
import sys
this_dirctory = os.path.dirname(__file__)
sys.path.append( this_dirctory + '/../src/')
import numpy as np
from graph import Graph

def test_graph():
    """Tests the graph class"""
    graph = Graph(size_A=100, heights=[45], base_station=(0, 0, 0),n_positions_per_axis=3, communication_range=20, coverage_tan_angle=np.tan(np.pi/20))
    assert graph.verify_trace_feasiblity([(50,50)]) == True
    coverage = graph.get_target_coverage((50,50))
    assert len(coverage) == 1
    assert coverage[0] == (50,50,45)
    assert graph.verify_trace_feasiblity([(0,0)]) == False
    assert graph.verify_trace_feasiblity([(50,50),(0,0)]) == False
    assert graph.verify_trace_feasiblity([]) == False
