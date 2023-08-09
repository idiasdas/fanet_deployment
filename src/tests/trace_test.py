import sys
sys.path.append('..')
from import_file import *
from trace import Trace

sensors_trace = Trace(5,3,20,100)
sensors_trace.plot_trace("out/trace_plot.eps")