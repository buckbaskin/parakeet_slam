#!/usr/bin/env python

print('\n...\n')

# import sys; print(sys.path)
# print('/home/buck/miniconda2/envs/prkt_env/lib/python2.7/site-packages/' in sys.path)

import scipy
from scipy.stats import multivariate_normal

print('\n...\n')

import sys
sys.path.append('/home/buck/ros_ws/src')

print('\n...\n')

from viz_feature_sim.msg import VizScan, Blob

print('\n...\n')

import crispy_parakeet
from crispy_parakeet.src import prkt_core_v2

print('\n...\n')